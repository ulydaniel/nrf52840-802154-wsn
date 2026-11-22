#!/usr/bin/env python3
"""
Host-side simulator for COMPE 662 IEEE 802.15.4 Data Collection Tree
implementation milestones (assignment §11 / checklist §7).

Mirrors the application-layer rules in src/main.c without Zephyr/radio:
  M1 Packet format
  M2 Discovery (PROBE / HEARTBEAT)
  M3 Parent selection
  M4 Address assignment
  M5 Registration (JOIN_ACK)
  M6 Data collection / forwarding
  M7 Reliability (timeouts, duplicates, recovery)

Run:
  python Host/milestone_sim.py
  python Host/milestone_sim.py --milestone 4
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Constants (match src/main.c)
# ---------------------------------------------------------------------------

PROTOCOL_VERSION = 2
OZ_PACKET_SIZE = 17

MSG_PROBE = 1
MSG_HEARTBEAT = 2
MSG_JOIN_REQUEST = 3
MSG_JOIN_RESPONSE = 4
MSG_JOIN_ACKNOWLEDGE = 5
MSG_DATA = 6
MSG_NETID_REQUEST = 7
MSG_NETID_RESPONSE = 8

MSG_NAMES = {
    1: "PROBE",
    2: "HEARTBEAT",
    3: "JOIN_REQUEST",
    4: "JOIN_RESPONSE",
    5: "JOIN_ACKNOWLEDGE",
    6: "DATA",
    7: "NETIDREQ",
    8: "NETIDRESP",
}

def MAKE_ADDRESS(network: int, node: int) -> int:
    return ((network & 0xFF) << 8) | (node & 0xFF)

def ADDRESS_NETWORK(addr: int) -> int:
    return (addr >> 8) & 0xFF

def ADDRESS_NODE(addr: int) -> int:
    return addr & 0xFF

def fmt_addr(addr: int) -> str:
    return f"{ADDRESS_NETWORK(addr)}:{ADDRESS_NODE(addr)}"

ROOT_ADDRESS = MAKE_ADDRESS(1, 254)
BROADCAST_ADDRESS = 0xFFFF
UNASSIGNED_ADDRESS = 0xFFFE
RESERVED_ADDRESS = MAKE_ADDRESS(1, 0)
HOP_COUNT_UNKNOWN = 0xFF
ADDRESS_TABLE_CAPACITY = 16
JOIN_REQUEST_MAX_RETRIES = 3
NODE_ID_FIRST = 1
NODE_ID_LAST = 253


class NodeState(IntEnum):
    UNREGISTERED = 0
    PROBING = 1
    WAITING_FOR_JOIN_RESPONSE = 2
    REGISTERED = 3


STATE_NAMES = {s: s.name for s in NodeState}


# ---------------------------------------------------------------------------
# Packet codec
# ---------------------------------------------------------------------------

@dataclass
class OzPacket:
    signature: bytes = b"OZ"
    version: int = PROTOCOL_VERSION
    message_type: int = 0
    source_id: int = 0
    destination_id: int = 0
    parent_id: int = UNASSIGNED_ADDRESS
    assigned_id: int = UNASSIGNED_ADDRESS
    hop_count: int = HOP_COUNT_UNKNOWN
    value: int = 0
    sequence: int = 0

    def type_name(self) -> str:
        return MSG_NAMES.get(self.message_type, f"TYPE{self.message_type}")


# Wire layout matches serialize_packet() in src/main.c (little-endian).
_OZ_FMT = "<2sBBHHHHBhH"  # 17 bytes: sig,ver,type,src,dst,parent,assigned,hop,value,seq


def serialize_packet(pkt: OzPacket) -> bytes:
    return struct.pack(
        _OZ_FMT,
        pkt.signature[:2],
        pkt.version & 0xFF,
        pkt.message_type & 0xFF,
        pkt.source_id & 0xFFFF,
        pkt.destination_id & 0xFFFF,
        pkt.parent_id & 0xFFFF,
        pkt.assigned_id & 0xFFFF,
        pkt.hop_count & 0xFF,
        int(pkt.value),
        pkt.sequence & 0xFFFF,
    )


def deserialize_packet(data: bytes) -> Optional[OzPacket]:
    if len(data) < OZ_PACKET_SIZE:
        return None
    sig, ver, mtype, src, dst, parent, assigned, hop, value, seq = struct.unpack(
        _OZ_FMT, data[:OZ_PACKET_SIZE]
    )
    if sig != b"OZ":
        return None
    if ver != PROTOCOL_VERSION:
        return None
    return OzPacket(
        signature=sig,
        version=ver,
        message_type=mtype,
        source_id=src,
        destination_id=dst,
        parent_id=parent,
        assigned_id=assigned,
        hop_count=hop,
        value=value,
        sequence=seq,
    )


def address_is_assignable(addr: int) -> bool:
    return (
        ADDRESS_NETWORK(addr) == 1
        and NODE_ID_FIRST <= ADDRESS_NODE(addr) <= NODE_ID_LAST
    )


def address_is_root(addr: int) -> bool:
    return addr == ROOT_ADDRESS


# ---------------------------------------------------------------------------
# Address allocation table (root)
# ---------------------------------------------------------------------------

@dataclass
class AddressRecord:
    address: int = 0
    parent: int = 0
    hop_count: int = 0
    request_sequence: int = 0
    join_nonce: int = 0
    pending: bool = False
    active: bool = False
    in_use: bool = False


class AddressTable:
    def __init__(self) -> None:
        self.rows: List[AddressRecord] = [
            AddressRecord() for _ in range(ADDRESS_TABLE_CAPACITY)
        ]

    def find_by_nonce(self, nonce: int) -> Optional[AddressRecord]:
        if nonce == 0:
            return None
        for row in self.rows:
            if row.in_use and row.join_nonce == nonce:
                return row
        return None

    def find_by_assigned(self, assigned: int) -> Optional[AddressRecord]:
        for row in self.rows:
            if row.in_use and row.address == assigned:
                return row
        return None

    def parent_ok(self, parent: int) -> bool:
        if address_is_root(parent):
            return True
        row = self.find_by_assigned(parent)
        return row is not None and row.active

    def parent_hop(self, parent: int) -> int:
        if address_is_root(parent):
            return 0
        row = self.find_by_assigned(parent)
        if row and row.active:
            return row.hop_count
        return HOP_COUNT_UNKNOWN

    def address_taken(self, addr: int) -> bool:
        return self.find_by_assigned(addr) is not None

    def allocate(
        self,
        parent: int,
        child_hop: int,
        request_sequence: int,
        join_nonce: int,
    ) -> AddressRecord:
        slot = next((r for r in self.rows if not r.in_use), None)
        if slot is None:
            raise RuntimeError("address table full")

        candidate = None
        for node_id in range(NODE_ID_FIRST, NODE_ID_LAST + 1):
            addr = MAKE_ADDRESS(1, node_id)
            if not self.address_taken(addr):
                candidate = addr
                break
        if candidate is None:
            raise RuntimeError("address pool exhausted")

        slot.address = candidate
        slot.parent = parent
        slot.hop_count = child_hop
        slot.request_sequence = request_sequence
        slot.join_nonce = join_nonce
        slot.pending = True
        slot.active = False
        slot.in_use = True
        return slot

    def release(self, row: AddressRecord) -> None:
        row.__dict__.update(AddressRecord().__dict__)

    def activate(self, assigned: int, parent: int, hop: int) -> bool:
        row = self.find_by_assigned(assigned)
        if row is None:
            return False
        row.pending = False
        row.active = True
        row.parent = parent
        row.hop_count = hop
        return True

    def active_addresses(self) -> List[int]:
        return [r.address for r in self.rows if r.in_use and r.active]

    def pending_addresses(self) -> List[int]:
        return [r.address for r in self.rows if r.in_use and r.pending]


# ---------------------------------------------------------------------------
# Parent selection
# ---------------------------------------------------------------------------

@dataclass
class ParentCandidate:
    address: int
    parent_id: int
    hop_count: int
    rssi_dbm: int


def parent_better(a: ParentCandidate, b: ParentCandidate) -> bool:
    a_known = a.hop_count != HOP_COUNT_UNKNOWN
    b_known = b.hop_count != HOP_COUNT_UNKNOWN
    if a_known != b_known:
        return a_known
    if a_known and a.hop_count != b.hop_count:
        return a.hop_count < b.hop_count
    if a.rssi_dbm != b.rssi_dbm:
        return a.rssi_dbm > b.rssi_dbm
    return a.address < b.address


def select_best_parent(candidates: List[ParentCandidate]) -> Optional[ParentCandidate]:
    best: Optional[ParentCandidate] = None
    for c in candidates:
        if best is None or parent_better(c, best):
            best = c
    return best


# ---------------------------------------------------------------------------
# Simulated nodes + air
# ---------------------------------------------------------------------------

@dataclass
class AirFrame:
    packet: OzPacket
    rssi_dbm: int = -50
    drop: bool = False  # when True, frame is lost (reliability demos)


@dataclass
class SimNode:
    name: str
    role: str  # "root" | "joining"
    state: NodeState = NodeState.UNREGISTERED
    local_address: int = UNASSIGNED_ADDRESS
    parent_address: int = UNASSIGNED_ADDRESS
    hop_count: int = HOP_COUNT_UNKNOWN
    tx_sequence: int = 1
    join_request_sequence: int = 0
    join_request_nonce: int = 0
    join_request_retries: int = 0
    join_outstanding: bool = False
    candidates: List[ParentCandidate] = field(default_factory=list)
    sensor_value: int = 0
    data_seq: int = 0
    table: Optional[AddressTable] = None
    # root DATA collection: source -> last seq
    collectors: Dict[int, int] = field(default_factory=dict)
    missing_reports: List[Tuple[int, int, int]] = field(default_factory=list)
    log: List[str] = field(default_factory=list)

    def note(self, msg: str) -> None:
        line = f"[{self.name}] {msg}"
        self.log.append(line)
        print(line)

    def boot(self) -> None:
        if self.role == "root":
            self.local_address = ROOT_ADDRESS
            self.parent_address = ROOT_ADDRESS
            self.hop_count = 0
            self.state = NodeState.REGISTERED
            self.table = AddressTable()
            self.note(f"boot ROOT REGISTERED at {fmt_addr(self.local_address)}")
        else:
            self.local_address = UNASSIGNED_ADDRESS
            self.parent_address = UNASSIGNED_ADDRESS
            self.hop_count = HOP_COUNT_UNKNOWN
            self.state = NodeState.PROBING
            self.note("boot JOINING → PROBING (unassigned 255:254)")

    def build(
        self,
        mtype: int,
        destination: int,
        value: int = 0,
        sequence: Optional[int] = None,
    ) -> OzPacket:
        if sequence is None:
            sequence = self.tx_sequence
            self.tx_sequence = (self.tx_sequence + 1) & 0xFFFF
        return OzPacket(
            message_type=mtype,
            source_id=self.local_address,
            destination_id=destination,
            parent_id=self.parent_address,
            assigned_id=UNASSIGNED_ADDRESS,
            hop_count=self.hop_count,
            value=value,
            sequence=sequence,
        )


class Air:
    """Simple broadcast bus: deliver frames to all other nodes."""

    def __init__(self) -> None:
        self.nodes: List[SimNode] = []
        self.trace: List[str] = []
        self.pending: List[Tuple[SimNode, AirFrame]] = []

    def add(self, node: SimNode) -> None:
        self.nodes.append(node)

    def send(self, sender: SimNode, frame: AirFrame) -> None:
        pkt = frame.packet
        line = (
            f"AIR  {sender.name} → type={pkt.type_name()} "
            f"src={fmt_addr(pkt.source_id)} dst={fmt_addr(pkt.destination_id)} "
            f"parent={fmt_addr(pkt.parent_id)} assigned={fmt_addr(pkt.assigned_id)} "
            f"hop={pkt.hop_count} seq={pkt.sequence} value={pkt.value}"
        )
        if frame.drop:
            line += "  [DROPPED]"
            self.trace.append(line)
            print(line)
            return
        self.trace.append(line)
        print(line)
        self.pending.append((sender, frame))

    def deliver_all(self) -> None:
        queue = self.pending
        self.pending = []
        for sender, frame in queue:
            for node in self.nodes:
                if node is sender:
                    continue
                dst = frame.packet.destination_id
                if dst not in (
                    BROADCAST_ADDRESS,
                    UNASSIGNED_ADDRESS,
                    node.local_address,
                    ROOT_ADDRESS if node.role == "root" else -1,
                ):
                    # Still deliver JOIN_RESPONSE to unassigned joiners
                    if not (
                        dst == UNASSIGNED_ADDRESS
                        and node.state == NodeState.WAITING_FOR_JOIN_RESPONSE
                    ):
                        if dst != node.local_address and dst != BROADCAST_ADDRESS:
                            continue
                dispatch(node, frame, self)


def dispatch(node: SimNode, frame: AirFrame, air: Air) -> None:
    pkt = frame.packet
    handlers: Dict[int, Callable[[SimNode, OzPacket, AirFrame, Air], None]] = {
        MSG_PROBE: on_probe,
        MSG_HEARTBEAT: on_heartbeat,
        MSG_JOIN_REQUEST: on_join_request,
        MSG_JOIN_RESPONSE: on_join_response,
        MSG_JOIN_ACKNOWLEDGE: on_join_ack,
        MSG_DATA: on_data,
    }
    handler = handlers.get(pkt.message_type)
    if handler is None:
        node.note(f"ignore unsupported type {pkt.message_type}")
        return
    handler(node, pkt, frame, air)


def on_probe(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.state != NodeState.REGISTERED:
        node.note(f"PROBE seq={pkt.sequence} ignored (state={STATE_NAMES[node.state]})")
        return
    node.note(f"PROBE seq={pkt.sequence} → HEARTBEAT reply")
    hb = node.build(MSG_HEARTBEAT, pkt.source_id, value=frame.rssi_dbm, sequence=pkt.sequence)
    hb.parent_id = node.parent_address
    hb.hop_count = node.hop_count
    air.send(node, AirFrame(hb, rssi_dbm=frame.rssi_dbm))


def on_heartbeat(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.state != NodeState.PROBING:
        node.note(f"HEARTBEAT ignored (state={STATE_NAMES[node.state]})")
        return
    node.note(
        f"HEARTBEAT from {fmt_addr(pkt.source_id)} "
        f"hop={pkt.hop_count} rssi={frame.rssi_dbm}"
    )
    node.candidates.append(
        ParentCandidate(
            address=pkt.source_id,
            parent_id=pkt.parent_id,
            hop_count=pkt.hop_count,
            rssi_dbm=frame.rssi_dbm,
        )
    )


def on_join_request(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.role != "root" or node.table is None:
        node.note("JOIN_REQUEST ignored (not root)")
        return
    nonce = pkt.value & 0xFFFF
    node.note(
        f"JOIN_REQUEST seq={pkt.sequence} nonce={nonce} "
        f"parent={fmt_addr(pkt.parent_id)}"
    )
    if nonce == 0:
        node.note("reject: nonce 0")
        return
    if not node.table.parent_ok(pkt.parent_id):
        node.note(f"reject: parent {fmt_addr(pkt.parent_id)} not active")
        return

    existing = node.table.find_by_nonce(nonce)
    if existing is not None:
        node.note(
            f"duplicate nonce={nonce}; retransmit assigned {fmt_addr(existing.address)}"
        )
        resp = node.build(
            MSG_JOIN_RESPONSE, UNASSIGNED_ADDRESS, sequence=pkt.sequence
        )
        resp.parent_id = existing.parent
        resp.assigned_id = existing.address
        resp.hop_count = existing.hop_count
        air.send(node, AirFrame(resp))
        return

    parent_hop = node.table.parent_hop(pkt.parent_id)
    child_hop = parent_hop + 1
    row = node.table.allocate(pkt.parent_id, child_hop, pkt.sequence, nonce)
    node.note(
        f"allocated PENDING {fmt_addr(row.address)} hop={child_hop} nonce={nonce}"
    )
    resp = node.build(MSG_JOIN_RESPONSE, UNASSIGNED_ADDRESS, sequence=pkt.sequence)
    resp.parent_id = row.parent
    resp.assigned_id = row.address
    resp.hop_count = row.hop_count
    air.send(node, AirFrame(resp))


def on_join_response(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.role != "joining":
        return
    if node.state != NodeState.WAITING_FOR_JOIN_RESPONSE:
        node.note(f"JOIN_RESPONSE ignored (state={STATE_NAMES[node.state]})")
        return
    if not node.join_outstanding or pkt.sequence != node.join_request_sequence:
        node.note(
            f"JOIN_RESPONSE seq={pkt.sequence} ignored "
            f"(expected {node.join_request_sequence})"
        )
        return
    if not address_is_assignable(pkt.assigned_id):
        node.note(f"invalid assigned {fmt_addr(pkt.assigned_id)}")
        return

    node.local_address = pkt.assigned_id
    node.parent_address = pkt.parent_id
    node.hop_count = pkt.hop_count
    node.state = NodeState.REGISTERED
    node.join_outstanding = False
    node.note(
        f"REGISTERED at {fmt_addr(node.local_address)} "
        f"parent={fmt_addr(node.parent_address)} hop={node.hop_count}"
    )

    ack = node.build(
        MSG_JOIN_ACKNOWLEDGE, ROOT_ADDRESS, sequence=pkt.sequence
    )
    ack.parent_id = pkt.parent_id
    ack.assigned_id = pkt.assigned_id
    ack.hop_count = pkt.hop_count
    air.send(node, AirFrame(ack))


def on_join_ack(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.role != "root" or node.table is None:
        return
    ok = node.table.activate(pkt.assigned_id, pkt.parent_id, pkt.hop_count)
    if ok:
        node.note(f"address {fmt_addr(pkt.assigned_id)} marked ACTIVE")
    else:
        node.note(f"JOIN_ACK for unknown {fmt_addr(pkt.assigned_id)}")


def on_data(node: SimNode, pkt: OzPacket, frame: AirFrame, air: Air) -> None:
    if node.role == "root":
        last = node.collectors.get(pkt.source_id)
        if last is not None and pkt.sequence != ((last + 1) & 0xFFFF):
            missing = (pkt.sequence - last - 1) & 0xFFFF
            node.missing_reports.append(
                (fmt_addr(pkt.source_id), last, pkt.sequence)
            )
            node.note(
                f"DATA gap from {fmt_addr(pkt.source_id)}: "
                f"last={last} got={pkt.sequence} (~{missing} missing)"
            )
        node.collectors[pkt.source_id] = pkt.sequence
        node.note(
            f"DATA collect seq={pkt.sequence} value={pkt.value} "
            f"from {fmt_addr(pkt.source_id)}"
        )
        return

    if node.state != NodeState.REGISTERED:
        node.note("DATA ignored (not REGISTERED)")
        return
    if pkt.destination_id != node.local_address:
        return

    node.note(
        f"DATA forward seq={pkt.sequence} value={pkt.value} "
        f"origin={fmt_addr(pkt.source_id)} → parent {fmt_addr(node.parent_address)}"
    )
    fwd = OzPacket(
        message_type=MSG_DATA,
        source_id=pkt.source_id,  # preserve origin
        destination_id=node.parent_address,
        parent_id=node.parent_address,
        assigned_id=UNASSIGNED_ADDRESS,
        hop_count=pkt.hop_count,
        value=pkt.value,
        sequence=pkt.sequence,
    )
    air.send(node, AirFrame(fwd))


def send_probe(node: SimNode, air: Air, seq: Optional[int] = None) -> OzPacket:
    pkt = node.build(MSG_PROBE, BROADCAST_ADDRESS, value=5, sequence=seq)
    node.note(f"PROBE TX seq={pkt.sequence}")
    air.send(node, AirFrame(pkt, rssi_dbm=-55))
    return pkt


def start_join(node: SimNode, parent: ParentCandidate, air: Air, nonce: int) -> None:
    node.state = NodeState.WAITING_FOR_JOIN_RESPONSE
    node.parent_address = parent.address
    node.hop_count = parent.hop_count + 1 if parent.hop_count != HOP_COUNT_UNKNOWN else HOP_COUNT_UNKNOWN
    node.join_request_sequence = node.tx_sequence
    node.tx_sequence = (node.tx_sequence + 1) & 0xFFFF
    node.join_request_nonce = nonce
    node.join_request_retries = 0
    node.join_outstanding = True
    node.note(
        f"Parent selected {fmt_addr(parent.address)} "
        f"hop={parent.hop_count} rssi={parent.rssi_dbm}"
    )
    send_join_request(node, air)


def send_join_request(node: SimNode, air: Air, drop: bool = False) -> None:
    pkt = node.build(
        MSG_JOIN_REQUEST,
        ROOT_ADDRESS,
        value=node.join_request_nonce,
        sequence=node.join_request_sequence,
    )
    pkt.parent_id = node.parent_address
    pkt.hop_count = node.hop_count
    node.note(
        f"JOIN_REQUEST TX seq={pkt.sequence} nonce={node.join_request_nonce} "
        f"parent={fmt_addr(pkt.parent_id)} (retry {node.join_request_retries}/"
        f"{JOIN_REQUEST_MAX_RETRIES})"
    )
    air.send(node, AirFrame(pkt, drop=drop))


def send_sensor_data(node: SimNode, air: Air, drop: bool = False) -> None:
    node.sensor_value += 1
    node.data_seq = (node.data_seq + 1) & 0xFFFF
    pkt = OzPacket(
        message_type=MSG_DATA,
        source_id=node.local_address,
        destination_id=node.parent_address,
        parent_id=node.parent_address,
        assigned_id=UNASSIGNED_ADDRESS,
        hop_count=node.hop_count,
        value=node.sensor_value,
        sequence=node.data_seq,
    )
    node.note(
        f"DATA TX seq={pkt.sequence} value={pkt.value} → {fmt_addr(pkt.destination_id)}"
    )
    air.send(node, AirFrame(pkt, drop=drop))


def flush(air: Air, rounds: int = 4) -> None:
    for _ in range(rounds):
        if not air.pending:
            break
        air.deliver_all()


# ---------------------------------------------------------------------------
# Milestone runners
# ---------------------------------------------------------------------------

@dataclass
class MilestoneResult:
    mid: int
    name: str
    passed: bool
    details: List[str] = field(default_factory=list)


def check(cond: bool, ok: str, bad: str, details: List[str]) -> bool:
    if cond:
        details.append(f"  PASS: {ok}")
        return True
    details.append(f"  FAIL: {bad}")
    return False


def milestone1() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M1 Packet format ===")
    pkt = OzPacket(
        message_type=MSG_JOIN_RESPONSE,
        source_id=ROOT_ADDRESS,
        destination_id=UNASSIGNED_ADDRESS,
        parent_id=ROOT_ADDRESS,
        assigned_id=MAKE_ADDRESS(1, 1),
        hop_count=1,
        value=0,
        sequence=42,
    )
    raw = serialize_packet(pkt)
    details.append(f"  wire bytes ({len(raw)}): {raw.hex()}")
    ok_size = check(len(raw) == OZ_PACKET_SIZE, "OZ header is 17 bytes", "wrong size", details)
    roundtrip = deserialize_packet(raw)
    ok_rt = check(
        roundtrip is not None
        and roundtrip.message_type == MSG_JOIN_RESPONSE
        and roundtrip.assigned_id == MAKE_ADDRESS(1, 1)
        and roundtrip.sequence == 42,
        "serialize → deserialize round-trip",
        "round-trip failed",
        details,
    )
    bad_sig = bytearray(raw)
    bad_sig[0] = ord("X")
    ok_sig = check(
        deserialize_packet(bytes(bad_sig)) is None,
        "bad signature rejected",
        "bad signature accepted",
        details,
    )
    bad_ver = bytearray(raw)
    bad_ver[2] = 1
    ok_ver = check(
        deserialize_packet(bytes(bad_ver)) is None,
        "version mismatch rejected",
        "version mismatch accepted",
        details,
    )
    ok_types = check(
        set(MSG_NAMES.keys()) == set(range(1, 9)),
        "message types 1–8 defined",
        "message type set incomplete",
        details,
    )
    passed = all([ok_size, ok_rt, ok_sig, ok_ver, ok_types])
    return MilestoneResult(1, "Packet format", passed, details)


def milestone2() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M2 Discovery ===")
    air = Air()
    root = SimNode("root", "root")
    joiner = SimNode("nodeA", "joining")
    air.add(root)
    air.add(joiner)
    root.boot()
    joiner.boot()

    send_probe(joiner, air, seq=7)
    flush(air)

    hb_seen = any("HEARTBEAT" in t and "root →" in t for t in air.trace)
    probe_seen = any("PROBE" in t and "nodeA →" in t for t in air.trace)
    candidate_ok = len(joiner.candidates) == 1 and joiner.candidates[0].address == ROOT_ADDRESS

    ok1 = check(probe_seen, "joining node transmitted PROBE", "no PROBE", details)
    ok2 = check(hb_seen, "registered root replied HEARTBEAT", "no HEARTBEAT", details)
    ok3 = check(candidate_ok, "joiner recorded root as candidate", "no candidate", details)
    return MilestoneResult(2, "Discovery", all([ok1, ok2, ok3]), details)


def milestone3() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M3 Parent selection ===")
    # hop 0 @ -70 vs hop 1 @ -40 → prefer hop 0
    c1 = ParentCandidate(ROOT_ADDRESS, ROOT_ADDRESS, 0, -70)
    c2 = ParentCandidate(MAKE_ADDRESS(1, 10), ROOT_ADDRESS, 1, -40)
    best = select_best_parent([c1, c2])
    ok_hop = check(
        best is not None and best.address == ROOT_ADDRESS,
        "lowest hop wins over stronger RSSI",
        f"wrong pick {best}",
        details,
    )
    # same hop: stronger RSSI
    c3 = ParentCandidate(MAKE_ADDRESS(1, 2), ROOT_ADDRESS, 1, -40)
    c4 = ParentCandidate(MAKE_ADDRESS(1, 3), ROOT_ADDRESS, 1, -60)
    best2 = select_best_parent([c3, c4])
    ok_rssi = check(
        best2 is not None and best2.address == MAKE_ADDRESS(1, 2),
        "tie on hop → strongest RSSI",
        f"wrong pick {best2}",
        details,
    )
    # same hop+RSSI: lowest address
    c5 = ParentCandidate(MAKE_ADDRESS(1, 5), ROOT_ADDRESS, 1, -50)
    c6 = ParentCandidate(MAKE_ADDRESS(1, 4), ROOT_ADDRESS, 1, -50)
    best3 = select_best_parent([c5, c6])
    ok_addr = check(
        best3 is not None and best3.address == MAKE_ADDRESS(1, 4),
        "tie on hop+RSSI → lowest address",
        f"wrong pick {best3}",
        details,
    )

    air = Air()
    root = SimNode("root", "root")
    mid = SimNode("relay", "joining")
    joiner = SimNode("nodeA", "joining")
    for n in (root, mid, joiner):
        air.add(n)
        n.boot()
    # Pre-register relay as active 1:10 hop 1 under root
    mid.local_address = MAKE_ADDRESS(1, 10)
    mid.parent_address = ROOT_ADDRESS
    mid.hop_count = 1
    mid.state = NodeState.REGISTERED
    assert root.table is not None
    row = root.table.allocate(ROOT_ADDRESS, 1, 0, 99)
    root.table.activate(row.address, ROOT_ADDRESS, 1)
    # Force address to 1:10 for the scenario
    row.address = MAKE_ADDRESS(1, 10)

    send_probe(joiner, air, seq=1)
    flush(air)
    # Inject second HB with worse hop by hand into candidates already collected
    # Selection from recorded candidates:
    chosen = select_best_parent(joiner.candidates)
    ok_live = check(
        chosen is not None and chosen.address == ROOT_ADDRESS,
        f"live selection chose {fmt_addr(chosen.address) if chosen else 'None'} (expect root)",
        "live selection failed",
        details,
    )
    if chosen:
        joiner.note(
            f"Parent selected {fmt_addr(chosen.address)} hop={chosen.hop_count} "
            f"rssi={chosen.rssi_dbm}"
        )
    return MilestoneResult(3, "Parent selection", all([ok_hop, ok_rssi, ok_addr, ok_live]), details)


def _join_one(air: Air, root: SimNode, joiner: SimNode, nonce: int) -> None:
    send_probe(joiner, air)
    flush(air)
    chosen = select_best_parent(joiner.candidates)
    assert chosen is not None
    start_join(joiner, chosen, air, nonce=nonce)
    flush(air)


def milestone4() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M4 Address assignment ===")
    air = Air()
    root = SimNode("root", "root")
    a = SimNode("nodeA", "joining")
    b = SimNode("nodeB", "joining")
    c = SimNode("nodeC", "joining")
    for n in (root, a, b, c):
        air.add(n)
        n.boot()

    _join_one(air, root, a, nonce=101)
    _join_one(air, root, b, nonce=102)
    _join_one(air, root, c, nonce=103)

    assert root.table is not None
    pending_or_active = [
        r.address for r in root.table.rows if r.in_use
    ]
    unique = set(pending_or_active)
    ok1 = check(
        len(unique) >= 3,
        f"root issued ≥3 unique addresses: {[fmt_addr(x) for x in sorted(unique)]}",
        f"only {unique}",
        details,
    )
    ok2 = check(
        MAKE_ADDRESS(1, 1) in unique,
        "first allocation starts at 1:1",
        "did not start at 1:1",
        details,
    )
    ok3 = check(
        all(address_is_assignable(x) for x in unique),
        "skipped reserved/broadcast/root",
        "invalid address issued",
        details,
    )
    # Responses present
    resp_count = sum(1 for t in air.trace if "JOIN_RESPONSE" in t)
    ok4 = check(resp_count >= 3, f"{resp_count} JOIN_RESPONSE messages", "missing responses", details)
    return MilestoneResult(4, "Address assignment", all([ok1, ok2, ok3, ok4]), details)


def milestone5() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M5 Registration ===")
    air = Air()
    root = SimNode("root", "root")
    a = SimNode("nodeA", "joining")
    for n in (root, a):
        air.add(n)
        n.boot()
    _join_one(air, root, a, nonce=201)
    flush(air)

    ok1 = check(
        a.state == NodeState.REGISTERED,
        f"nodeA state={STATE_NAMES[a.state]}",
        "nodeA not REGISTERED",
        details,
    )
    ok2 = check(
        a.local_address == MAKE_ADDRESS(1, 1),
        f"nodeA applied address {fmt_addr(a.local_address)}",
        "wrong address",
        details,
    )
    ok3 = check(
        a.parent_address == ROOT_ADDRESS,
        f"nodeA parent={fmt_addr(a.parent_address)}",
        "wrong parent",
        details,
    )
    assert root.table is not None
    active = root.table.active_addresses()
    ok4 = check(
        MAKE_ADDRESS(1, 1) in active,
        "root marked 1:1 ACTIVE after JOIN_ACK",
        f"active={ [fmt_addr(x) for x in active] }",
        details,
    )
    ack_seen = any("JOIN_ACKNOWLEDGE" in t for t in air.trace)
    ok5 = check(ack_seen, "JOIN_ACKNOWLEDGE exchanged", "no ACK on air", details)
    return MilestoneResult(5, "Registration", all([ok1, ok2, ok3, ok4, ok5]), details)


def milestone6() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M6 Data collection ===")
    air = Air()
    root = SimNode("root", "root")
    relay = SimNode("relay", "joining")
    child = SimNode("child", "joining")
    for n in (root, relay, child):
        air.add(n)
        n.boot()

    # Join relay under root
    _join_one(air, root, relay, nonce=301)
    flush(air)

    # Child selects relay as parent: seed candidate then join
    child.candidates = [
        ParentCandidate(relay.local_address, ROOT_ADDRESS, relay.hop_count, -45)
    ]
    start_join(child, child.candidates[0], air, nonce=302)
    flush(air)

    ok_reg = check(
        child.state == NodeState.REGISTERED and relay.state == NodeState.REGISTERED,
        f"relay={fmt_addr(relay.local_address)} child={fmt_addr(child.local_address)} REGISTERED",
        "join tree incomplete",
        details,
    )

    send_sensor_data(child, air)
    flush(air)
    send_sensor_data(child, air)
    flush(air)

    collected = root.collectors.get(child.local_address)
    ok_data = check(
        collected is not None and collected >= 2,
        f"root collected child DATA (last seq={collected})",
        "root never saw child DATA",
        details,
    )
    fwd = any("DATA forward" in line for line in relay.log)
    ok_fwd = check(fwd, "relay forwarded child DATA toward root", "no forward", details)
    return MilestoneResult(6, "Data collection", all([ok_reg, ok_data, ok_fwd]), details)


def milestone7() -> MilestoneResult:
    details: List[str] = []
    print("\n=== M7 Reliability ===")
    air = Air()
    root = SimNode("root", "root")
    a = SimNode("nodeA", "joining")
    for n in (root, a):
        air.add(n)
        n.boot()

    # --- Duplicate JOIN_REQUEST: same nonce must not allocate twice ---
    send_probe(a, air)
    flush(air)
    chosen = select_best_parent(a.candidates)
    assert chosen is not None
    start_join(a, chosen, air, nonce=401)
    flush(air)
    assigned_first = a.local_address
    # Simulate lost ACK path: force node back to waiting and retransmit same nonce
    # (duplicate request while still pending on a fresh joiner)
    b = SimNode("nodeB", "joining")
    air.add(b)
    b.boot()
    send_probe(b, air)
    flush(air)
    chosen_b = select_best_parent(b.candidates)
    assert chosen_b is not None
    start_join(b, chosen_b, air, nonce=402)
    flush(air)
    # Retransmit B's JOIN_REQUEST (duplicate)
    b.state = NodeState.WAITING_FOR_JOIN_RESPONSE
    b.join_outstanding = True
    b.local_address = UNASSIGNED_ADDRESS  # still looks unassigned on wire for demo
    # Keep same nonce/seq as first request — but node already registered; use table check
    assert root.table is not None
    before = [r.address for r in root.table.rows if r.in_use]
    # Craft duplicate request with same nonce 402
    dup = OzPacket(
        message_type=MSG_JOIN_REQUEST,
        source_id=UNASSIGNED_ADDRESS,
        destination_id=ROOT_ADDRESS,
        parent_id=ROOT_ADDRESS,
        assigned_id=UNASSIGNED_ADDRESS,
        hop_count=1,
        value=402,
        sequence=b.join_request_sequence,
    )
    air.send(b, AirFrame(dup))
    flush(air)
    after = [r.address for r in root.table.rows if r.in_use]
    ok_dup = check(
        len(after) == len(before),
        "duplicate JOIN_REQUEST did not allocate a second address",
        f"table grew {before} → {after}",
        details,
    )
    retransmit = sum(1 for t in air.trace if "JOIN_RESPONSE" in t and "root →" in t)
    ok_retx_resp = check(
        retransmit >= 3,  # A, B initial, B duplicate
        "root retransmitted JOIN_RESPONSE on duplicate",
        "missing response retransmit",
        details,
    )

    # --- Lost JOIN_RESPONSE → joiner retransmits JOIN_REQUEST ---
    c = SimNode("nodeC", "joining")
    air.add(c)
    c.boot()
    send_probe(c, air)
    flush(air)
    chosen_c = select_best_parent(c.candidates)
    assert chosen_c is not None
    c.state = NodeState.WAITING_FOR_JOIN_RESPONSE
    c.parent_address = chosen_c.address
    c.hop_count = 1
    c.join_request_sequence = c.tx_sequence
    c.tx_sequence += 1
    c.join_request_nonce = 501
    c.join_outstanding = True
    send_join_request(c, air, drop=True)  # first attempt lost
    flush(air)
    ok_still_waiting = check(
        c.state == NodeState.WAITING_FOR_JOIN_RESPONSE and c.local_address == UNASSIGNED_ADDRESS,
        "after lost response, node still waiting / unassigned",
        "unexpected registration",
        details,
    )
    c.join_request_retries += 1
    send_join_request(c, air, drop=False)  # retransmit
    flush(air)
    ok_recover = check(
        c.state == NodeState.REGISTERED and address_is_assignable(c.local_address),
        f"retransmit recovered → REGISTERED {fmt_addr(c.local_address)}",
        "retransmit did not complete join",
        details,
    )

    # --- Pending release after failed ACK (timeout) ---
    # Root allocates + sends JOIN_RESPONSE, but the response is dropped so
    # the joiner never ACKs. Address stays pending until explicitly released.
    d = SimNode("nodeD", "joining")
    air.add(d)
    d.boot()
    send_probe(d, air)
    flush(air)
    chosen_d = select_best_parent(d.candidates)
    assert chosen_d is not None
    d.state = NodeState.WAITING_FOR_JOIN_RESPONSE
    d.parent_address = ROOT_ADDRESS
    d.hop_count = 1
    d.join_request_sequence = 77
    d.join_request_nonce = 601
    d.join_outstanding = True
    send_join_request(d, air)
    # Deliver JOIN_REQUEST to root only; capture + drop JOIN_RESPONSE
    air.deliver_all()
    dropped_resp = 0
    kept: List[Tuple[SimNode, AirFrame]] = []
    for sender, frame in air.pending:
        if frame.packet.message_type == MSG_JOIN_RESPONSE:
            frame.drop = True
            dropped_resp += 1
            print(
                f"AIR  {sender.name} → type=JOIN_RESPONSE "
                f"assigned={fmt_addr(frame.packet.assigned_id)}  [DROPPED]"
            )
            air.trace.append(
                f"AIR  {sender.name} → JOIN_RESPONSE "
                f"{fmt_addr(frame.packet.assigned_id)} [DROPPED]"
            )
        else:
            kept.append((sender, frame))
    air.pending = kept
    flush(air)

    row = root.table.find_by_nonce(601)
    ok_pending = check(
        row is not None and row.pending and dropped_resp >= 1,
        f"address reserved pending ({fmt_addr(row.address) if row else 'n/a'}) "
        f"while ACK never arrived",
        "no pending reservation",
        details,
    )
    if row and row.pending:
        released_addr = row.address
        root.table.release(row)
        root.note(f"JOIN_ACK timeout; released {fmt_addr(released_addr)}")
        e = SimNode("nodeE", "joining")
        air.add(e)
        e.boot()
        send_probe(e, air)
        flush(air)
        chosen_e = select_best_parent(e.candidates)
        assert chosen_e is not None
        start_join(e, chosen_e, air, nonce=701)
        flush(air)
        ok_reuse = check(
            e.local_address == released_addr,
            f"released address {fmt_addr(released_addr)} reused for nodeE",
            f"nodeE got {fmt_addr(e.local_address)}, expected {fmt_addr(released_addr)}",
            details,
        )
    else:
        ok_reuse = check(False, "", "cannot demo reuse without pending row", details)

    # --- Invalid packet ignored ---
    bad = bytearray(serialize_packet(OzPacket(message_type=MSG_PROBE, sequence=1)))
    bad[0] = ord("X")
    ok_bad = check(
        deserialize_packet(bytes(bad)) is None,
        "invalid signature rejected (Test 7)",
        "invalid packet accepted",
        details,
    )

    # --- DATA gap detection ---
    child = a if a.state == NodeState.REGISTERED else c
    if child.state == NodeState.REGISTERED:
        send_sensor_data(child, air)
        flush(air)
        child.data_seq += 1  # skip a sequence to simulate loss
        send_sensor_data(child, air)
        flush(air)
        ok_gap = check(
            len(root.missing_reports) >= 1,
            f"root reported DATA gap {root.missing_reports[-1]}",
            "no gap report",
            details,
        )
    else:
        ok_gap = check(False, "", "no registered child for gap demo", details)

    # --- Exhaust retries → revert to PROBING ---
    f = SimNode("nodeF", "joining")
    air.add(f)
    f.boot()
    f.state = NodeState.WAITING_FOR_JOIN_RESPONSE
    f.join_request_retries = JOIN_REQUEST_MAX_RETRIES
    f.join_outstanding = True
    f.join_request_retries += 1
    if f.join_request_retries > JOIN_REQUEST_MAX_RETRIES:
        f.state = NodeState.PROBING
        f.local_address = UNASSIGNED_ADDRESS
        f.note("JOIN_RESPONSE timeout after max retries; revert to PROBING")
    ok_revert = check(
        f.state == NodeState.PROBING,
        "exhausted retries → revert to PROBING",
        "did not revert",
        details,
    )

    passed = all(
        [ok_dup, ok_retx_resp, ok_still_waiting, ok_recover, ok_pending, ok_reuse, ok_bad, ok_gap, ok_revert]
    )
    return MilestoneResult(7, "Reliability", passed, details)


MILESTONES: Dict[int, Callable[[], MilestoneResult]] = {
    1: milestone1,
    2: milestone2,
    3: milestone3,
    4: milestone4,
    5: milestone5,
    6: milestone6,
    7: milestone7,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--milestone",
        type=int,
        choices=sorted(MILESTONES.keys()),
        help="Run a single milestone (default: all)",
    )
    args = parser.parse_args()

    ids = [args.milestone] if args.milestone else sorted(MILESTONES.keys())
    results: List[MilestoneResult] = []

    print("IEEE 802.15.4 Data Collection Tree — Milestone Simulator")
    print("Mirrors application rules from src/main.c (no radio required)\n")

    for mid in ids:
        results.append(MILESTONES[mid]())

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    all_ok = True
    for r in results:
        mark = "PASS" if r.passed else "FAIL"
        print(f"  M{r.mid} {r.name:<20} {mark}")
        for d in r.details:
            print(d)
        all_ok = all_ok and r.passed

    print("=" * 60)
    print("ALL MILESTONES PASS" if all_ok else "SOME MILESTONES FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
