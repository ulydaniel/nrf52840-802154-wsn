# Section 3 — Application Message Types & Packet Format

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, message types + packet fields  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §3  
**Depends on:** Section 1 addressing, Section 2 node FSM

This document is the protocol description for the common OZ header, the assignment type numbers, serialize/deserialize rules, and the handler table.

---

## 1. Why this section exists

Sections 1–2 gave every board an address model and an FSM. Section 3 makes the **on-wire language** match the assignment:

1. Message type numbers 1–8 (including `DATA = 6`)
2. Common header fields for parent, assigned address, and hop count
3. One serialize/deserialize path for every type
4. A handler for every type (join / DATA / NETID still decode-and-log until later milestones)

---

## 2. Message type numbers

Aligned with the assignment PDF (no remapping):

| Type | Value | Handler | Behavior today |
|------|------:|---------|----------------|
| `PROBE` | 1 | `handle_probe` | Registered node replies with HEARTBEAT |
| `HEARTBEAT` | 2 | `handle_heartbeat` | Probing joiner accepts matching reply |
| `JOIN_REQUEST` | 3 | `handle_join_request` | Decode + log; allocation TBD |
| `JOIN_RESPONSE` | 4 | `handle_join_response` | Decode + log; registration TBD |
| `JOIN_ACKNOWLEDGE` | 5 | `handle_join_acknowledge` | Decode + log; table activate TBD |
| `DATA` | 6 | `handle_data` | Decode + log; forward / collect TBD |
| `NETIDREQ` | 7 | `handle_netid_request` | Reserved (multi-net) |
| `NETIDRESP` | 8 | `handle_netid_response` | Reserved (multi-net) |

```c
#define MSG_TYPE_PROBE              1
#define MSG_TYPE_HEARTBEAT          2
#define MSG_TYPE_JOIN_REQUEST       3
#define MSG_TYPE_JOIN_RESPONSE      4
#define MSG_TYPE_JOIN_ACKNOWLEDGE   5
#define MSG_TYPE_DATA               6
#define MSG_TYPE_NETID_REQUEST      7
#define MSG_TYPE_NETID_RESPONSE     8
```

`message_handlers[]` is indexed by `(type - MSG_TYPE_FIRST)`. There are **no NULL slots** for types 1–8.

---

## 3. Common OZ header (PROTOCOL_VERSION 2)

**17 bytes**, little-endian multi-byte fields:

```text
Byte  0-1   Signature "OZ"
Byte  2     Protocol version (= 2)
Byte  3     Message type (1..8)
Byte  4-5   Source address
Byte  6-7   Destination address
Byte  8-9   Parent address
Byte  10-11 Assigned address
Byte  12    Hop count (0xFF = unknown)
Byte  13-14 Value / sensor / type-specific int16
Byte  15-16 Sequence
```

```c
struct oz_packet {
    uint8_t  signature[2];
    uint8_t  version;
    uint8_t  message_type;
    uint16_t source_id;
    uint16_t destination_id;
    uint16_t parent_id;
    uint16_t assigned_id;
    uint8_t  hop_count;
    int16_t  value;
    uint16_t sequence;
};
```

`OZ_PACKET_SIZE` is **17**. Version was bumped from 1 → **2** because the wire layout changed; peers on v1 headers are rejected by `deserialize_packet()`.

### Type-specific use of shared fields

| Type | `value` | `parent_id` | `assigned_id` | `hop_count` | `sequence` |
|------|---------|-------------|---------------|-------------|------------|
| PROBE | K (seconds) | usually unassigned | unused | unknown until joined | probe seq |
| HEARTBEAT | probe RSSI (dBm) | responder parent | unused | responder hop | echoes PROBE seq |
| JOIN_REQUEST | optional / unused | selected parent | unused | proposed child hop | request seq |
| JOIN_RESPONSE | optional status | parent to install | **granted address** | child hop | response seq |
| JOIN_ACKNOWLEDGE | unused | installed parent | accepted address | installed hop | ack seq |
| DATA | sensor reading | sender parent (optional) | unused | sender hop | data seq |
| NETIDREQ/RESP | TBD | TBD | hint / unused | TBD | seq |

### Optional bodies

Some types still append a body **after** the 17-byte header. Today only HEARTBEAT does:

| Body bytes | Field |
|-----------:|-------|
| 0–7 | GUID |
| 8–9 | Battery mV |
| 10 | Battery % (`0xFF` = unknown) |
| 11–12 | Heartbeat sequence |

`HEARTBEAT_FRAME_SIZE = OZ_PACKET_SIZE + HEARTBEAT_BODY_SIZE` (30 bytes).

---

## 4. Serialize / deserialize

| Function | Role |
|----------|------|
| `serialize_packet()` | Writes exactly `OZ_PACKET_SIZE` LE bytes |
| `deserialize_packet()` | Requires `length >= OZ_PACKET_SIZE`; checks `"OZ"` + `PROTOCOL_VERSION` |
| `build_packet()` | Fills signature/version/type/src/dst from `local_address`, defaults `parent_id` / `hop_count` from FSM globals, `assigned_id = UNASSIGNED` |
| `send_oz_packet()` | Header only |
| `send_oz_packet_with_body()` | Header + body (HEARTBEAT) |
| `process_frame()` | Header decode → destination filter → `dispatch_message()` |

Join / DATA TX builders in later milestones call `build_packet()` then override `assigned_id` / `parent_id` / `hop_count` as needed.

---

## 5. Handler table

```text
RX frame
  → deserialize common header
  → address_is_for_me(destination)?
  → message_handlers[type - 1](packet, frame)
```

| Handler | Gate / note |
|---------|-------------|
| `handle_probe` | Only `REGISTERED` replies (`node_can_reply_heartbeat`) |
| `handle_heartbeat` | Only meaningful in `PROBING` today |
| `handle_join_request` | Root `REGISTERED` logs; allocation later |
| `handle_join_response` | Joiner in `PROBING` / `WAITING_FOR_JOIN_RESPONSE` logs |
| `handle_join_acknowledge` | Root `REGISTERED` logs |
| `handle_data` | Root logs collection TBD; registered non-root logs forward TBD |
| `handle_netid_*` | Reserved multi-net; log only |

`print_packet()` now logs **Parent**, **Assigned**, and **Hop Count** in addition to source / destination / value / sequence — matching demo log requirements in checklist §8.

---

## 6. What changed vs PROTOCOL_VERSION 1

| Item | Before | After |
|------|--------|-------|
| Header size | 12 bytes | **17 bytes** |
| Version | 1 | **2** |
| Parent / assigned / hop | not on wire | in common header |
| `DATA` type | missing (NETID was 6–7) | **`DATA = 6`**, NETID **7–8** |
| JOIN / DATA / NETID handlers | `NULL` | decode + log handlers |
| HEARTBEAT body offset | after byte 12 | after byte 17 |

Both boards in a demo must run this firmware revision; mixed v1/v2 peers will drop each other’s packets.

---

## 7. Relationship to other sections

| Concern | Section |
|---------|---------|
| Who am I? | §1 `local_address` |
| What phase am I in? | §2 `current_state` |
| What did I receive? | **§3 type + header fields** |
| When do I PROBE / JOIN / send DATA? | §4+ protocol flow |

---

## 8. Still not done (later checklist sections)

- Building and transmitting JOIN_REQUEST / RESPONSE / ACKNOWLEDGE
- Root address pool + duplicate detection on JOIN_REQUEST
- Calling `node_enter_*` from join handlers
- Periodic DATA generation and parent forwarding
- Multi-network NETIDREQ / NETIDRESP behavior

---

## 9. Symbol map

| Item | Location in `main.c` |
|------|----------------------|
| `MSG_TYPE_*`, `PROTOCOL_VERSION` | Near top, after addressing macros |
| `struct oz_packet`, `OZ_PACKET_SIZE` | “Application payload” |
| `serialize_packet` / `deserialize_packet` | After `print_packet` |
| `build_packet` | Before message handlers |
| `handle_*` + `message_handlers[]` | “Message handlers” |
| `dispatch_message` / `process_frame` | After handler table |
