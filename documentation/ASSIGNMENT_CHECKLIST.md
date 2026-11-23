# IEEE 802.15.4 Data Collection Tree — Assignment Checklist

**Course:** COMPE 662  
**Starting point:** nRF52840 IEEE 802.15.4 sender/receiver program (`src/main.c`)  
**Assignment focus:** Application-layer discovery, address assignment, joining, and tree formation  
**Last reviewed against:** `IEEE_802154_Data_Collection_Tree_Assignment.pdf` + current tree state

**Legend**

- `[x]` Done in current project
- `[~]` Partial / exists but needs rework for the assignment
- `[ ]` Not implemented yet

---

## 0. Project baseline (already in place)

Infrastructure from the supplied / evolved sender-receiver program. Keep unless a change is justified.

### Radio / Zephyr stack

- [x] IEEE 802.15.4 channel configuration
- [x] PAN ID configuration
- [x] Transmit power configuration
- [x] Short address set via `net_mgmt` / L2
- [x] AF_PACKET + SOCK_DGRAM style transmit path
- [x] Receive via Zephyr `net_context` + packet callback
- [x] Both TX and RX on every participating node (shared `receive_context`)
- [x] RSSI available on received frames
- [x] Board / NCS build (`OzturkLab/nrf52840`, `prj.conf`, overlays)

### Application packet plumbing

- [x] Application signature (`"OZ"`)
- [x] Protocol version field + reject incompatible versions
- [x] Message type field
- [x] Source / destination application addresses
- [x] Sequence number field
- [x] Serialize / deserialize (little-endian, not raw structs)
- [x] Message-handler dispatch table (`message_handlers[]`)
- [x] RX queue from radio callback → main processing path
- [x] Logging of type, addresses, sequence, RSSI (RTT)

### Extra features (not required by PDF; optional keep)

- [x] Device GUID in heartbeat body (`hwinfo`)
- [x] Battery ADC / percent reporting
- [x] Probe K / N backoff (current sender poll model)
- [x] Link / sequence-gap stats (`main.c`)
- [x] RX/protocol + TX threads in `src/main.c` (see `documentation/RX_TX_THREADS.md`); older experiment remains in `src/code.c`

---

## 1. Addressing model

Represent network address as network ID + node ID; on the wire as 16-bit.  
**Doc:** `documentation/SECTION1_ADDRESSING.md`

- [x] `#define MAKE_ADDRESS(network, node) (((network & 0xFF) << 8) | (node & 0xFF))`
- [x] `ROOT_ADDRESS` = `MAKE_ADDRESS(1, 254)` → **1:254** (fixed, hard-coded on root)
- [x] `BROADCAST_ADDRESS` = `0xFFFF` → **255:255**
- [x] `UNASSIGNED_ADDRESS` = `0xFFFE` → **255:254**
- [x] Treat **1:0** as reserved (`RESERVED_ADDRESS`)
- [x] Assignable range **1:1 through 1:253** (`address_is_assignable()`)
- [x] Root assumes discovery complete and starts with **1:254** (`ROLE_ROOT` / `BOOT_ADDRESS`)
- [x] Replace fixed compile-time `SENDER_ADDR` / `RECEIVER_ADDR` peer model (`ROLE_ROOT` / `ROLE_JOINING` + `local_address`)
- [x] Do **not** hard-code all participating node addresses (only root is fixed)
- [x] Do **not** assign all addresses manually at compile time (joiners boot unassigned)
- [x] Update IEEE 802.15.4 short address when a node becomes registered — hook ready (`set_local_address()` / `ieee802154_apply_short_address()`; join path will call it)

---

## 2. Node roles and finite-state machine

Required states (names may change; behavior must match):  
**Doc:** `documentation/SECTION2_NODE_FSM.md`

```text
UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
```

- [x] `enum node_state` with at least:
  - [x] `NODE_UNREGISTERED`
  - [x] `NODE_PROBING`
  - [x] `NODE_WAITING_FOR_JOIN_RESPONSE`
  - [x] `NODE_REGISTERED`
- [x] **UNREGISTERED:** no assigned address; cannot send normal DATA (`node_can_send_data()`)
- [x] **PROBING:** periodically broadcast PROBE; listen for HEARTBEAT (gated + joining `node_fsm_init`)
- [x] **WAITING_FOR_JOIN_RESPONSE:** entered after parent selection; JOIN_REQUEST outstanding
- [x] **REGISTERED:** root boots here; joiners enter via JOIN_RESPONSE + `node_enter_registered()`; DATA send loop still §4.7
- [x] Root starts already **REGISTERED** at **1:254** (hop 0, parent = self / `ROOT_ADDRESS`)
- [x] Non-root nodes start **UNREGISTERED** / enter **PROBING** (`node_fsm_init`)
- [x] Revert to UNREGISTERED on join timeout / registration abort (`node_revert_to_unregistered`)
- [x] Single firmware path that can be root **or** joining node (`ROLE_ROOT` / `ROLE_JOINING`)

### Current state vs FSM

- [x] Compile-time `ROLE_ROOT` / `ROLE_JOINING` with correct boot addresses + FSM
- [x] Full join / registration state machine (PROBE→HB→JOIN drives UNREGISTERED…REGISTERED; see `SECTION4_PROTOCOL_FLOW.md`)

---

## 3. Application message types

**Doc:** `documentation/SECTION3_MESSAGE_TYPES.md`  
Assignment numbering (aligned in code; `PROTOCOL_VERSION` **2**, 17-byte common header):

| Type | Value | Status |
|------|------:|--------|
| PROBE | 1 | [x] Defined + `handle_probe` |
| HEARTBEAT | 2 | [x] Defined + `handle_heartbeat` (body still GUID/battery) |
| JOIN_REQUEST | 3 | [x] `handle_join_request` (root allocates / retransmits) |
| JOIN_RESPONSE | 4 | [x] `handle_join_response` (joiner registers + ACK) |
| JOIN_ACKNOWLEDGE | 5 | [x] `handle_join_acknowledge` (root marks active) |
| DATA | 6 | [x] `handle_data` (decode/log; generate/forward later) |
| NETIDREQ | 7 | [x] `handle_netid_request` (reserved multi-net) |
| NETIDRESP | 8 | [x] `handle_netid_response` (reserved multi-net) |

Checklist:

- [x] PROBE type defined and handled
- [x] HEARTBEAT type defined and handled
- [x] JOIN_REQUEST handler
- [x] JOIN_RESPONSE handler
- [x] JOIN_ACKNOWLEDGE handler
- [x] DATA handler (generation / forwarding still §4 / M6)
- [x] NETIDREQ handler (reserved; logs only)
- [x] NETIDRESP handler (reserved; logs only)
- [x] Same serialize/deserialize format used consistently (`serialize_packet` / `deserialize_packet`)
- [x] Document final packet format (common header + type-specific fields/body) — see `SECTION3_MESSAGE_TYPES.md`

### Suggested / example fields (extend current OZ header or add bodies)

- [x] Signature (2) — `"OZ"`
- [x] Protocol version (1) — `PROTOCOL_VERSION` 2
- [x] Message type (1)
- [x] Source address (2)
- [x] Destination address (2)
- [x] Parent address (2) — `parent_id`
- [x] Assigned address (2) — `assigned_id`
- [x] Hop count (1) — `hop_count` (`0xFF` = unknown)
- [x] Sequence number (2)
- [x] Payload / sensor value (2) — `value` (DATA semantics / TX loop later)

---

## 4. Protocol flow

```text
PROBE → HEARTBEAT → JOIN REQUEST → JOIN RESPONSE → JOIN ACKNOWLEDGE
```

Then (registered): periodic **DATA** toward root.

**Doc:** `documentation/SECTION4_PROTOCOL_FLOW.md`

### 4.1 Probe (unregistered / probing node)

- [x] Periodically transmit **broadcast** PROBE
- [x] Source = unassigned address (`0xFFFE` / 255:254)
- [x] Destination = broadcast (`0xFFFF`)
- [x] Include sequence number
- [x] Retransmit PROBE on timeout if no eligible heartbeat
- [x] Revert toward unregistered / re-probe if join fails
- [x] Broadcast from unassigned (no longer fixed unicast peer model)

### 4.2 Heartbeat (registered node answering a probe)

- [x] Registered node responds to PROBE with HEARTBEAT
- [x] Registered source address
- [x] Parent address of responding node
- [x] Hop count from responder to root
- [x] Optional RSSI / LQI from the **received probe** (`value` = probe RSSI; GUID/battery body kept)
- [x] Any `REGISTERED` node may reply (`node_can_reply_heartbeat`)

### 4.3 Parent selection

- [x] Collect multiple heartbeats during a selection window
- [x] Document selection algorithm (`SECTION4_PROTOCOL_FLOW.md` §5)
- [x] Minimum policy:
  1. Lowest hop count
  2. If tie: strongest RSSI
  3. If tie / no RSSI: lowest address
- [x] Record selected parent before JOIN_REQUEST

### 4.4 Join request

- [x] Send JOIN_REQUEST after parent selection
- [x] Source = unassigned
- [x] Parent field = selected parent
- [x] Identify / direct request toward root (`destination = ROOT_ADDRESS`)
- [x] Include sequence number (+ join nonce in `value`)
- [x] Start response timer after TX
- [x] On failure: retransmit JOIN_REQUEST **or** return to UNREGISTERED / PROBING

### 4.5 Join response (root)

- [x] Root validates JOIN_REQUEST
- [x] Allocate unique address from pool
- [x] Response includes assigned address, selected parent, hop count
- [x] No duplicate allocations
- [x] Pending address reserved until ACK success or timeout
- [x] Address pool exhausted → report join failure
- [x] Duplicate request → **retransmit original response**, do not allocate again

### 4.6 Join acknowledge

- [x] On valid JOIN_RESPONSE: store assigned address + parent
- [x] Transition to REGISTERED
- [x] Send JOIN_ACKNOWLEDGE
- [x] Configure IEEE 802.15.4 short address when required
- [x] Do not send normal DATA before registration complete (`node_can_send_data`)
- [x] Root marks node **active** after ACK

### 4.7 Data collection (after registration)

- [x] Registered node generates sensor message **every 1 second** (`data_timer` / `TX_PERIOD_MS`)
- [x] Sensor value = sequential number (`sensor_value`)
- [x] Send / forward toward root (via parent) (`send_sensor_data` / `forward_data_to_parent`)
- [x] Root records and reports **missing** sensor data packets (`root_collect_data`)
- [x] Child DATA reaches root through parent (multi-hop when needed; origin `source_id` preserved)

---

## 5. Root address allocation

**Doc:** `documentation/SECTION5_ADDRESS_ALLOCATION.md`

```c
struct address_record {
    uint16_t address;
    uint16_t parent;
    uint8_t hop_count;
    uint16_t request_sequence;
    uint16_t join_nonce;   /* distinguishes concurrent 0xFFFE joiners */
    uint16_t requester_mac;
    bool pending;
    bool active;
    bool in_use;
    int64_t pending_deadline_ms;
};
```

- [x] Address-allocation table (or equivalent) on root
- [x] Start allocation at **1:1**
- [x] Skip reserved / broadcast / root addresses
- [x] Support **at least three** joining nodes (`ADDRESS_TABLE_CAPACITY` 16)
- [x] Reserve address while waiting for ACK (`pending`)
- [x] Mark `active` after JOIN_ACK
- [x] Reuse address only after failed transaction explicitly released
- [x] Concurrent requests: resolve so assignment stays accurate (no double-issue; keyed by join nonce)
- [x] Root allocates each live address only once while in use

---

## 6. Reliability requirements

- [x] Waiting for heartbeat → retransmit PROBE after timeout
- [x] Waiting for join response → retransmit JOIN_REQUEST **or** back to unregistered + PROBE
- [x] Waiting for join ACK → release pending address on timeout (retransmit JOIN_RESPONSE on duplicate REQUEST)
- [x] Duplicate JOIN_REQUEST → do not allocate a second address
- [x] Invalid packet (bad signature / version) → ignore + diagnostic log
- [x] Every control message includes a sequence number
- [x] Detect duplicate join requests and duplicate responses (nonce + sequence)
- [~] Probe K backoff / stale HB ignore retained; fuller loss demos still open

---

## 7. Implementation milestones (assignment §11)

Host-side dry-run (no radio): `python Host/milestone_sim.py`  
Single milestone: `python Host/milestone_sim.py --milestone N`  
Doc: `documentation/MILESTONE_SIM.md`

- [x] **M1** Packet format — serialization, version checks, types 1–8, parent/assigned/hop *(see §3 / `SECTION3_MESSAGE_TYPES.md`)*
- [x] **M2** Discovery — PROBE + HEARTBEAT *(broadcast + registered reply)*
- [x] **M3** Parent selection — joining node selects and records eligible parent
- [x] **M4** Address assignment — root JOIN_REQUEST → unique JOIN_RESPONSE
- [x] **M5** Registration — apply address + JOIN_ACKNOWLEDGE exchange
- [x] **M6** Data collection — registered nodes send/forward DATA to root
- [x] **M7** Reliability — timeouts, retransmit, duplicate handling, pending release, DATA gaps *(sim PASS; hardware demo still open)*

---

## 8. Required demonstration (§12)

Demonstrate **one root + at least two joining nodes**. Serial output must show progression like:

- [ ] Root starts with address **1:254**
- [ ] Node A sends PROBE
- [ ] Root sends HEARTBEAT
- [ ] Node A sends JOIN_REQUEST
- [ ] Root sends JOIN_RESPONSE assigning **1:1**
- [ ] Node A sends JOIN_ACKNOWLEDGE
- [ ] Node B (and C) join and receive **unique** addresses

Logs should clearly show when relevant:

- [ ] Message type
- [ ] Source address
- [ ] Destination address
- [ ] Assigned address
- [ ] Parent address
- [ ] Sequence number
- [ ] Node state
- [ ] RSSI when available

---

## 9. Required test cases (§13)

**How-to:** `documentation/TEST_CASES_HOWTO.md` (setup, RTT pass criteria, RF tips, sim fallbacks)

- [ ] **Test 1:** Single-node join — one unregistered node joins successfully
- [ ] **Test 2:** Multiple nodes — three or more nodes receive unique addresses
- [ ] **Test 3:** Simultaneous join — two nodes request at approximately the same time
- [ ] **Test 4:** Lost response — lost JOIN_RESPONSE causes request retransmit
- [ ] **Test 5:** Duplicate request — retransmit does not allocate a second address
- [ ] **Test 6:** No root — node continues probing and does not become registered
- [ ] **Test 7:** Invalid packet — bad signature or version ignored
- [ ] **Test 8:** Data forwarding — child DATA reaches root through parent

---

## 10. Required deliverables (§14)

- [~] Modified C source code *(in progress; join/tree not done)*
- [~] Protocol description and message-format table *(§3 doc done; full protocol writeup later)*
- [~] Node state-machine diagram *(§2 FSM doc; formal diagram later)*
- [ ] Address-allocation explanation
- [ ] Test results with serial-log evidence
- [ ] Wireshark screenshots or packet captures
- [ ] Short discussion of limitations and possible improvements

---

## 11. Design questions to answer in writeup (§15)

- [ ] 1. Why should an unregistered node use a special address?
- [ ] 2. Why must the root prevent duplicate address allocation?
- [ ] 3. What happens if two nodes select the same parent?
- [ ] 4. How can RSSI improve parent selection?
- [ ] 5. How should a node rejoin after losing contact with its parent?
- [ ] 6. What happens if the join acknowledgment is lost?
- [ ] 7. Should registered nodes respond to probes, transmit periodic heartbeats, or both?
- [ ] 8. How could the protocol support multiple network IDs?
- [ ] 9. How can the tree detect a failed parent?
- [ ] 10. What security problems exist in this joining protocol?

---

## 12. Implementation constraints (must not violate)

- [x] Preserve IEEE 802.15.4 config + packet-socket approach unless justified
- [x] Do not hard-code addresses of all participating nodes
- [x] Do not assign all addresses at compile time
- [x] Do not bypass the join protocol
- [x] Do not treat every packet as a DATA packet
- [x] Do not remove sequence numbers or timeout handling
- [~] Document any packet-format changes or Zephyr address-config assumptions *(§1–§4 docs; expand in final writeup)*
- [~] Final demo: addresses assigned **dynamically** through joining protocol *(code ready; hardware demo open)*

---

## 13. Grading rubric (track coverage)

| Component | Points | Progress |
|-----------|-------:|----------|
| Packet format and serialization | 15 | [x] §3 complete (v2 header + handlers) |
| Probe and heartbeat implementation | 15 | [x] Broadcast PROBE + REGISTERED HEARTBEAT |
| Parent selection | 10 | [x] Window + hop/RSSI/addr policy |
| Join request and response | 20 | [x] TX/RX + allocation |
| Address allocation | 15 | [x] Root table + pending/active |
| Join acknowledgment and state transitions | 10 | [x] REGISTERED + ACK activate |
| Reliability and duplicate handling | 10 | [x] Sim M7 PASS (retries/dup/pending/gaps); hardware logs open |
| Testing and documentation | 5 | [~] §4 doc done; hardware tests open |
| **Total** | **100** | |

---

## 14. Suggested implementation order

Use this as the working sequence:

1. [x] Address macros + root vs non-root startup + FSM skeleton + logging of state
2. [x] Align message-type numbers; extend packet fields (parent, assigned, hop)
3. [x] Rewrite PROBE (broadcast from unassigned) and HEARTBEAT (registered reply)
4. [x] Parent selection window + document policy (`SECTION4_PROTOCOL_FLOW.md`)
5. [x] JOIN_REQUEST / JOIN_RESPONSE / JOIN_ACK + root `address_record` table
6. [x] Apply short address on registration; block DATA until REGISTERED
7. [x] 1 Hz DATA + forward to parent + root missing-seq reporting
8. [~] Timers / retransmits / duplicates / failure recovery *(join + DATA gaps done; expand with demos)*
9. [ ] Hardware demo (1 root + ≥2 nodes) + Wireshark + writeup / design Qs

---

## 15. Quick status summary

| Area | Status |
|------|--------|
| Radio + OZ framing + RX/TX on all nodes | **Done** |
| Version / signature filtering | **Done** (v2) |
| Message types 1–8 + common header fields | **Done** (see `SECTION3_MESSAGE_TYPES.md`) |
| PROBE / HEARTBEAT (root listens; joining broadcasts) | **Done** |
| Addressing (`1:254`, unassigned, broadcast, pool helpers) | **Done** (see `SECTION1_ADDRESSING.md`) |
| Node FSM | **Done** (join-driven; see `SECTION2_NODE_FSM.md` + `SECTION4_PROTOCOL_FLOW.md`) |
| Parent selection | **Done** |
| Join request / response / ack | **Done** |
| Root address allocation table | **Done** (see §5 / `SECTION5_ADDRESS_ALLOCATION.md`) |
| DATA collection + forwarding | **Done** (1 Hz + parent forward + root missing-seq) |
| Join reliability / duplicates | **Sim M7 PASS; hardware evidence open** |
| Demo, tests, docs, Wireshark | **Host sim done; hardware demo open** |

**Bottom line:** M1–M7 logic verified via `Host/milestone_sim.py` (ALL PASS). Next: hardware demo (1 root + ≥2 nodes) / Wireshark / writeup.
