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
- [~] Multi-threaded variant in `src/code.c` (builds if CMake pointed at it; not default)

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
- [x] **WAITING_FOR_JOIN_RESPONSE:** helper `node_enter_waiting_for_join_response()` ready; join TX not wired yet
- [x] **REGISTERED:** root boots here with parent/hop; DATA send loop still future
- [x] Root starts already **REGISTERED** at **1:254** (hop 0, parent = self / `ROOT_ADDRESS`)
- [x] Non-root nodes start **UNREGISTERED** / enter **PROBING** (`node_fsm_init`)
- [x] Revert to UNREGISTERED helper (`node_revert_to_unregistered`); call sites on join failure still future
- [x] Single firmware path that can be root **or** joining node (`ROLE_ROOT` / `ROLE_JOINING`)

### Current state vs FSM

- [x] Compile-time `ROLE_ROOT` / `ROLE_JOINING` with correct boot addresses + FSM
- [~] Full join / registration state machine (states + transitions exist; JOIN messages not driving them yet)

---

## 3. Application message types

**Doc:** `documentation/SECTION3_MESSAGE_TYPES.md`  
Assignment numbering (aligned in code; `PROTOCOL_VERSION` **2**, 17-byte common header):

| Type | Value | Status |
|------|------:|--------|
| PROBE | 1 | [x] Defined + `handle_probe` |
| HEARTBEAT | 2 | [x] Defined + `handle_heartbeat` (body still GUID/battery) |
| JOIN_REQUEST | 3 | [x] `handle_join_request` (decode/log; allocation later) |
| JOIN_RESPONSE | 4 | [x] `handle_join_response` (decode/log; registration later) |
| JOIN_ACKNOWLEDGE | 5 | [x] `handle_join_acknowledge` (decode/log; table activate later) |
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

### 4.1 Probe (unregistered / probing node)

- [ ] Periodically transmit **broadcast** PROBE
- [ ] Source = unassigned address (`0xFFFE` / 255:254)
- [ ] Destination = broadcast (`0xFFFF`)
- [ ] Include sequence number
- [ ] Retransmit PROBE on timeout if no eligible heartbeat
- [ ] Revert toward unregistered / re-probe if join fails
- [~] Current PROBE is unicast from fixed sender to fixed receiver — **rewrite**

### 4.2 Heartbeat (registered node answering a probe)

- [ ] Registered node responds to PROBE with HEARTBEAT
- [ ] Registered source address
- [ ] Parent address of responding node
- [ ] Hop count from responder to root
- [ ] Optional RSSI / LQI from the **received probe** (have RSSI plumbing)
- [~] Current HEARTBEAT is only from fixed receiver, with GUID/battery body — **rewrite fields / who may send**

### 4.3 Parent selection

- [ ] Collect multiple heartbeats during a selection window
- [ ] Document selection algorithm
- [ ] Minimum policy:
  1. Lowest hop count
  2. If tie: strongest RSSI
  3. If tie / no RSSI: lowest address
- [ ] Record selected parent before JOIN_REQUEST

### 4.4 Join request

- [ ] Send JOIN_REQUEST after parent selection
- [ ] Source = unassigned
- [ ] Parent field = selected parent
- [ ] Identify / direct request toward root (per protocol design)
- [ ] Include sequence number
- [ ] Start response timer after TX
- [ ] On failure: retransmit JOIN_REQUEST **or** return to UNREGISTERED / PROBING

### 4.5 Join response (root)

- [ ] Root validates JOIN_REQUEST
- [ ] Allocate unique address from pool
- [ ] Response includes assigned address, selected parent, hop count
- [ ] No duplicate allocations
- [ ] Pending address reserved until ACK success or timeout
- [ ] Address pool exhausted → report join failure
- [ ] Duplicate request → **retransmit original response**, do not allocate again

### 4.6 Join acknowledge

- [ ] On valid JOIN_RESPONSE: store assigned address + parent
- [ ] Transition to REGISTERED
- [ ] Send JOIN_ACKNOWLEDGE
- [ ] Configure IEEE 802.15.4 short address when required
- [ ] Do not send normal DATA before registration complete
- [ ] Root marks node **active** after ACK

### 4.7 Data collection (after registration)

- [ ] Registered node generates sensor message **every 1 second**
- [ ] Sensor value = sequential number
- [ ] Send / forward toward root (via parent)
- [ ] Root records and reports **missing** sensor data packets
- [ ] Child DATA reaches root through parent (multi-hop when needed)

---

## 5. Root address allocation

```c
struct address_record {
    uint16_t address;
    uint16_t parent;
    uint8_t hop_count;
    uint16_t request_sequence;
    bool pending;
    bool active;
};
```

- [ ] Address-allocation table (or equivalent) on root
- [ ] Start allocation at **1:1**
- [ ] Skip reserved / broadcast / root addresses
- [ ] Support **at least three** joining nodes
- [ ] Reserve address while waiting for ACK (`pending`)
- [ ] Mark `active` after JOIN_ACK
- [ ] Reuse address only after failed transaction explicitly released
- [ ] Concurrent requests: resolve so assignment stays accurate (no double-issue)
- [ ] Root allocates each live address only once while in use

---

## 6. Reliability requirements

- [ ] Waiting for heartbeat → retransmit PROBE after timeout
- [ ] Waiting for join response → retransmit JOIN_REQUEST **or** back to unregistered + PROBE
- [ ] Waiting for join ACK → retransmit JOIN_RESPONSE **or** release pending address on timeout
- [ ] Duplicate JOIN_REQUEST → do not allocate a second address
- [ ] Invalid packet (bad signature / version) → ignore + diagnostic log
- [ ] Every control message includes a sequence number
- [ ] Detect duplicate join requests and duplicate responses
- [~] Current probe K backoff / stale HB ignore — useful patterns, not full join reliability

---

## 7. Implementation milestones (assignment §11)

- [x] **M1** Packet format — serialization, version checks, types 1–8, parent/assigned/hop *(see §3 / `SECTION3_MESSAGE_TYPES.md`)*
- [~] **M2** Discovery — PROBE + HEARTBEAT *(broadcast path present; parent-selection rewrite still §4)*
- [ ] **M3** Parent selection — joining node selects and records eligible parent
- [ ] **M4** Address assignment — root JOIN_REQUEST → unique JOIN_RESPONSE
- [ ] **M5** Registration — apply address + JOIN_ACKNOWLEDGE exchange
- [ ] **M6** Data collection — registered nodes send/forward DATA to root
- [ ] **M7** Reliability — timeouts, retransmissions, duplicates, failure recovery demonstrated

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
- [ ] Do not hard-code addresses of all participating nodes
- [ ] Do not assign all addresses at compile time
- [ ] Do not bypass the join protocol
- [ ] Do not treat every packet as a DATA packet
- [ ] Do not remove sequence numbers or timeout handling
- [ ] Document any packet-format changes or Zephyr address-config assumptions
- [ ] Final demo: addresses assigned **dynamically** through joining protocol

---

## 13. Grading rubric (track coverage)

| Component | Points | Progress |
|-----------|-------:|----------|
| Packet format and serialization | 15 | [x] §3 complete (v2 header + handlers) |
| Probe and heartbeat implementation | 15 | [~] Present; needs assignment semantics |
| Parent selection | 10 | [ ] |
| Join request and response | 20 | [ ] |
| Address allocation | 15 | [ ] |
| Join acknowledgment and state transitions | 10 | [ ] |
| Reliability and duplicate handling | 10 | [ ] Partial probe-only patterns |
| Testing and documentation | 5 | [ ] |
| **Total** | **100** | |

---

## 14. Suggested implementation order

Use this as the working sequence:

1. [x] Address macros + root vs non-root startup + FSM skeleton + logging of state
2. [x] Align message-type numbers; extend packet fields (parent, assigned, hop)
3. [~] Rewrite PROBE (broadcast from unassigned) and HEARTBEAT (registered reply) — mostly done; parent-selection fields already on HB header
4. [ ] Parent selection window + document policy
5. [ ] JOIN_REQUEST / JOIN_RESPONSE / JOIN_ACK + root `address_record` table
6. [ ] Apply short address on registration; block DATA until REGISTERED
7. [ ] 1 Hz DATA + forward to parent + root missing-seq reporting
8. [ ] Timers / retransmits / duplicates / failure recovery
9. [ ] Hardware demo (1 root + ≥2 nodes) + Wireshark + writeup / design Qs

---

## 15. Quick status summary

| Area | Status |
|------|--------|
| Radio + OZ framing + RX/TX on all nodes | **Done** |
| Version / signature filtering | **Done** (v2) |
| Message types 1–8 + common header fields | **Done** (see `SECTION3_MESSAGE_TYPES.md`) |
| PROBE / HEARTBEAT (root listens; joining broadcasts) | **Partial — parent selection / join TX still §4** |
| Addressing (`1:254`, unassigned, broadcast, pool helpers) | **Done** (see `SECTION1_ADDRESSING.md`) |
| Node FSM | **Done** (see `SECTION2_NODE_FSM.md`; join-driven transitions pending) |
| Parent selection | **Not started** |
| Join request / response / ack | **Handlers decode/log; TX + allocation not started** |
| Root address allocation table | **Not started** |
| DATA collection + forwarding | **Handler only; generate/forward not started** |
| Join reliability / duplicates | **Not started** |
| Demo, tests, docs, Wireshark | **Not started** |

**Bottom line:** Types + packet format are assignment-aligned. Next: parent selection and JOIN TX/RX driving the FSM (§4 / M3–M5), then DATA tree (M6).
