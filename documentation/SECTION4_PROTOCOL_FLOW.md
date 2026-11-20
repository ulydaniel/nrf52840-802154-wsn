# Section 4 — Protocol Flow (PROBE → JOIN → REGISTERED → DATA)

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §4 / §7 protocol flow  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §4.1–4.7  
**Depends on:** Section 1 addressing, Section 2 FSM, Section 3 message types

This document describes how joining nodes move through the FSM by exchanging PROBE, HEARTBEAT, JOIN_REQUEST, JOIN_RESPONSE, and JOIN_ACKNOWLEDGE, then send periodic DATA toward the root, and points to the code that implements each step.

---

## 1. Goal

Complete registration and data collection:

```text
UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
                                                      └─ 1 Hz DATA → parent → … → root
```

driven by:

```text
PROBE → HEARTBEAT → JOIN_REQUEST → JOIN_RESPONSE → JOIN_ACKNOWLEDGE → DATA
```

---

## 2. End-to-end flow

```text
Joining                          Root (or any REGISTERED node)
--------                         -----------------------------
PROBING
  |  broadcast PROBE (src=255:254, dst=255:255)
  |----------------------------->|
  |                              | node_can_reply_heartbeat()?
  |         HEARTBEAT            |  parent + hop + probe RSSI
  |<-----------------------------|
  |  collect candidates during
  |  PARENT_SELECTION_WINDOW_MS
  |  select best parent
  v
WAITING_FOR_JOIN_RESPONSE
  |  JOIN_REQUEST → root
  |  (parent=selected, value=nonce)
  |----------------------------->|
  |                              | allocate 1:n (pending)
  |         JOIN_RESPONSE        | assigned + parent + hop
  |<-----------------------------|
  |  set_local_address(assigned)
  |  → REGISTERED
  |  JOIN_ACKNOWLEDGE
  |----------------------------->|
  v                              | mark address ACTIVE
REGISTERED
  |  every TX_PERIOD_MS (1000 ms)
  |  DATA value=1,2,3… → parent
  |----------------------------->|  (or via intermediate parent)
  |                              | root_collect_data()
  |                              | report missing sequences
```

On JOIN_RESPONSE timeout: retransmit JOIN_REQUEST up to `JOIN_REQUEST_MAX_RETRIES`, then `node_revert_to_unregistered()` → `node_enter_probing()`.

---

## 3. What was already present vs newly wired

| Step | Before §4 work | After |
|------|----------------|-------|
| §4.1 Broadcast PROBE | Mostly done (`probe_timer_expired`) | Confirmed; fields forced unassigned; gated by FSM |
| §4.2 HEARTBEAT reply | Done for REGISTERED | Unchanged path; still carries parent/hop/RSSI |
| §4.3 Parent selection | Missing | Candidate table + selection window + policy |
| §4.4 JOIN_REQUEST TX | Handler log-only | TX + response timer + retries |
| §4.5 JOIN_RESPONSE | Handler log-only | Root address table + allocate + TX |
| §4.6 JOIN_ACK + REGISTERED | Helpers unused | `node_enter_registered` + ACK TX + activate |
| §4.7 DATA collection | Handler log-only | 1 Hz TX, parent forward, root missing-seq |

---

## 4. New / changed symbols in `main.c`

### 4.1 Constants (after probe K defines)

```c
#define PARENT_SELECTION_WINDOW_MS       2000
#define MAX_PARENT_CANDIDATES            8
#define JOIN_RESPONSE_TIMEOUT_SECONDS    5
#define JOIN_REQUEST_MAX_RETRIES         3
#define JOIN_ACK_PENDING_TIMEOUT_SECONDS 10
#define ADDRESS_TABLE_CAPACITY           16
#define JOINING_LOOP_POLL_MS             100
#define TX_PERIOD_MS                     1000   /* DATA period */
```

### 4.2 Joining-node state (`#if NODE_ROLE == ROLE_JOINING`)

| Symbol | Role |
|--------|------|
| `selection_timer` / `join_timer` | Selection window + JOIN_RESPONSE wait |
| `data_timer` | 1 Hz DATA after REGISTERED |
| `struct parent_candidate` | address, parent_id, hop, RX RSSI |
| `parent_candidates[]` | Up to 8 HEARTBEAT responders |
| `join_request_sequence` / `join_request_nonce` | Match response; distinguish concurrent joiners |
| `selected_parent_address` / `selected_parent_hop` | Chosen parent before JOIN_REQUEST |
| `sensor_value` / `data_tx_sequence` | Sequential sensor reading + DATA seq |
| `data_collection_active` | DATA timer armed after registration |

### 4.3 Root address table (`#if NODE_ROLE == ROLE_ROOT`)

```c
struct address_record {
    uint16_t address;
    uint16_t parent;
    uint8_t hop_count;
    uint16_t request_sequence;
    uint16_t join_nonce;      /* JOIN_REQUEST.value */
    uint16_t requester_mac;
    bool pending;
    bool active;
    bool in_use;
    int64_t pending_deadline_ms;
};
```

Helpers: `address_table_allocate()`, `address_table_find_by_nonce()`, `address_table_check_timeouts()`, `send_join_response_for_record()`.

**Why `join_nonce`?** All unregistered nodes share IEEE short address `0xFFFE`. Duplicate / concurrent joins cannot be keyed by MAC alone; the joiner puts a non-zero nonce in `JOIN_REQUEST.value`.

### 4.4 Root DATA collectors (`#if NODE_ROLE == ROLE_ROOT`)

```c
struct data_collector_entry {
    uint16_t source;          /* original child address */
    uint16_t last_sequence;
    bool sequence_valid;
    uint32_t received;
    uint32_t lost;
    int16_t last_value;
    bool in_use;
};
```

| Function | Role |
|----------|------|
| `data_collector_find` / `alloc` | Per-originator slot |
| `root_collect_data()` | Count RX, detect gaps via `track_sequence_gap()`, log missing |

### 4.5 Parent selection helpers (joining)

| Function | Role |
|----------|------|
| `record_parent_candidate()` | Insert/update HEARTBEAT responder |
| `parent_candidate_better()` | hop ↑ better, then RSSI ↑, then address ↑ |
| `select_best_parent()` | Pick winner from table |
| `parent_selection_window_expired()` | After 2 s → start join |
| `send_join_request()` | TX toward `ROOT_ADDRESS` |
| `join_response_timeout()` | Retry or revert to PROBING |
| `send_join_acknowledge()` | Confirm after REGISTERED |

### 4.6 DATA helpers (joining)

| Function | Role |
|----------|------|
| `start_data_collection()` | Called from `node_enter_registered()`; arms `data_timer` |
| `stop_data_collection()` | On revert / re-probe |
| `send_sensor_data()` | DATA to `parent_address`; value = sequential counter |
| `data_timer_expired()` | 1 Hz TX + restart timer |
| `forward_data_to_parent()` | Re-address child DATA to this node's parent; keep origin `source_id` |

### 4.7 FSM enter helpers (joining only)

- `node_enter_probing()` — boot + join failure retry  
- `node_enter_waiting_for_join_response()` — after parent selection  
- `node_enter_registered()` — short address + **starts DATA collection**  
- `node_revert_to_unregistered()` — timeout / registration abort; stops DATA  

### 4.8 Handler behavior changes

| Handler | Behavior |
|---------|----------|
| `handle_heartbeat` | Records candidates; opens selection window on first accept |
| `handle_join_request` | Root allocates or retransmits existing assignment |
| `handle_join_response` | Joiner → REGISTERED + sends JOIN_ACKNOWLEDGE |
| `handle_join_acknowledge` | Root marks row `active` |
| `handle_data` | Root: `root_collect_data()`; registered child/parent: `forward_data_to_parent()` |

### 4.9 Main loops

- **Joining** `run_probe_sender()` — 100 ms poll; services probe / selection / join / **data** timers by `current_state`  
- **Root** `run_probe_receiver()` — `address_table_check_timeouts()` + collect DATA  

---

## 5. Parent selection policy (§4.3)

1. **Lowest hop count** (unknown hop sorts last)  
2. If tie: **strongest RX RSSI** of the HEARTBEAT frame (`frame->rssi_dbm`)  
3. If tie / no RSSI: **lowest 16-bit address**

Implemented in `parent_candidate_better()`.

---

## 6. DATA collection design (§4.7)

### Generation (registered joining node)

- Period: `TX_PERIOD_MS` (1000 ms)  
- `value`: sequential `int16_t` starting at 1 after registration  
- `sequence`: independent DATA sequence (`data_tx_sequence`)  
- `source_id`: this node's assigned address  
- `destination_id` / MAC: **parent** (hop-by-hop toward root)  
- Gated by `node_can_send_data()` (REGISTERED + assigned address)

### Forwarding (registered non-root parent)

- Accept DATA whose `destination_id == local_address`  
- Copy packet; set `destination_id` / MAC to **this node's parent**  
- Preserve original `source_id`, `sequence`, and `value` so the root attributes the sample correctly  

### Root collection

- Key stats by originator `source_id`  
- On each sample: update last sequence; if gap > 0, log  
  `DATA missing from N:M: K packet(s) before seq=…`  
- Also log cumulative `lost / rx` per child  

### Multi-hop example

```text
Child 1:2  --DATA dest=1:1-->  Parent 1:1  --DATA dest=1:254-->  Root 1:254
           source stays 1:2                 source stays 1:2
```

(Requires a second joining node that selected `1:1` as parent. Direct child→root hops work with one joiner.)

---

## 7. Serial log progression (what you should see)

**Joining**

```text
FSM boot state=UNREGISTERED ...
STATE UNREGISTERED -> PROBING
PROBE TX ...
HEARTBEAT ... accepted ...
Parent selection window open for 2000 ms
Parent selected 1:254 hop=0 rssi=...
STATE PROBING -> WAITING_FOR_JOIN_RESPONSE
JOIN_REQUEST TX ...
JOIN_RESPONSE RX ... assigned=1:1 ...
STATE WAITING_FOR_JOIN_RESPONSE -> REGISTERED
JOIN_ACKNOWLEDGE TX ...
DATA collection started (period=1000 ms, value starts at 1)
DATA TX ... value=1
DATA TX ... value=2
...
```

**Root**

```text
FSM boot state=REGISTERED ...
PROBE ... → HEARTBEAT TX
JOIN_REQUEST RX ...
Allocated pending 1:1 ...
JOIN_RESPONSE TX ...
JOIN_ACKNOWLEDGE RX ...
Address 1:1 marked ACTIVE ...
DATA collector started for 1:1
DATA collected origin=1:1 seq=1 value=1 (lost=0 / rx=1)
DATA collected origin=1:1 seq=2 value=2 (lost=0 / rx=2)
...
DATA missing from 1:1: 1 packet(s) before seq=5 (total lost=1 / rx=4)   # if a gap occurs
```

---

## 8. Relationship to checklist sections

| Checklist | Covered by this work? |
|-----------|------------------------|
| §4.1–4.6 | **Yes** |
| §4.7 DATA | **Yes** |
| §5 Address allocation | **Yes** (root table) |
| §6 Full reliability suite | **Partial** — join retries + pending ACK + DATA gap logging; expand demos |

---

## 9. Still not done

- Multi-hop JOIN_REQUEST forwarding via intermediate parents (today JOIN goes directly to root)  
- Richer reliability demos (forced loss / Wireshark evidence)  
- Formal demo writeup (§8–§10) + design questions (§11)  

---

## 10. How to flash a two-board demo

1. Board A: `#define NODE_ROLE ROLE_ROOT` → build/flash  
2. Board B: `#define NODE_ROLE ROLE_JOINING` → build/flash  
3. Same channel / PAN ID  
4. Watch RTT for STATE transitions, then 1 Hz `DATA TX` / `DATA collected`  

Assigned addresses start at **1:1** and increase for additional joiners.
