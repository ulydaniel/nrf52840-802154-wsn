# Section 6 — Reliability Requirements

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §6  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §6  
**Depends on:** Section 2 FSM, Section 4 join / DATA flow, Section 5 address allocation

This document maps every §6 reliability requirement to the concrete timers, handlers, and recovery paths in `main.c`, including fixes made so those paths cannot leave a node stuck.

---

## 1. Goal

Loss and duplication are normal on IEEE 802.15.4. The application must:

1. Retransmit discovery / join control traffic on timeout  
2. Release unfinished address reservations on the root  
3. Never allocate a second address for a duplicate join  
4. Ignore malformed packets with a clear diagnostic  
5. Carry sequence numbers on every control message  
6. Detect duplicate join requests and duplicate join responses  

```text
PROBE timeout        → retransmit PROBE (K backoff)
JOIN_RESPONSE timeout → retransmit JOIN_REQUEST (≤3), else revert → PROBING
JOIN_ACK timeout     → root releases pending address
Duplicate REQUEST    → retransmit original JOIN_RESPONSE (same 1:n)
Bad sig / version    → ignore + log
```

---

## 2. Checklist map (§6)

| Requirement | Status | Primary code |
|-------------|:------:|--------------|
| Waiting for heartbeat → retransmit PROBE after timeout | Done | `probe_timer_expired()` |
| Waiting for join response → retransmit JOIN_REQUEST **or** back to unregistered + PROBE | Done | `join_response_timeout()` |
| Waiting for join ACK → release pending address on timeout | Done | `address_table_check_timeouts()` |
| Duplicate JOIN_REQUEST → do not allocate a second address | Done | `handle_join_request()` + nonce lookup |
| Invalid packet (bad signature / version) → ignore + diagnostic log | Done | `process_frame()` + `deserialize_packet()` |
| Every control message includes a sequence number | Done | `build_packet(..., sequence)` |
| Detect duplicate join requests and duplicate responses | Done | nonce (root) + sequence / outstanding (joiner) |
| Probe K backoff / stale HB ignore | Done (demos open) | `probe_k_seconds`, `handle_heartbeat()` |

---

## 3. Constants (timers and limits)

```c
#define PROBE_K_INITIAL_SECONDS          2
#define PROBE_N_INCREMENT_SECONDS        2
#define PROBE_K_MAX_SECONDS             30

#define PARENT_SELECTION_WINDOW_MS    2000
#define JOIN_RESPONSE_TIMEOUT_SECONDS    5
#define JOIN_REQUEST_MAX_RETRIES         3
#define JOIN_ACK_PENDING_TIMEOUT_SECONDS 10
```

| Symbol | Role |
|--------|------|
| `PROBE_K_*` / `PROBE_N_*` | Discovery retransmit interval; grows when no HEARTBEAT arrives |
| `JOIN_RESPONSE_TIMEOUT_SECONDS` | How long the joiner waits for `JOIN_RESPONSE` |
| `JOIN_REQUEST_MAX_RETRIES` | Retransmits before `node_revert_to_unregistered()` |
| `JOIN_ACK_PENDING_TIMEOUT_SECONDS` | Root releases pending `1:n` if ACK never arrives |

Zephyr one-shot timers used: `probe_timer`, `selection_timer`, `join_timer`, `data_timer`.

---

## 4. Waiting for heartbeat → retransmit PROBE

**Role:** `ROLE_JOINING`, state `NODE_PROBING`  
**Loop:** `run_probe_sender()` polls `probe_timer` every `JOINING_LOOP_POLL_MS`.

### Behavior

1. `probe_timer_expired()` runs when K expires.  
2. If a previous PROBE is still outstanding (`probe_outstanding`), count a miss, bump K by N (cap at 30 s), log, then send a new broadcast PROBE.  
3. Each PROBE gets a fresh `tx_sequence++` and stores `probe_outstanding_sequence`.  
4. Source = unassigned (`0xFFFE`); destination = broadcast (`0xFFFF`).  
5. While the parent-selection window is open, the timer is restarted **without** emitting another PROBE (candidates are still being collected).

### Stale HEARTBEAT ignore

In `handle_heartbeat()`:

- Matching sequence for the outstanding PROBE → accept, clear outstanding, reset K to initial.  
- Extra replies while `selection_window_active` → accept as candidates.  
- Otherwise → `heartbeats_stale++`, log, ignore.

### Key symbols

| Symbol / function | Purpose |
|-------------------|---------|
| `probe_outstanding` / `probe_outstanding_sequence` | Match HEARTBEAT to last PROBE |
| `probe_k_seconds` | Current retransmit period |
| `link_stats.probes_missed` / `heartbeats_stale` | Loss / stale counters |

---

## 5. Waiting for join response → retry or revert

**Role:** `ROLE_JOINING`, state `NODE_WAITING_FOR_JOIN_RESPONSE`  
**Functions:** `send_join_request()`, `join_response_timeout()`, `node_revert_to_unregistered()`, `node_enter_probing()`

### Happy path

1. Parent selected → `start_join_with_parent()`  
2. Assign `join_request_sequence = tx_sequence++` and a non-zero `join_request_nonce`  
3. `send_join_request()` TX → start `join_timer` for 5 s  
4. Valid `JOIN_RESPONSE` (matching sequence) → `node_enter_registered()` → `JOIN_ACKNOWLEDGE`

### Timeout path

```text
join_timer expires
        │
        ▼
join_request_retries++
        │
        ├── retries ≤ MAX (3) ──► retransmit JOIN_REQUEST
        │                         (same sequence + same nonce)
        │
        └── retries > MAX ─────► node_revert_to_unregistered()
                                  node_enter_probing()
                                  restart probe_timer
```

Retransmits keep the **same** sequence and nonce so:

- Root treats them as duplicates (nonce key)  
- Joiner still matches the echoed `JOIN_RESPONSE.sequence`

### Fix: retransmit TX failure must not stall

**Problem:** If `send_join_request()` failed after a timeout, the join timer was not restarted. The node stayed in `WAITING_FOR_JOIN_RESPONSE` with no deadline forever.

**Change in `join_response_timeout()`:**

```c
if (send_join_request()) {
    LOG_ERR("JOIN_REQUEST retransmit failed; restarting join timer");
    k_timer_start(
        &join_timer,
        K_SECONDS(JOIN_RESPONSE_TIMEOUT_SECONDS),
        K_NO_WAIT
    );
}
```

Initial TX failure in `start_join_with_parent()` already reverts to PROBING; that path was left unchanged.

---

## 6. Waiting for join ACK → release pending address

**Role:** `ROLE_ROOT` only  
**Functions:** `address_table_allocate()`, `address_table_check_timeouts()`, `address_table_release()`, `handle_join_acknowledge()`

### Lifecycle

```text
JOIN_REQUEST (new nonce)
    → allocate lowest free 1:n
    → pending=true, active=false, in_use=true
    → pending_deadline_ms = now + 10 s

JOIN_ACKNOWLEDGE (matching assigned)
    → pending=false, active=true

deadline passed with pending still set
    → LOG "JOIN_ACK timeout ... releasing"
    → address_table_release()  (memset row; address reusable)
```

`run_probe_receiver()` calls `address_table_check_timeouts()` on every loop iteration so pending rows cannot leak forever if ACK is lost.

Duplicate REQUEST while still pending **refreshes** `pending_deadline_ms` and retransmits the same `JOIN_RESPONSE` (see §7).

---

## 7. Duplicate JOIN_REQUEST → no second address

**Function:** `handle_join_request()`  
**Lookup:** `address_table_find_by_nonce(join_nonce)` where `join_nonce = packet->value`

| Case | Action |
|------|--------|
| Nonce already in table (pending or active) | Update sequence / MAC; refresh pending deadline if needed; **retransmit** original assignment; return |
| New nonce, parent OK, pool free | `address_table_allocate()` → new pending row → `JOIN_RESPONSE` |
| Nonce == 0 | Reject (log) |
| Parent not root/active | Reject (log) |
| Table full / pool exhausted | Fail (log); no allocation |

Concurrent joiners still share IEEE short address `0xFFFE`, so the **nonce**—not MAC alone—is the duplicate / concurrency key (see `SECTION5_ADDRESS_ALLOCATION.md`).

Comment in code was corrected to say “same join nonce” rather than “same MAC”.

---

## 8. Duplicate JOIN_RESPONSE detection (joiner)

**Function:** `handle_join_response()`

A response is accepted only when **all** hold:

1. `current_state == NODE_WAITING_FOR_JOIN_RESPONSE`  
2. `join_request_outstanding == true`  
3. `packet->sequence == join_request_sequence`  
4. `assigned_id` is assignable (`1:1` … `1:253`)

Otherwise it is logged and ignored. After success, `node_enter_registered()` → `clear_join_request_state()` clears outstanding / sequence / nonce, so a late duplicate cannot re-enter registration.

---

## 9. Invalid packet → ignore + diagnostic

**Functions:** `deserialize_packet()`, `process_frame()`

`deserialize_packet()` returns `false` when:

- Signature is not `'O''Z'`  
- `version != PROTOCOL_VERSION` (2)

### Change: distinguish diagnostics in `process_frame()`

Previously both failures logged a generic `Non-OZ packet ignored`. §6 / Test 7 expect a clear diagnostic for signature vs version:

```c
if (!deserialize_packet(frame->data, OZ_PACKET_SIZE, &packet)) {
    if ((frame->data[0] != 'O') || (frame->data[1] != 'Z')) {
        LOG_INF("Invalid packet ignored: bad signature 0x%02X 0x%02X", ...);
    } else {
        LOG_INF("Invalid packet ignored: bad version %u (expected %u)",
                ..., PROTOCOL_VERSION);
    }
    return;
}
```

Short frames (`length < OZ_PACKET_SIZE`) are ignored separately: `Short packet ignored: N bytes`.

---

## 10. Sequence numbers on every control message

All TX builders call `build_packet(..., sequence)`, which writes the 16-bit little-endian sequence field in the common OZ header.

| Message | Sequence source |
|---------|-----------------|
| PROBE | `tx_sequence++` |
| HEARTBEAT | Echoes PROBE sequence (header); body has `heartbeat_sequence` |
| JOIN_REQUEST | `join_request_sequence` (stable across retries) |
| JOIN_RESPONSE | Echoes request sequence |
| JOIN_ACKNOWLEDGE | Echoes response / request sequence |
| DATA | `data_tx_sequence++` |

Receivers use sequences for matching (join), gap tracking (PROBE / HEARTBEAT / DATA), and duplicate rejection.

---

## 11. Related DATA reliability (supports M7)

Not listed as a separate §6 bullet, but used for loss visibility after registration:

| Path | Function |
|------|----------|
| Root per-child sequence gaps | `root_collect_data()` → `DATA missing from …` |
| Probe / heartbeat link stats | `track_sequence_gap()`, `print_link_stats()` |

---

## 12. Code inventory (reliability-facing)

| Area | Symbols / functions |
|------|---------------------|
| Probe retransmit | `probe_timer`, `probe_timer_expired()`, `run_probe_sender()` |
| Join retry / revert | `join_timer`, `send_join_request()`, `join_response_timeout()`, `node_revert_to_unregistered()` |
| Pending ACK release | `address_table_check_timeouts()`, `address_table_release()` |
| Duplicate REQUEST | `address_table_find_by_nonce()`, `handle_join_request()`, `send_join_response_for_record()` |
| Duplicate RESPONSE | `handle_join_response()` sequence + outstanding checks |
| Invalid frames | `deserialize_packet()`, `process_frame()` |
| Sequence stamping | `build_packet()`, `tx_sequence`, `join_request_sequence`, `data_tx_sequence` |
| Stale HB / K backoff | `handle_heartbeat()`, `probe_k_seconds` |

---

## 13. Example RTT logs

**Joiner — PROBE miss / K backoff**

```text
PROBE TX ... seq=3
No HEARTBEAT for PROBE seq=3, K incremented to 4 s
PROBE TX ... seq=4
```

**Joiner — JOIN_RESPONSE lost → retry → success**

```text
JOIN_REQUEST seq=10 nonce=... (retry 0/3)
JOIN_RESPONSE timeout; retransmitting JOIN_REQUEST (1/3)
JOIN_REQUEST seq=10 nonce=... (retry 1/3)
JOIN_RESPONSE RX ... seq=10 assigned=1:1 ...
STATE WAITING_FOR_JOIN_RESPONSE -> REGISTERED
JOIN_ACKNOWLEDGE TX ...
```

**Joiner — retries exhausted**

```text
JOIN_RESPONSE timeout after 3 retries; revert to PROBING
Join failed; reverted
STATE ... -> UNREGISTERED
STATE UNREGISTERED -> PROBING
```

**Root — pending ACK timeout**

```text
Allocated pending 1:1 ... nonce=...
JOIN_ACK timeout for pending 1:1; releasing
Released address 1:1 (pending=1 active=0)
```

**Root — duplicate REQUEST**

```text
Duplicate JOIN_REQUEST nonce=...; retransmit 1:1
JOIN_RESPONSE TX ... assigned=1:1 ...
```

**Any node — invalid packet**

```text
Invalid packet ignored: bad signature 0x00 0x00
Invalid packet ignored: bad version 1 (expected 2)
```

---

## 14. Relationship to other sections

| Document | Overlap |
|----------|---------|
| `SECTION2_NODE_FSM.md` | Revert / PROBING / WAITING / REGISTERED transitions |
| `SECTION4_PROTOCOL_FLOW.md` | End-to-end join + DATA; reliability hooks called from those loops |
| `SECTION5_ADDRESS_ALLOCATION.md` | Pending / active / nonce duplicate policy |
| Checklist §6 / M7 / Tests 4–5 / 7 | Code done; hardware demo evidence still open |

---

## 15. Still open (not a code gap)

- Hardware / Wireshark demos for Test 4 (lost JOIN_RESPONSE), Test 5 (duplicate REQUEST), Test 7 (invalid packet)  
- Forced-loss tooling for classroom demos  
- Formal writeup answers that cite these logs (§11 design questions 2, 6, …)

---

## 16. How to exercise §6 on hardware

1. **Lost response:** power-cycle or move the root after JOIN_REQUEST; joiner should log retransmits then either succeed or revert to PROBING.  
2. **Duplicate request:** let retries fire with root online; root must log `Duplicate JOIN_REQUEST …; retransmit` and keep a single `1:n`.  
3. **Pending release:** kill the joiner after RESPONSE, before ACK; root should release the pending row after ~10 s.  
4. **Invalid packet:** inject or forge a frame with wrong signature/version; RTT must show the new diagnostic lines and no state change.  

Same channel / PAN ID as the rest of the tree demo (`SECTION4_PROTOCOL_FLOW.md` §10).
