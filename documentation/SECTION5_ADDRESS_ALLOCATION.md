# Section 5 — Root Address Allocation

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §5 / address allocation  
**Code:** `src/main.c` (root-only block `#if NODE_ROLE == ROLE_ROOT`)  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §5  
**Depends on:** Section 1 addressing, Section 2 FSM, Section 3 message types, Section 4 join flow (§4.5–4.6)

This document describes the root’s address-allocation table: how unique node IDs are reserved, confirmed, released on failure, and kept collision-free when several unregistered joiners share short address `0xFFFE`.

---

## 1. Goal

Only the root may hand out network addresses. While a join is in progress the chosen address is **pending**; after a valid `JOIN_ACKNOWLEDGE` it becomes **active**. Failed joins release the row so the address can be reused.

```text
JOIN_REQUEST  →  allocate lowest free 1:n (pending)
JOIN_RESPONSE →  tell joiner assigned + parent + hop
JOIN_ACK      →  mark active
ACK timeout   →  release pending row (reuse allowed)
```

---

## 2. Why a table (and why a join nonce)

| Problem | Solution in this project |
|---------|--------------------------|
| Many nodes may join | Fixed-size `address_table[]` on root |
| Same address must not be issued twice while live | `in_use` + `address_table_address_taken()` |
| Joiners share IEEE short address `0xFFFE` until registered | Duplicate / concurrent joins keyed by **`join_nonce`** in `JOIN_REQUEST.value`, not by MAC alone |
| ACK may be lost | Row stays `pending` until ACK or `JOIN_ACK_PENDING_TIMEOUT_SECONDS` |
| Retransmitted JOIN_REQUEST | Look up by nonce → **retransmit same** `JOIN_RESPONSE`; never allocate again |

---

## 3. Record layout (assignment §5)

Compiled only for `ROLE_ROOT`:

```c
struct address_record {
    uint16_t address;              /* assigned 1:1 .. 1:253 */
    uint16_t parent;               /* parent from JOIN_REQUEST */
    uint8_t  hop_count;            /* child hop = parent hop + 1 */
    uint16_t request_sequence;     /* last JOIN_REQUEST sequence */
    uint16_t join_nonce;           /* JOIN_REQUEST.value; concurrent key */
    uint16_t requester_mac;        /* IEEE short addr of requester */
    bool     pending;              /* waiting for JOIN_ACKNOWLEDGE */
    bool     active;               /* ACK received; live member */
    bool     in_use;               /* slot occupied */
    int64_t  pending_deadline_ms;  /* uptime deadline for ACK */
};

static struct address_record address_table[ADDRESS_TABLE_CAPACITY];
```

| Field | Role |
|-------|------|
| `address` | Network address handed to the joiner |
| `parent` | Tree parent chosen by the joiner |
| `hop_count` | Distance from root after join (`parent_hop + 1`) |
| `request_sequence` | Echoed in `JOIN_RESPONSE.sequence` so the joiner can match |
| `join_nonce` | Distinguishes concurrent `0xFFFE` joiners |
| `requester_mac` | Last seen MAC (logging / response targeting) |
| `pending` / `active` / `in_use` | Lifecycle flags |
| `pending_deadline_ms` | Absolute uptime when pending must be released |

---

## 4. Constants

| Symbol | Value | Meaning |
|--------|------:|---------|
| `ADDRESS_TABLE_CAPACITY` | 16 | Max simultaneous pending+active rows (≥3 required) |
| `JOIN_ACK_PENDING_TIMEOUT_SECONDS` | 10 | Pending release if no ACK |
| `NODE_ID_FIRST_ASSIGNABLE` | 1 | First allocated node ID → **1:1** |
| `NODE_ID_LAST_ASSIGNABLE` | 253 | Last pool ID → **1:253** |
| `NETWORK_ID_ROOT` | 1 | Only root network in this milestone |

Assignable helpers (Section 1): `address_is_assignable()`, `ADDRESS_FIRST_ASSIGNABLE` / `ADDRESS_LAST_ASSIGNABLE`. Reserved (`1:0`), root (`1:254`), unassigned (`0xFFFE`), and broadcast (`0xFFFF`) are never allocated.

---

## 5. Lifecycle of one allocation

```text
                  address_table_allocate()
                           |
                           v
                    +-------------+
                    |   PENDING   |  in_use=1, pending=1, active=0
                    |  deadline   |  JOIN_RESPONSE sent
                    +------+------+
                           |
           +---------------+----------------+
           |                                |
           v                                v
   JOIN_ACKNOWLEDGE                 pending timeout
   handle_join_acknowledge()        address_table_check_timeouts()
           |                                |
           v                                v
    +-------------+                  address_table_release()
    |   ACTIVE    |                  slot cleared (reuse OK)
    | pending=0   |
    | active=1    |
    +-------------+
```

Duplicate `JOIN_REQUEST` with the **same nonce** while the row exists (pending or active) only **retransmits** the original assignment.

---

## 6. API map in `main.c`

| Function | Role |
|----------|------|
| `address_table_find_by_nonce()` | Locate in-use row by `join_nonce` (0 never matches) |
| `address_table_find_by_assigned()` | Locate row by assigned `1:n` |
| `address_table_parent_hop()` | Hop of parent (0 if root; else active parent’s hop) |
| `address_table_parent_ok()` | Parent must be root or an **active** table entry |
| `address_table_alloc_slot()` | First free `!in_use` index |
| `address_table_address_taken()` | True if address already in a live row |
| `address_table_allocate()` | Lowest free `1:1`…`1:253` → pending row |
| `address_table_release()` | Clear one row (`memset`) |
| `address_table_check_timeouts()` | Release expired pending rows |
| `send_join_response_for_record()` | Build/TX `JOIN_RESPONSE` from a record |

Called from:

| Caller | When |
|--------|------|
| `handle_join_request()` | Validate → find-by-nonce or allocate → response |
| `handle_join_acknowledge()` | Find-by-assigned → mark active |
| `run_probe_receiver()` | Each loop iteration → `address_table_check_timeouts()` |

---

## 7. Allocation algorithm

`address_table_allocate(parent, child_hop, request_sequence, join_nonce, requester_mac, &out)`:

1. Take a free table slot (`-ENOMEM` if full).
2. Scan `node_id` from **1** to **253**; pick first `MAKE_ADDRESS(1, node_id)` not already `in_use`.
3. If none free → `-ENOSPC` (“Address pool exhausted”).
4. Fill record: `pending=true`, `active=false`, `in_use=true`, deadline = now + 10 s.
5. Log: `Allocated pending 1:n parent=… hop=… nonce=…`.

Pool exhaustion and table-full are both reported as join failure (no `JOIN_RESPONSE`).

---

## 8. JOIN_REQUEST handling (root)

`handle_join_request()` (root path):

1. Ignore unless local role is root and FSM is `NODE_REGISTERED`.
2. Reject `join_nonce == 0`.
3. Reject if `parent_id` is not root and not an **active** table entry (`address_table_parent_ok`).
4. **If nonce already in table:** refresh sequence / MAC / pending deadline; retransmit `JOIN_RESPONSE` for that address; return.
5. Else compute `child_hop = parent_hop + 1` (reject if parent hop unknown).
6. `address_table_allocate(…)` then `send_join_response_for_record(…)`.

`JOIN_RESPONSE` fields:

| Field | Value |
|-------|--------|
| `source_id` | Root (`local_address` / 1:254) |
| `destination_id` | `UNASSIGNED_ADDRESS` (app layer) |
| MAC destination | Requester’s `frame->mac_source` |
| `assigned_id` | Record’s `address` |
| `parent_id` | Record’s `parent` |
| `hop_count` | Record’s `hop_count` |
| `sequence` | Echo of request sequence (joiner match) |

---

## 9. JOIN_ACKNOWLEDGE handling (root)

`handle_join_acknowledge()`:

1. Look up `packet->assigned_id` in the table.
2. Unknown address → warn and ignore.
3. Set `pending=false`, `active=true`; refresh parent / hop / MAC from the ACK.
4. Log: `Address 1:n marked ACTIVE …`.

Joiner side (Section 4): after a sequence-matched `JOIN_RESPONSE`, `node_enter_registered()` applies the short address, then `send_join_acknowledge()` toward the root.

---

## 10. Concurrent joins

```text
Joiner A (0xFFFE, nonce=0xA1A1)     Joiner B (0xFFFE, nonce=0xB2B2)     Root
        |                                    |                            |
        |-------- JOIN_REQUEST nonce=A ----->|                            |
        |                                    |-------- JOIN_REQUEST B --->|
        |                                    |                            | allocate 1:1 for A
        |                                    |                            | allocate 1:2 for B
        |<------- JOIN_RESPONSE 1:1 ---------|----------------------------|
        |                                    |<----- JOIN_RESPONSE 1:2 ---|
        |-------- JOIN_ACK 1:1 ------------->|--------------------------->| A active
        |                                    |-------- JOIN_ACK 1:2 ----->| B active
```

- Nonces come from `start_join_with_parent()`: `k_cycle_get_32() ^ (uptime<<1) ^ tx_sequence`, forced non-zero.
- Joiner accepts a response only if `packet->sequence == join_request_sequence`, so A does not install B’s address.

---

## 11. Failure and reuse

| Event | Root behavior |
|-------|----------------|
| No ACK within 10 s | `address_table_check_timeouts()` → `address_table_release()`; address free again |
| Duplicate REQUEST (same nonce) | Retransmit original response; deadline refreshed if still pending |
| Table full (16 live rows) | `-ENOMEM`; no response |
| Pool 1:1..1:253 all taken | `-ENOSPC`; no response |
| Bad parent / zero nonce | Reject; no allocation |

Active rows are **not** automatically reclaimed when a registered node leaves the radio range; §5 only requires reuse after an explicitly failed pending transaction.

---

## 12. Checklist coverage (§5)

| Requirement | Status | Where |
|-------------|:------:|-------|
| Address-allocation table on root | Done | `address_table[]` / `struct address_record` |
| Start allocation at **1:1** | Done | Scan from `NODE_ID_FIRST_ASSIGNABLE` |
| Skip reserved / broadcast / root | Done | Only 1:1..1:253 candidates |
| Support ≥ three joining nodes | Done | Capacity 16 |
| Reserve while waiting for ACK | Done | `pending` + deadline |
| Mark `active` after JOIN_ACK | Done | `handle_join_acknowledge()` |
| Reuse only after failed TX released | Done | Timeout → `address_table_release()` |
| Concurrent requests stay accurate | Done | Keyed by `join_nonce` |
| Each live address issued once while in use | Done | `address_table_address_taken()` |

---

## 13. Serial log cues (root)

```text
JOIN_REQUEST RX ...
Allocated pending 1:1 parent=1:254 hop=1 nonce=... mac=0xFFFE seq=...
JOIN_RESPONSE TX ...
JOIN_ACKNOWLEDGE RX ...
Address 1:1 marked ACTIVE (parent=1:254 hop=1)

# on lost ACK:
JOIN_ACK timeout for pending 1:1; releasing
Released address 1:1 (pending=1 active=0)

# on retransmit:
Duplicate JOIN_REQUEST nonce=...; retransmit 1:1
```

Second / third joiners receive **1:2**, **1:3**, … while earlier rows remain `in_use`.

---

## 14. Relationship to other sections

| Section | Link |
|---------|------|
| §1 Addressing | Pool bounds, `MAKE_ADDRESS`, `address_is_assignable()` |
| §2 FSM | Root stays `REGISTERED`; joiner becomes `REGISTERED` after response |
| §3 Messages | Types 3–5 carry parent / assigned / hop / sequence / value(nonce) |
| §4 Protocol flow | End-to-end PROBE→…→ACK; this file details the root table only |
| §6 Reliability | Pending timeout + duplicate REQUEST handling (partial suite) |

---

## 15. Limits / not in this milestone

- Table capacity (16) can fill before the 253-address pool is exhausted.
- No reclaim of **active** addresses on parent loss / leave.
- `JOIN_REQUEST` is sent directly to the root (no multi-hop join forwarding).
- Hardware demo of simultaneous three-node join is still under checklist §8–§9.

---

## 16. How to exercise allocation

1. Flash board A with `#define NODE_ROLE ROLE_ROOT`.
2. Flash boards B (and C) with `ROLE_JOINING` (same channel / PAN).
3. On root RTT, confirm `Allocated pending 1:1`, then `1:2`, … and matching `marked ACTIVE` lines.
4. Optionally power-cycle a joiner mid-ACK to observe pending timeout release and later reuse of the same `1:n`.
