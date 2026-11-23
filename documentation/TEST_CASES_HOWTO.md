# Required Test Cases (§13) — How To

**Assignment:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §13  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §9  
**Related:** `SECTION4_PROTOCOL_FLOW.md`, `SECTION5_ADDRESS_ALLOCATION.md`, `SECTION6_RELIABILITY.md`, `MILESTONE_SIM.md`

This guide shows how to run each required hardware test, what RTT logs prove a pass, and when to use the host simulator as supporting evidence.

---

## 1. What you need

| Item | Notes |
|------|--------|
| Boards | At least **1 root + 3 joining** nRF52840 (`OzturkLab/nrf52840`) for Tests 2–3; **1 root + 2 joining** minimum for Test 8 multi-hop |
| Debugger / RTT | SEGGER J-Link RTT Viewer (or equivalent). Logs are **RTT**, not UART |
| Same radio config | All boards: `IEEE802154_CHANNEL` (default **11**) and `IEEE802154_PAN_ID` (`0x1234`) |
| Host PC (optional) | `python Host/milestone_sim.py` for logic dry-runs; `Host/nrf802154_serial_sniffer.py` for captures |

### Role selection (every flash)

Near the top of `src/main.c`:

```c
#define NODE_ROLE         ROLE_ROOT      /* one board only */
/* or */
#define NODE_ROLE         ROLE_JOINING   /* every other board */
```

Build / flash one board at a time (CMake board is set in `CMakeLists.txt`):

```text
west build -b OzturkLab/nrf52840 -p
west flash
```

Then change `NODE_ROLE`, rebuild, and flash the next board. Reset boards in the order described per test. Capture RTT from **root and joiners** when possible (two viewers / two sessions).

### Address / FSM cheat sheet

| Role | Boot address | Boot state |
|------|--------------|------------|
| Root | **1:254** (`0x01FE`) | `REGISTERED` |
| Joining | **255:254** (`0xFFFE`) unassigned | `UNREGISTERED` → `PROBING` |

Happy-path join:

```text
PROBE → HEARTBEAT → JOIN_REQUEST → JOIN_RESPONSE → JOIN_ACKNOWLEDGE → DATA (1 Hz)
```

Timers that matter for reliability tests:

| Constant | Value | Meaning |
|----------|------:|---------|
| `JOIN_RESPONSE_TIMEOUT_SECONDS` | 5 | Joiner waits for `JOIN_RESPONSE` |
| `JOIN_REQUEST_MAX_RETRIES` | 3 | Retransmits before revert to `PROBING` |
| `JOIN_ACK_PENDING_TIMEOUT_SECONDS` | 10 | Root releases pending address if no ACK |
| `PARENT_SELECTION_WINDOW_MS` | 2000 | Collect HEARTBEATs before parent pick |

### Optional: host simulator first

```text
python Host/milestone_sim.py              # M1–M7
python Host/milestone_sim.py --milestone 7
```

Sim PASS supports protocol logic; graded §13 evidence still needs board RTT (and preferably Wireshark / sniffer) where the test is RF-visible.

---

## 2. How to read a pass in the logs

`print_packet()` dumps type, addresses, parent, assigned, hop, sequence, and RSSI when present. Key strings:

| Event | Look for |
|-------|----------|
| State change | `STATE … -> …` |
| Probe | `PROBE TX` |
| Heartbeat | `HEARTBEAT TX` / `HEARTBEAT RX` / accepted |
| Parent pick | `Parent selected …` |
| Join TX/RX | `JOIN_REQUEST TX/RX`, `JOIN_RESPONSE TX/RX`, `JOIN_ACKNOWLEDGE TX/RX` |
| Allocation | `Allocated pending 1:n …` |
| Duplicate | `Duplicate JOIN_REQUEST nonce=…; retransmit 1:n` |
| Retransmit | `JOIN_RESPONSE timeout; retransmitting JOIN_REQUEST (r/3)` |
| Active | `Address 1:n marked ACTIVE` |
| DATA | `DATA TX`, `DATA FWD` / `DATA forwarded origin=…`, `DATA collected origin=…` |
| Bad packet | `Invalid packet ignored: bad signature …` or `bad version …` |

Save screenshots or copy RTT text into your test-results deliverable.

---

## 3. Test matrix (quick)

| # | Title | Boards | Hardest part |
|---|-------|--------|--------------|
| 1 | Single-node join | 1 root + 1 joiner | Baseline |
| 2 | Multiple unique addresses | 1 root + ≥3 joiners | Flash three `ROLE_JOINING` images |
| 3 | Simultaneous join | 1 root + 2 joiners | Power both joiners together |
| 4 | Lost JOIN_RESPONSE → retransmit | 1 root + 1 joiner | Force loss (RF or brief root off) |
| 5 | Duplicate request → no 2nd address | 1 root + 1 joiner | Often paired with Test 4 |
| 6 | No root → keep probing | 1 joiner only | Leave root off |
| 7 | Invalid signature / version | Any RX path | Inject bad frame or use sim + short HW note |
| 8 | Data forwarding via parent | 1 root + ≥2 joiners | Place child so parent ≠ root |

---

## 4. Test procedures

### Test 1 — Single-node join

**Goal:** One unregistered node joins successfully.

**Setup**

1. Flash Board R: `ROLE_ROOT`.
2. Flash Board A: `ROLE_JOINING`.
3. Power root first, attach RTT, reset. Confirm boot at **1:254** / `REGISTERED`.
4. Power / reset joiner A within range.

**Procedure**

1. Watch joiner: `UNREGISTERED` → `PROBING` → `PROBE TX`.
2. Watch root: `PROBE` → `HEARTBEAT TX`.
3. Joiner: parent selected (usually `1:254`), then `WAITING_FOR_JOIN_RESPONSE`, `JOIN_REQUEST TX`.
4. Root: `Allocated pending 1:1 …`, `JOIN_RESPONSE TX`.
5. Joiner: `JOIN_RESPONSE RX` with `assigned=1:1`, `REGISTERED`, `JOIN_ACKNOWLEDGE TX`.
6. Root: `Address 1:1 marked ACTIVE`.
7. Within ~1 s: joiner `DATA TX`; root `DATA collected origin=1:1 …`.

**Pass when**

- Full PROBE → … → ACK sequence appears.
- Joiner ends in `REGISTERED` with assigned **1:1** (first joiner).
- Root marks **1:1** ACTIVE and collects DATA.

**Fail if** joiner stays in `PROBING` forever (channel/PAN mismatch, root not up, out of range).

---

### Test 2 — Multiple nodes, unique addresses

**Goal:** Three or more nodes receive unique addresses.

**Setup**

1. One root + three joining boards (A, B, C), all `ROLE_JOINING` firmware.
2. Root up first. Bring joiners up one at a time (5–10 s apart is fine) **or** together (also covers Test 3).

**Procedure**

1. Capture root RTT through all three joins.
2. Record each `Allocated pending 1:n` and `Address 1:n marked ACTIVE`.
3. Confirm each joiner’s local assigned address in its RTT (`JOIN_RESPONSE` / `Registered`).

**Pass when**

- Root allocates distinct addresses, typically **1:1**, **1:2**, **1:3**.
- No two ACTIVE rows share the same address.
- Each joiner’s RTT shows a different assigned address.

**Evidence tip:** Keep a table: Board GUID / serial ↔ assigned `1:n` ↔ parent.

---

### Test 3 — Simultaneous join

**Goal:** Two nodes request addresses at approximately the same time; both succeed with unique addresses.

**Setup**

1. Root powered and idle.
2. Two joiners powered **off**, flashed with `ROLE_JOINING`.

**Procedure**

1. Start RTT on root (and joiners if possible).
2. Power or reset **both joiners within ~1–2 seconds**.
3. Expect overlapping `PROBE` / `JOIN_REQUEST` traffic.
4. Root uses **join nonce** (`JOIN_REQUEST.value`) so concurrent `0xFFFE` sources stay distinct (`SECTION5_ADDRESS_ALLOCATION.md`).

**Pass when**

- Both joiners reach `REGISTERED`.
- Root shows two different allocations (e.g. **1:1** and **1:2**) and both ACTIVE.
- No double allocation of the same `1:n` to both nonces.

**Notes**

- Parent selection window is 2 s; slight timing skew is OK.
- If one joiner loses the race and retries, that is still a pass as long as final addresses are unique and both register.

---

### Test 4 — Lost JOIN_RESPONSE causes retransmit

**Goal:** If `JOIN_RESPONSE` is lost, the joiner retransmits `JOIN_REQUEST`.

**Setup:** 1 root + 1 joiner (start from a clean join: reset both).

**Hardware methods (pick one)**

**A. Brief root silence (simplest)**

1. Let joiner reach `WAITING_FOR_JOIN_RESPONSE` / first `JOIN_REQUEST TX`.
2. Immediately **power off or hard-reset the root** for ~6–8 seconds (longer than `JOIN_RESPONSE_TIMEOUT_SECONDS` = 5).
3. Power root back on **before** retries are exhausted (≤3 × 5 s).
4. Joiner RTT should show:

   ```text
   JOIN_RESPONSE timeout; retransmitting JOIN_REQUEST (1/3)
   JOIN_REQUEST TX ... (same seq / same nonce)
   ```

5. After root returns, join should complete (or you re-run with shorter outage).

**B. RF attenuation**

1. Place boards at edge of range / metal shield between them only during the response window.
2. Harder to time; use only if you can correlate with sniffer.

**C. Host sim support**

```text
python Host/milestone_sim.py --milestone 7
```

M7 drops the first join attempt and checks retransmit recovery. Cite as logic proof; still try at least one hardware timeout log for the writeup.

**Pass when**

- Joiner logs `JOIN_RESPONSE timeout; retransmitting JOIN_REQUEST (r/3)`.
- Retransmit uses the **same sequence and nonce** (see `JOIN_REQUEST TX` lines).
- Preferably join eventually succeeds after the response path recovers.

**Related constant:** after more than `JOIN_REQUEST_MAX_RETRIES`, expect revert:

```text
JOIN_RESPONSE timeout after 3 retries; revert to PROBING
```

That path is useful for Test 6-style “no answer” behavior, but Test 4 specifically needs a **retransmit**, not only a full revert.

---

### Test 5 — Duplicate request does not allocate a second address

**Goal:** Retransmitting `JOIN_REQUEST` (same nonce) must **not** allocate a new address; root retransmits the original `JOIN_RESPONSE`.

**Best paired with Test 4**

1. Run Test 4 so the joiner retransmits while the root already allocated a pending row (root heard the first request, joiner never got the response — or ACK was delayed).
2. On root RTT look for:

   ```text
   Allocated pending 1:1 ... nonce=N ...
   ...
   Duplicate JOIN_REQUEST nonce=N; retransmit 1:1
   JOIN_RESPONSE TX ... assigned=1:1 ...
   ```

3. Confirm the address table did **not** grow to a second pending/active row for that nonce (only one `Allocated pending` for that join; duplicate line says **retransmit**, not a new allocation).

**Alternate procedure (ACK loss)**

1. Complete `JOIN_RESPONSE` delivery but prevent `JOIN_ACKNOWLEDGE` (power joiner off right after it enters `REGISTERED`, or power-cycle joiner before ACK TX if timing allows).
2. Bring joiner back so it re-joins with a **new** nonce (normal re-join) — that is a *new* allocation path, not Test 5.
3. For true duplicates you need the **same nonce** retransmit while still waiting / pending — Test 4’s timeout retransmit is the intended path (`join_response_timeout` keeps sequence + nonce).

**Pass when**

- Root logs `Duplicate JOIN_REQUEST nonce=…; retransmit 1:n`.
- Assigned address on the retransmitted response matches the first allocation.
- Only one ACTIVE (or pending) row for that joiner’s address.

**Sim:** M7 duplicate-nonce checks in `Host/milestone_sim.py`.

---

### Test 6 — No root: continue probing, never registered

**Goal:** With no root (and no other registered node), the joiner keeps probing and does not become registered.

**Setup**

1. Flash one board `ROLE_JOINING` only.
2. Do **not** power any `ROLE_ROOT` (and no already-registered relay).

**Procedure**

1. Reset joiner; attach RTT.
2. Observe for ≥30–60 seconds:

   ```text
   STATE UNREGISTERED -> PROBING
   PROBE TX ...
   ```

3. Probe interval K should grow after misses (`PROBE_K_INITIAL_SECONDS` → +`PROBE_N_INCREMENT_SECONDS`, capped at 30 s). Watch for probe-miss / K backoff messages in joining link stats logs.

**Pass when**

- State never reaches `WAITING_FOR_JOIN_RESPONSE` → `REGISTERED` (no HEARTBEAT → no parent → no join).
- Repeated `PROBE TX` continues.
- No `JOIN_RESPONSE RX` / no assigned address change from `255:254`.

**Fail if** another registered board is accidentally still powered nearby (it will HEARTBEAT).

---

### Test 7 — Invalid packet ignored (signature or version)

**Goal:** Bad signature or wrong protocol version is ignored with a diagnostic log.

**Code path:** `process_frame()` / `deserialize_packet()` — expects signature `"OZ"` and `PROTOCOL_VERSION` **2**.

#### Option A — Host simulator (always available)

```text
python Host/milestone_sim.py --milestone 7
```

M7 asserts invalid signature rejection (`invalid signature rejected (Test 7)`). Also covered by M1 packet-format checks.

#### Option B — Temporary one-shot TX for hardware evidence

Use only for the lab demo, then revert:

1. On a second joining board (or a throwaway build), temporarily change the TX path so one frame is sent with `signature[0]='X'` or `version=99` (e.g. after `build_packet`, overwrite those fields before `send_oz_packet`).
2. Keep a normal root or joiner as the receiver with RTT attached.
3. On the receiver expect:

   ```text
   Invalid packet ignored: bad signature 0xXX 0xXX
   ```

   or

   ```text
   Invalid packet ignored: bad version N (expected 2)
   ```

4. Confirm the receiver FSM does **not** change because of that frame.
5. **Revert** the temporary TX change before normal demos.

#### Option C — Sniffer / offline decode

If you capture frames with `Host/nrf802154_serial_sniffer.py`, document that the firmware drops non-`OZ` / wrong-version payloads using Option A/B evidence; the sniffer itself does not exercise `main.c`’s ignore path.

**Pass when**

- Diagnostic log appears for bad signature **or** bad version.
- No state transition / no allocation triggered by the bad frame.

---

### Test 8 — Data forwarding: child DATA reaches root through parent

**Goal:** A child’s DATA message reaches the root **through its parent** (multi-hop), preserving origin `source_id`.

**Why placement matters**

Parent selection prefers:

1. Lowest hop count  
2. Strongest RSSI  
3. Lowest address  

If the child can hear the root, it will pick **1:254** (hop 0) and DATA is one hop — that proves collection, **not** forwarding. For forwarding you need:

```text
Child 1:2  --DATA dest=1:1-->  Parent 1:1  --DATA dest=1:254-->  Root
           source stays 1:2                 source stays 1:2
```

**Setup**

1. Flash root + two joiners (A = future parent, B = child).
2. Bring up **root + A** first; let A join as **1:1** with parent **1:254**.
3. Place boards so **B hears A strongly but hears root weakly / not at all**:
   - Put root farther away or behind a shield.
   - Optionally lower `IEEE802154_TX_POWER_DBM` on root only for this test, then restore.
4. Power B; it should select parent **1:1** (hop 1), not root.

**Procedure**

1. On B: confirm `Parent selected 1:1 …`, join completes (`assigned` typically **1:2**), then `DATA TX` toward parent.
2. On A (intermediate): look for `DATA RX` then:

   ```text
   DATA FWD ...
   DATA forwarded origin=1:2 seq=… value=… -> 1:254
   ```

3. On root:

   ```text
   DATA collected origin=1:2 seq=… value=…
   ```

   Origin must be the **child** address, not the relay.

**Pass when**

- Intermediate logs `DATA forwarded origin=<child> …`.
- Root collects DATA with that same origin address.
- Child’s `source_id` is unchanged across the hop (`SECTION4_PROTOCOL_FLOW.md` §6).

**If B always selects the root:** increase distance / shielding until B’s selection log shows parent `1:1`. Direct child→root DATA alone does **not** satisfy Test 8.

**Sim support:** M6 in `milestone_sim.py` exercises child → relay → root with preserved origin.

---

## 5. Suggested run order (lab day)

1. **Test 6** (joiner alone) — quick negative case.  
2. **Test 1** — prove join works.  
3. **Test 2** — add third joiner; record unique addresses.  
4. **Test 3** — simultaneous power-on of two joiners.  
5. **Test 4 + 5** — lose response / show retransmit + duplicate nonce handling.  
6. **Test 7** — sim M7 + optional one-shot bad frame.  
7. **Test 8** — rearrange RF for multi-hop DATA.

Keep Wireshark / serial sniffer captures alongside RTT for the deliverable (§14).

---

## 6. Results template (copy into writeup)

```text
Test #:
Date:
Boards (role / GUID / assigned):
Channel / PAN / TX power:
Procedure notes:
Pass / Fail:
Key log lines (RTT):
  -
  -
Sniffer / Wireshark file:
```

Mark checklist items in `ASSIGNMENT_CHECKLIST.md` §9 when each test has saved evidence.

---

## 7. Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| No RADIO / no HEARTBEAT | Channel or PAN mismatch; root not up; out of range |
| Joiner stuck in `PROBING` | Test 6 setup accidentally; or root not answering |
| Always assigns only via root for Test 8 | Child still hears root — move / shield / lower root TX power |
| Two boards same address | Should not happen; check you did not flash two roots; capture nonce/alloc logs |
| No RTT | Attach J-Link RTT Viewer before/at reset; buffer is small |
| `JOIN_REQUEST rejected: parent … not root/active` | Parent not yet ACTIVE; wait for first joiner ACK before second joins through it |

---

## 8. Mapping to code (for the report)

| Test | Primary code |
|------|----------------|
| 1–3 | `run_probe_sender`, `handle_heartbeat`, `start_join_with_parent`, `handle_join_request`, `handle_join_response`, `handle_join_acknowledge` |
| 4 | `join_response_timeout`, `send_join_request` |
| 5 | `address_table_find_by_nonce`, `handle_join_request` duplicate branch |
| 6 | `probe_timer_expired`, `run_probe_sender` (no HEARTBEAT → no join) |
| 7 | `deserialize_packet`, `process_frame` |
| 8 | `send_sensor_data`, `forward_data_to_parent`, `root_collect_data` / `handle_data` |
