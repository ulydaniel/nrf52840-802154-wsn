# Section 2 — Node Finite-State Machine

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §5 Required Node States  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §2  
**Depends on:** Section 1 addressing (`local_address`, root / joining roles)

This document describes the protocol FSM: where it lives, how states are entered, and which behaviors are gated by state today.

---

## 1. Required progression

```text
UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
```

| State | Required behavior (assignment) | In this project now |
|-------|--------------------------------|---------------------|
| `NODE_UNREGISTERED` | No assigned address; cannot send normal DATA | Boot state for joining nodes; DATA blocked |
| `NODE_PROBING` | Periodically broadcast PROBE; listen for HEARTBEAT | Joining nodes enter this after radio up; probe timer gated |
| `NODE_WAITING_FOR_JOIN_RESPONSE` | Parent selected; waiting for assigned address | Helper ready (`node_enter_waiting_for_join_response`); join TX not yet |
| `NODE_REGISTERED` | Address + parent + route; send periodic sensor DATA | Root boots here; DATA still future milestone |

---

## 2. Where it is in `main.c`

### Enum and boot constants

Near the top, after the addressing macros:

```c
enum node_state {
    NODE_UNREGISTERED = 0,
    NODE_PROBING,
    NODE_WAITING_FOR_JOIN_RESPONSE,
    NODE_REGISTERED,
};
```

Role-specific boot values:

| Role | `BOOT_ADDRESS` | `BOOT_NODE_STATE` | `BOOT_PARENT_ADDRESS` | `BOOT_HOP_COUNT` |
|------|----------------|-------------------|------------------------|------------------|
| `ROLE_ROOT` | `1:254` | `NODE_REGISTERED` | `ROOT_ADDRESS` (self) | `0` |
| `ROLE_JOINING` | `255:254` | `NODE_UNREGISTERED` | `UNASSIGNED_ADDRESS` | `0xFF` (`HOP_COUNT_UNKNOWN`) |

### Runtime globals

```c
static enum node_state current_state = BOOT_NODE_STATE;
static uint16_t parent_address = BOOT_PARENT_ADDRESS;
static uint8_t hop_count = BOOT_HOP_COUNT;
```

Together with `local_address` (Section 1), these form the registration context the join protocol will update.

### Helpers (section “Node FSM helpers”)

| Function | Role |
|----------|------|
| `node_state_name()` | State → string for logs |
| `log_node_state()` | Log state + addr + parent + hop |
| `set_node_state()` | Transition + `STATE old -> new` log |
| `node_can_send_data()` | `REGISTERED` and not unassigned |
| `node_can_send_probe()` | `PROBING` only |
| `node_can_reply_heartbeat()` | `REGISTERED` only |
| `node_enter_probing()` | Clear join context; `→ PROBING` |
| `node_revert_to_unregistered()` | Fail/rejoin path; restore unassigned |
| `node_enter_waiting_for_join_response()` | After parent selection (join later) |
| `node_enter_registered()` | Apply assigned address + `→ REGISTERED` |
| `node_fsm_init()` | Called from `main()` after radio context is up |

---

## 3. Boot sequence

```text
main()
  ├─ banner (prints FSM State, parent, hop, DATA allowed)
  ├─ configure_ieee802154()     // short addr = local_address
  ├─ create_receive_context()
  ├─ node_fsm_init()
  │     ├─ restore BOOT_* context
  │     ├─ log "FSM boot"
  │     └─ if ROLE_JOINING and UNREGISTERED:
  │           node_enter_probing()   // STATE UNREGISTERED -> PROBING
  └─ run_probe_sender() or run_probe_receiver()
```

**Root** stays `REGISTERED` for the whole run (until a future failure/rejoin design changes that).

**Joining** always shows in the log:

```text
FSM boot state=UNREGISTERED ...
STATE UNREGISTERED -> PROBING
FSM state=PROBING ...
```

---

## 4. How state gates behavior today

```text
                    PROBE RX
                       │
                       ▼
              node_can_reply_heartbeat()?
                 │              │
                yes            no
                 │              │
                 ▼              ▼
           send HEARTBEAT   ignore + log state

                    HEARTBEAT RX
                       │
                       ▼
              current_state == PROBING?
                 │              │
                yes            no
                 │              ▼
                 ▼           ignore
           match seq / reset K

                    probe timer
                       │
                       ▼
              node_can_send_probe()?
                 │              │
                yes            no
                 │              ▼
                 ▼           ignore
           broadcast PROBE
```

Concrete call sites:

- `handle_probe()` — replies only if `node_can_reply_heartbeat()`
- `handle_heartbeat()` — accepted only in `NODE_PROBING` (for now)
- `probe_timer_expired()` — sends only if `node_can_send_probe()`
- Banner / loops — `node_can_send_data()`, `log_node_state()`

---

## 5. Intended later transitions (helpers ready)

```text
Joining:
  UNREGISTERED
       │  node_fsm_init / retry
       ▼
    PROBING  ----(parent selected)----►  WAITING_FOR_JOIN_RESPONSE
       ▲                                         │
       │              join fail / timeout         │ JOIN_RESPONSE OK
       └──────── node_revert_to_unregistered() ◄──┤
                                                  ▼
                                            REGISTERED
                                            (set_local_address,
                                             parent, hop)
```

- `node_enter_waiting_for_join_response(parent, parent_hop)` — stores selected parent; sets provisional child hop = parent_hop + 1  
- `node_enter_registered(assigned, parent, hop)` — pushes short address via `set_local_address()`, then `REGISTERED`  
- `node_revert_to_unregistered()` — clears parent/hop, optionally restores `UNASSIGNED_ADDRESS`, returns to `UNREGISTERED`

Those three are compiled and documented but not yet driven by JOIN message handlers.

---

## 6. What you should see on serial (RTT)

**Root**

```text
Role       : ROOT
FSM State  : REGISTERED
App Addr   : 1:254 (0x01FE) [ROOT]
Parent     : 1:254 | Hop = 0 | DATA allowed = yes
...
FSM boot state=REGISTERED addr=1:254 parent=1:254 hop=0
Root loop state=REGISTERED ...
```

**Joining**

```text
Role       : JOINING
FSM State  : UNREGISTERED
App Addr   : 255:254 (0xFFFE) [UNASSIGNED]
Parent     : 255:254 | Hop = 255 | DATA allowed = no
...
FSM boot state=UNREGISTERED ...
STATE UNREGISTERED -> PROBING
FSM state=PROBING ...
Joining loop state=PROBING ...
```

Demo requirement “Node state” in logs is covered by `STATE ... -> ...` and `log_node_state()` lines.

---

## 7. Relationship to Section 1

| Concern | Section 1 | Section 2 |
|---------|-----------|-----------|
| Who am I on the wire? | `local_address` | unchanged |
| Am I allowed to act? | — | `current_state` + `node_can_*` |
| Parent / hop | — | `parent_address`, `hop_count` |
| Root fixed identity | `ROLE_ROOT` + `ROOT_ADDRESS` | boots `REGISTERED` |
| Joiner identity | boots unassigned | boots `UNREGISTERED` → `PROBING` |

Addressing answers “what is my 16-bit ID?”. The FSM answers “what phase of join / operation am I in?”

---

## 8. Still not done (later checklist sections)

- Driving `WAITING_FOR_JOIN_RESPONSE` from parent selection  
- JOIN_REQUEST / RESPONSE / ACKNOWLEDGE **TX** and handlers calling `node_enter_*` (decode handlers exist in §3)  
- Periodic DATA only when `node_can_send_data()`  
- Automatic revert on join timeout (helper exists; timer not wired)  
- Multi-hop registered nodes answering probes (gate already allows any `REGISTERED` node)

---

## 9. Symbol map

| Item | Location |
|------|----------|
| `enum node_state` | After addressing model, before boot macros |
| `current_state`, `parent_address`, `hop_count` | Globals |
| FSM helpers | “Node FSM helpers” (after `set_local_address`) |
| `node_fsm_init()` call | `main()`, after `create_receive_context()` |
| Probe / HB gates | `handle_probe`, `handle_heartbeat`, `probe_timer_expired` |
