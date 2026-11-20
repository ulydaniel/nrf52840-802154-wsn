# Section 1 — Addressing Model

**Assignment reference:** COMPE 662 IEEE 802.15.4 Data Collection Tree, §4 Network Addressing  
**Code:** `src/main.c`  
**Checklist:** `documentation/ASSIGNMENT_CHECKLIST.md` §1

This document explains how application addresses are defined, stored, and applied to the IEEE 802.15.4 radio in the current project.

---

## 1. Why this section exists

The original demo used two fixed short addresses:

| Old role | Address |
|----------|---------|
| Sender | `0x0001` |
| Receiver | `0x0003` |

That cannot satisfy the assignment: only the **root** may be hard-coded; every other node must receive an address through the join protocol.

Section 1 replaces that model with the assignment’s **network:node** addressing, a runtime `local_address`, and helpers the join / allocation code will reuse later.

---

## 2. Address format

An application address is a 16-bit value:

```text
 bits 15 ............ 8 | 7 ................. 0
 +----------------------+----------------------+
 |     network ID       |       node ID        |
 +----------------------+----------------------+
```

Built and split with:

```c
#define MAKE_ADDRESS(network, node) \
    ((uint16_t)((((uint16_t)(network) & 0xFFU) << 8) | \
                ((uint16_t)(node) & 0xFFU)))

#define ADDRESS_NETWORK(addr)  ((uint8_t)(((uint16_t)(addr) >> 8) & 0xFFU))
#define ADDRESS_NODE(addr)     ((uint8_t)((uint16_t)(addr) & 0xFFU))
```

Logs print both forms, for example `1:254 (0x01FE)`.

---

## 3. Well-known addresses

| Symbolic name | Value | Meaning |
|---------------|------:|---------|
| `ROOT_ADDRESS` | `MAKE_ADDRESS(1, 254)` → **1:254** / `0x01FE` | Fixed root; only hard-coded node address |
| `ADDRESS_FIRST_ASSIGNABLE` .. `ADDRESS_LAST_ASSIGNABLE` | **1:1** .. **1:253** | Pool the root may allocate |
| `RESERVED_ADDRESS` | **1:0** / `0x0100` | Reserved; never assigned |
| `UNASSIGNED_ADDRESS` | **255:254** / `0xFFFE` | Node has no network address yet |
| `BROADCAST_ADDRESS` | **255:255** / `0xFFFF` | Application-layer broadcast |

`NETWORK_ID_ROOT` is `1`. This milestone implements the **root network only**.

---

## 4. Board role vs address

Flash-time switch (only line that must change per board):

```c
#define NODE_ROLE         ROLE_ROOT      /* or ROLE_JOINING */
```

| `NODE_ROLE` | Boot address (`BOOT_ADDRESS`) | Meaning |
|-------------|-------------------------------|---------|
| `ROLE_ROOT` | `ROOT_ADDRESS` (1:254) | Root; discovery assumed complete |
| `ROLE_JOINING` | `UNASSIGNED_ADDRESS` (255:254) | Waits for join to get 1:1..1:253 |

No joining-node address is compiled in. The pool exists as constants and predicates only; allocation is a later milestone.

---

## 5. Runtime state: `local_address`

```c
static uint16_t local_address = BOOT_ADDRESS;
```

This is the single source of truth for “who am I?” at the application layer.

| Consumer | How it uses `local_address` |
|----------|-----------------------------|
| `build_packet()` | Sets OZ header `source_id` |
| `process_frame()` | Accepts frames via `address_is_for_me()` |
| `configure_ieee802154()` | Programs the MAC short address at boot |
| Future join ACK path | Will call `set_local_address(assigned)` |

### Accepting frames

```c
static bool address_is_for_me(uint16_t destination)
{
    return (destination == local_address) ||
           address_is_broadcast(destination);
}
```

Unicast to this node **or** broadcast (required for discovery probes) is accepted. Everything else is logged and dropped.

---

## 6. Helper functions

| Function | Purpose |
|----------|---------|
| `address_is_broadcast()` | `addr == 0xFFFF` |
| `address_is_unassigned()` | `addr == 0xFFFE` |
| `address_is_root()` | `addr == ROOT_ADDRESS` |
| `address_is_reserved()` | Root, unassigned, broadcast, or **1:0** — not for normal allocation |
| `address_is_assignable()` | True for **1:1 .. 1:253** only |
| `address_is_for_me()` | RX filter (local or broadcast) |
| `ieee802154_apply_short_address()` | `NET_REQUEST_IEEE802154_SET_SHORT_ADDR` |
| `set_local_address()` | Update `local_address` **and** MAC short address |

`set_local_address()` is ready for Milestone 5 (registration). Until join is implemented it is unused at run time (`__maybe_unused`).

Boot self-check in `main()` verifies that **1:1** and **1:253** are assignable and not reserved, so the pool helpers cannot silently disagree with the macros.

---

## 7. How this hooks into the radio

At boot, `configure_ieee802154()` still does:

1. Channel  
2. TX power  
3. PAN ID  
4. **Short address ← `local_address`** (via `ieee802154_apply_short_address`)  
5. `net_if_up()`

So:

- Root radio short address = `0x01FE`  
- Joining radio short address = `0xFFFE`  

Application OZ `source_id` / `destination_id` use the same 16-bit values. MAC destination for TX is chosen by the sender (`send_oz_packet`); discovery probes use `BROADCAST_ADDRESS` so every listener can hear them at L2.

**Later note:** once PAN ID and short address are both set, Zephyr may treat the node as associated and return `-EBUSY` on further parameter changes. When join calls `set_local_address()`, we may need a brief interface down/reconfigure. That is documented here so Milestone 5 is not surprised.

---

## 8. What changed for the transitional probe path

Section 1 is addressing, not the full join FSM. The old fixed-peer poll was remapped so identities match the assignment:

| Board | Role | Behavior now |
|-------|------|----------------|
| Root | `ROLE_ROOT` | Address **1:254**; listens; answers PROBE with HEARTBEAT |
| Joining | `ROLE_JOINING` | Address **255:254**; broadcasts PROBE; accepts HEARTBEAT |

Probe TX uses:

- App source = `local_address` (unassigned on joining boards)  
- App + MAC destination = `BROADCAST_ADDRESS`

Full parent selection / JOIN_* / dynamic allocation are **not** part of this section. Common header parent / assigned / hop fields are documented in `SECTION3_MESSAGE_TYPES.md`.

---

## 9. How to flash two boards

1. Set `#define NODE_ROLE ROLE_ROOT`, build, flash board A.  
2. Set `#define NODE_ROLE ROLE_JOINING`, build, flash board B.  
3. Same `IEEE802154_CHANNEL` and `IEEE802154_PAN_ID` on both.

Expected boot lines (excerpt):

**Root**

```text
Role       : ROOT
App Addr   : 1:254 (0x01FE) [ROOT]
Assignable : 1:1 .. 1:253
```

**Joining**

```text
Role       : JOINING
App Addr   : 255:254 (0xFFFE) [UNASSIGNED]
Assignable : 1:1 .. 1:253
```

Packet logs now show `Source` / `Destination` as `network:node (0xXXXX)`.

---

## 10. What Section 1 deliberately does *not* do

| Item | Status |
|------|--------|
| Root address-allocation table (`address_record`) | Later (allocation milestone) |
| Assigning 1:1, 1:2, … at join time | Later (`set_local_address`) |
| FSM `UNREGISTERED` → … → `REGISTERED` | Section 2 |
| Parent / hop fields in the packet | Later packet-format work |
| DATA forwarding | Later |

Section 1 only establishes the address **vocabulary**, boot roles, runtime local address, RX filter rules, and the MAC update hook.

---

## 11. File / symbol map

| Symbol / area | Location in `main.c` |
|---------------|----------------------|
| `MAKE_ADDRESS`, well-known addresses | “Application addressing model” |
| `BOOT_ADDRESS`, `local_address` | After role → boot address |
| Helpers + `set_local_address` | “Addressing helpers” |
| MAC apply at boot | `configure_ieee802154()` |
| App source on TX | `build_packet()` |
| App dest filter on RX | `process_frame()` → `address_is_for_me()` |
| Banner / self-check | `main()` |

---

## 12. Checklist alignment

After this implementation, checklist §1 items for macros, root/unassigned/broadcast/reserved/assignable range, root boot address, removal of fixed `0x0001`/`0x0003` peers, and the short-address update hook are **done**. Dynamic issuance of 1:1..1:253 still waits on the join protocol (correctly — that must not be compile-time).
