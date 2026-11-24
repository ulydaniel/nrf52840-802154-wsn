# 11-23 Code Report: join discovery / registration RF fixes

Date: Monday, November 23, 2026  
File changed: `src/main.c`  
Related: hardware RTT logs (root + joining) showing 100% probe miss and `JOIN_ACK` timeouts

## Summary

Two boards (one `ROLE_ROOT`, one `ROLE_JOINING`) had good RSSI (~−40 dBm) but never completed registration:

1. The joiner stayed in `PROBING` with `HB accepted = 0` / `Probe missed = 100%`.
2. When the root did see join traffic, it allocated pending `1:n`, sent `JOIN_RESPONSE`, then logged `JOIN_ACK timeout … releasing` with `active=0`.

Root cause analysis pointed at IEEE 802.15.4 short-address semantics for `0xFFFE` and Zephyr `-EBUSY` on late short-address updates. This session implements three fixes in `src/main.c` so discovery and registration can complete on the air.

## Symptoms (from RTT)

| Side | Observation |
|------|-------------|
| Joining | `PROBE` TX OK; only other `PROBE`s RX; `PROBE … ignored (state=PROBING; only REGISTERED replies)`; never `HEARTBEAT RX` |
| Root | `PROBE` RX → `HEARTBEAT TX` with `MAC TX -> 0xFFFE`; sometimes `JOIN_REQUEST` / `JOIN_RESPONSE`; then `JOIN_ACK timeout` and address release |

Happy path that was failing:

```text
PROBE → HEARTBEAT → JOIN_REQUEST → JOIN_RESPONSE → JOIN_ACK → REGISTERED
```

## Root causes

### 1. HEARTBEAT MAC unicast to `0xFFFE`

`handle_probe()` sent the HEARTBEAT with MAC destination = OZ `source_id` (`0xFFFE`).

In IEEE 802.15.4, **`0xFFFE` means “no short address.”** A device whose short address is still `0xFFFE` typically **does not accept unicasts addressed to `0xFFFE`**. It does accept broadcasts (`0xFFFF`).

That matches the joiner log: broadcast PROBEs arrive; HEARTBEATs never do → parent selection never starts.

### 2. Unreliable MAC destination for `JOIN_RESPONSE`

Root stored `frame->mac_source` and used it as the `JOIN_RESPONSE` MAC destination. On this stack the LL source often shows up as `0xFFFF` in logs (short source not recovered). Sending to a real `0xFFFE` unicast would hit the same RX-filter problem as HEARTBEAT. Broadcast is the safe pre-registration path.

### 3. `-EBUSY` when applying the assigned short address

After PAN ID + short address are programmed at boot, Zephyr may reject a later `NET_REQUEST_IEEE802154_SET_SHORT_ADDR` with `-EBUSY`. `node_enter_registered()` then aborted without sending `JOIN_ACKNOWLEDGE`, so the root released the pending address after `JOIN_ACK_PENDING_TIMEOUT_SECONDS` (10 s).

## Changes made

### 1. Broadcast HEARTBEAT at the MAC layer

**Where:** `handle_probe()` → `send_oz_packet_with_body(...)`

| Layer | Before | After |
|-------|--------|-------|
| OZ `destination_id` | Probe `source_id` (`0xFFFE`) | Unchanged (app filter still targets unassigned joiners) |
| MAC destination | `packet->source_id` (`0xFFFE`) | `BROADCAST_ADDRESS` (`0xFFFF`) |

Expected RTT after fix: root shows `MAC TX -> 0xFFFF` on HEARTBEAT; joiner shows `HEARTBEAT RX` / `accepted` and opens the parent-selection window.

### 2. `mac_dest_for_unassigned_peer()`

New helper used by the root when sending `JOIN_RESPONSE`:

- If recovered MAC source is broadcast, unassigned (`0xFFFE`), or zero → use `BROADCAST_ADDRESS`.
- Otherwise keep the recovered short address (future-proof if LL source is populated correctly).

Applied on both the fresh-allocation path and the duplicate-nonce retransmit path in `handle_join_request()`.

### 3. `SET_SHORT_ADDR` on join — disassociate then reassociate

**Where:** `ieee802154_apply_short_address()`

Hardware follow-up (same day) showed the first `-EBUSY` workaround was wrong: cycling `net_if_down()` / `net_if_up()` and retrying `SET_SHORT_ADDR(1:n)` still returned **-16 (`-EBUSY`)**. Zephyr only allows one parameter change while associated: set short address to `IEEE802154_SHORT_ADDRESS_NOT_ASSOCIATED` (`0xFFFF`), which clears **both** short address and PAN ID.

Correct sequence on `-EBUSY`:

1. `SET_SHORT_ADDR(0xFFFF)` — disassociate.
2. `SET_PAN_ID(0x1234)` — restore PAN.
3. `SET_SHORT_ADDR(assigned)` — associate with `1:n`.

Also set `CONFIG_NET_IPV6=n` / `CONFIG_NET_IPV4=n` in `prj.conf` so reassociation does not inject IPv6 ND frames (`0x60…`) into the AF_PACKET socket.

This unblocks `set_local_address()` during `node_enter_registered()` so the joiner can send `JOIN_ACKNOWLEDGE` and the root can mark the row **ACTIVE**.

## Files

| Path | Action |
|------|--------|
| `src/main.c` | HEARTBEAT MAC broadcast; MAC dest helper; disassociate/`SET_PAN_ID`/retry on `-EBUSY` |
| `prj.conf` | `CONFIG_NET_IPV6=n`, `CONFIG_NET_IPV4=n` |
| `documentation/11-23-code-report.md` | This report |

No CMake / host-sim changes.

## How to verify on hardware

1. Flash one board with `#define NODE_ROLE ROLE_ROOT`, another with `ROLE_JOINING` (rebuild between flashes).
2. Confirm boot banners: root `1:254 (0x01FE)`, joiner `255:254 (0xFFFE)`, same channel / PAN ID.
3. On joiner, expect within a few probe periods:
   - `HEARTBEAT … accepted`
   - `Parent selected …`
   - `JOIN_REQUEST TX` → `JOIN_RESPONSE RX`
   - Possibly `SET_SHORT_ADDR … busy; disassociate then retry`, then `Short Addr = 0x01nn`
   - `JOIN_ACKNOWLEDGE TX` / `Node REGISTERED`
4. On root, expect:
   - `HEARTBEAT TX` with `MAC TX -> 0xFFFF`
   - `Allocated pending 1:n …`
   - `JOIN_ACKNOWLEDGE RX` → `Address 1:n marked ACTIVE`
   - No repeating `JOIN_ACK timeout` / release loop
   - No flood of `Invalid packet ignored: bad signature 0x60` (IPv6 disabled)

## Out of scope / follow-ups

- Improving LL short-source recovery so `mac_source` is not stuck at `0xFFFF` (helper already falls back to broadcast).
- Updating `SECTION1_ADDRESSING.md` / `SECTION4_PROTOCOL_FLOW.md` prose to mention MAC-broadcast HEARTBEAT (behavior change is in `main.c` + this report).
- Host `milestone_sim.py` already abstracts MAC delivery; no sim change required for this RF filter issue.
