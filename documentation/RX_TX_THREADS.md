# RX / TX Application Threads

**File:** `src/main.c` (default build via `CMakeLists.txt`)  
**Related:** `prj.conf` (`CONFIG_THREAD_NAME`), earlier experiment in `src/code.c` / `documentation/10-27-code-report.md`

This document describes the Zephyr-based split of receive processing and radio transmit into two application threads so a node can keep draining RX work while TX is in progress on the air stack.

---

## Why

Previously, a single main-thread loop both:

1. Dequeued RX frames and ran the FSM / message handlers  
2. Called `net_context_sendto()` inline (with `K_FOREVER`)

While the radio callback could still enqueue frames during a send, **application TX blocked application RX processing** until the send returned. Separating threads fixes that without changing the assignment protocol.

Threads are **not** required by the PDF checklist; this is an architecture improvement for concurrent RX handling vs TX.

---

## Architecture

```text
radio / Zephyr net RX context
   packet_receive_callback
        │  copy frame + RSSI
        │  k_msgq_put(receive_queue, K_NO_WAIT)
        ▼
receive_queue
        │
RX/protocol thread  (prio 5, name "app_rx")
   run_probe_sender() or run_probe_receiver()
        │  k_msgq_get(receive_queue)
        │  process_frame → message_handlers[]
        │  timer polls (probe / selection / join / data)
        │  send_oz_packet*() → serialize → tx_queue_submit()
        ▼
tx_queue   (struct tx_request: dest + length + payload bytes)
        │
TX thread  (prio 6, name "app_tx")
   k_msgq_get(tx_queue, K_FOREVER)
   radio_send_buffer() → net_context_sendto(..., K_MSEC(100))
```

### Ownership rules

| Resource | Owner | Notes |
|----------|--------|--------|
| `receive_queue` | Producer: radio callback; consumer: RX thread | Callback never blocks |
| FSM / probe / join / DATA timers / `link_stats` | RX/protocol thread only | No mutex required for protocol state |
| `tx_queue` | Producer: RX thread via `send_oz_packet*`; consumer: TX thread | Payload fully copied into `struct tx_request` |
| `net_context_sendto` / `receive_context` TX | TX thread only | Bounded timeout |

IEEE 802.15.4 remains **half-duplex on the air**: the radio still cannot transmit and receive RF in the same instant. What this design enables is **parallel software paths** — handlers and the FSM keep running while a send waits on the network stack.

---

## Code changes (`src/main.c`)

### New configuration

| Symbol | Default | Purpose |
|--------|---------|---------|
| `RX_QUEUE_DEPTH` | 8 | Deeper RX backlog while TX is busy |
| `TX_QUEUE_DEPTH` | 8 | Pending serialized TX jobs |
| `APP_RX_STACK_SIZE` / `APP_TX_STACK_SIZE` | 2048 | Thread stacks |
| `APP_RX_PRIORITY` / `APP_TX_PRIORITY` | 5 / 6 | RX drains first |
| `TX_SEND_TIMEOUT_MS` | 100 | Avoid forever-block on send |

### New types / objects

- `struct tx_request` — MAC destination + length + `data[OZ_PACKET_SIZE + OZ_MAX_BODY_SIZE]`
- `K_MSGQ_DEFINE(tx_queue, …)`
- `app_rx_stack` / `app_tx_stack` / `app_rx_thread` / `app_tx_thread`  
  (names are `app_`-prefixed so they do not collide with Zephyr `net_tc.c` symbols)

### Send path refactor

| Function | Role |
|----------|------|
| `send_oz_packet()` / `send_oz_packet_with_body()` | Serialize and **enqueue** (RX/protocol thread) |
| `tx_queue_submit()` | `k_msgq_put(tx_queue, K_NO_WAIT)`; `-ENOBUFS` if full |
| `radio_send_buffer()` | Actual `net_context_sendto` with `K_MSEC(TX_SEND_TIMEOUT_MS)` |

Call sites (PROBE, HEARTBEAT, JOIN_*, DATA, forward) still call `send_oz_packet*`; they no longer block on the radio.

### Threads and `main()`

- `rx_thread_entry` → role loop (`run_probe_sender` / `run_probe_receiver`)
- `tx_thread_entry` → forever drain `tx_queue` → `radio_send_buffer`
- `main()` configures radio + FSM, starts both threads with `k_thread_create` / `k_thread_name_set`, then returns (threads keep running)

### `prj.conf`

```text
CONFIG_THREAD_NAME=y
```

Makes RTT / debugger thread lists show `app_rx` and `app_tx`.

---

## Behaviour notes

1. **Submit success ≠ air success.** `send_oz_packet` returning 0 means the job was queued. Radio failures are logged in the TX thread. Link counters that used to treat submit as “sent” still increment on queue accept (same practical meaning as before for queue-full drops).
2. **Queue full drops.** RX: `RX queue full; packet dropped`. TX: `TX queue full; packet dropped`. Increase depths if demos show drops under load.
3. **No protocol mutex.** Only the RX thread mutates FSM/probe/join/DATA state. The TX thread only sees a private `tx_request` copy.
4. **Contrast with `code.c`.** That file used a probe TX thread waiting on a semaphore + mutex (race-prone). This design keeps protocol on RX and uses a pure TX worker queue instead.

---

## How to verify on hardware

1. Build/flash one `ROLE_ROOT` and one or more `ROLE_JOINING` boards as usual.  
2. Confirm boot log: `RX/protocol thread started`, `TX thread started`, `Threads started (RX prio 5, TX prio 6)`.  
3. Confirm PROBE ↔ HEARTBEAT, join, and 1 Hz DATA still work.  
4. Optional: enable in `prj.conf` while debugging stacks:

```text
CONFIG_THREAD_ANALYZER=y
CONFIG_THREAD_ANALYZER_AUTO=y
CONFIG_INIT_STACKS=y
```

---

## Future (optional)

- A third **protocol** thread + event queue (as sketched in `10-27-code-report.md`) if timer/FSM work must be isolated from heavy RX logging.
- Atomic / TX-thread updates for `heartbeats_tx_failed` on radio timeout (today only queue-full failures bump some fail counters from the submit path).
- Do **not** point `CMakeLists.txt` at `code.c` unless intentionally reviving that older experiment; `main.c` is the threaded production path now.
