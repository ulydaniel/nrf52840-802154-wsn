# 10-27 Code Report: `src/code.c` multi-threading fixes

Date: Tuesday, October 27, 2026
File changed: `src/code.c` (no other source or build files were changed)

## Summary

`src/code.c` is the multi-threaded version of the firmware: the radio
callback queues received frames, an RX thread dispatches them through the
`message_handlers[]` table, and a TX thread sends PROBEs and waits for
HEARTBEATs. At the start of this session it did not compile.

This session:

1. Fixed the compile and link errors so `code.c` builds for both
   `ROLE_SENDER` and `ROLE_RECEIVER`.
2. Fixed a race in which a stale semaphore give could make a new PROBE
   look answered.
3. Bounded the send timeout so the RX thread can no longer block forever
   on a transmit.
4. Updated comments that described the old single-threaded design.

Two further design problems were analyzed, and recommendations are given
below; they are not implemented yet:

- Both roles start the TX (probe) thread.
- Two threads update the same probe state.

> **Build note:** `CMakeLists.txt` still compiles `src/main.c`, the older
> single-threaded version. To build and flash the threaded firmware, change
> the last line of `CMakeLists.txt` to:
>
> ```cmake
> target_sources(app PRIVATE src/code.c)
> ```

## Starting point

Threading pieces that already existed in `code.c` before this session:

| Piece | Purpose |
|---|---|
| `packet_receive_callback` | Runs in Zephyr's network RX thread; copies the frame and its RSSI into `receive_queue` with `K_NO_WAIT` (never blocks). |
| RX thread → `run_probe_receiver()` | Takes frames from `receive_queue`, then calls `process_frame()` → `dispatch_message()` → `message_handlers[type]`. |
| TX thread → `run_probe_sender()` | Sends a PROBE, then waits on `heartbeat_sem` for up to K seconds. |
| `heartbeat_sem` | Lets the RX thread (`handle_heartbeat`) wake the TX thread. |
| `probe_state_mutex` | Protects `probe_outstanding`, `probe_outstanding_sequence`, `probe_k_seconds`. |
| `message_handlers[]` | PROBE and HEARTBEAT implemented; JOIN / NETID entries are `NULL`. |

## Changes made

### 1. Compile and link fixes

**Handler typedef.** The typedef was missing the `*` on its second
parameter. This had already been fixed in the editor before these changes;
it is now:

```c
typedef void (*message_handler_t)(
	const struct oz_packet *packet,
	const struct received_frame *frame);
```

**Removed `run_probe_sender_old()`.** It used `probe_timer` and
`probe_timer_expired()`, which only exist in `main.c`, so `code.c` could
not compile. Its startup log lines (destination address, K, N and K max)
were moved to the top of `run_probe_sender()`.

**Thread stack name collision (link error).** Once the file compiled, the
link failed:

```text
multiple definition of `rx_stack' ... net_tc.c:53
multiple definition of `tx_stack' ... net_tc.c:49
```

`K_THREAD_STACK_DEFINE` creates a global symbol, and Zephyr's network
stack (`subsys/net/ip/net_tc.c`) already defines global `rx_stack` and
`tx_stack`. The application stacks are now `static` and use an `app_`
prefix:

```c
static K_THREAD_STACK_DEFINE(app_rx_stack, RX_STACK_SIZE);
static K_THREAD_STACK_DEFINE(app_tx_stack, TX_STACK_SIZE);

static struct k_thread app_rx_thread;
static struct k_thread app_tx_thread;
```

The two `k_thread_create()` calls in `main()` were updated to use the new
names.

**Unused-function warning.** `handle_heartbeatold()` is kept for reference
but is not in the handler table, so it is now marked `__maybe_unused`,
the same way the file already treats `send_oz_packet` and
`receive_oz_packet`.

### 2. Stale semaphore could fake a heartbeat

**The problem.** The following sequence makes a new PROBE count as
answered even though no HEARTBEAT arrived for it:

1. The TX thread's `k_sem_take(&heartbeat_sem, K)` times out.
2. Before the TX thread takes the mutex, the HEARTBEAT for that probe
   arrives. `handle_heartbeat` sees a matching outstanding probe and calls
   `k_sem_give`.
3. The TX thread records a timeout, increases K, and sends the next PROBE.
4. The next `k_sem_take` returns immediately because the semaphore was
   already given, so the new PROBE is treated as answered.

A second form of the same race: `handle_heartbeat` unlocked the mutex
*before* calling `k_sem_give`. If the RX thread was preempted between the
two calls, the give could land after the TX thread had already started
the next probe.

**The fix (two parts).**

In `run_probe_sender()`, the semaphore is reset under the same lock that
marks the new probe outstanding, so a leftover give from the previous
probe is discarded:

```c
k_mutex_lock(&probe_state_mutex, K_FOREVER);

probe_outstanding = true;
probe_outstanding_sequence = probe.sequence;
k_sem_reset(&heartbeat_sem);

k_mutex_unlock(&probe_state_mutex);
```

In `handle_heartbeat()`, `k_sem_give` moved inside the lock, before
`k_mutex_unlock`. Giving and resetting now both happen under the mutex,
so they can no longer interleave. `k_sem_give` never blocks, so calling it
while holding the mutex cannot deadlock.

### 3. RX thread could block on a send

**The problem.** `handle_probe` runs in the RX thread and replies with a
HEARTBEAT through `send_oz_packet_with_body()`. Both send functions called
`net_context_sendto(..., K_FOREVER, ...)`. If the network stack ran out of
TX buffers, the RX thread would wait forever, `receive_queue` would fill
up, and incoming frames would be dropped.

**The fix.** A new constant bounds the wait:

```c
#define TX_SEND_TIMEOUT_MS       100
```

Both `send_oz_packet()` and `send_oz_packet_with_body()` now pass
`K_MSEC(TX_SEND_TIMEOUT_MS)`. On timeout the send returns an error, which
the caller already logs, and the RX thread continues draining the queue.

### 4. Comment updates

- The probe-state comment previously said the state was "owned by the
  main thread ... so no locking is needed". It now says the state is
  written by the TX thread and the RX thread and that every access holds
  `probe_state_mutex`.
- The message-handler section said "Handlers run in the main thread". It
  now says the RX thread.

## Verification

`code.c` was test-built with `west build` (NCS v3.4.1, board
`OzturkLab/nrf52840`) in a temporary copy of the project whose
`CMakeLists.txt` pointed at `code.c`, once per role:

| Role | Result | FLASH | RAM |
|---|---|---|---|
| `ROLE_RECEIVER` | Builds and links | 153,784 B (14.67 %) | 48,336 B (18.44 %) |
| `ROLE_SENDER` | Builds and links | 153,784 B (14.67 %) | 48,336 B (18.44 %) |

Two harmless warnings were already present and remain:

- `receive_oz_packet`: `const` qualifier discarded when passing an address
  to `zsock_recvfrom`. The function is not used.
- `packet_receive_callback`: `source->addr != NULL` is always true, because
  `addr` is an array in this Zephyr version.

The fixes have not yet been tested on hardware.

## Recommendations (not yet implemented)

### A. Both roles start the TX (probe) thread

**The problem.** The role is chosen by one compile-time switch,
`NODE_ROLE`, but `main()` starts both threads regardless. On a receiver,
the TX thread sends PROBEs to `RECEIVER_ADDR`, which is its own address.

**Recommendation.** Since a node should eventually be both a sender and a
receiver, treat probing and responding as separate abilities instead of
one either-or role:

1. **Always start the RX thread.** Every node needs it to answer PROBEs
   with HEARTBEATs and to receive HEARTBEATs.
2. **Start the probe thread only if the node probes.** Replace
   `NODE_ROLE` with a separate setting such as `#define NODE_PROBES 1`, or
   better, a Kconfig option (`CONFIG_APP_PROBER=y`) set per board, so the
   source does not need editing for each board. A node with probing turned
   on is then both a sender and a receiver.
3. **Make addresses variables, not role macros.** `LOCAL_SHORT_ADDR` could
   come from Kconfig or be derived from the GUID, and the probe target
   should be a `probe_peer_addr` variable instead of `RECEIVER_ADDR`. This
   prepares for probing several peers, and for JOIN assigning addresses.
4. Optionally, define the probe thread with
   `K_THREAD_DEFINE(..., K_FOREVER)` so it starts suspended, and call
   `k_thread_start()` only when probing is turned on.

### B. Two threads update the same probe state

**The problem.** `probe_outstanding`, `probe_outstanding_sequence` and
`probe_k_seconds` are written by two threads:

- **RX thread**, in `handle_heartbeat`: clears `probe_outstanding` and
  resets K.
- **TX thread**, in `run_probe_sender`: marks the probe outstanding, then
  on success clears it and resets K again, or on timeout clears it and
  increases K.

Consequences:

- The same decision ("this probe is answered, reset K") is made in two
  places, which is how the timing races above appeared.
- Every access needs the mutex, and some reads do not take it:
  `build_packet(..., probe_k_seconds, ...)` and
  `K_SECONDS(probe_k_seconds)`.
- Each new message type (JOIN, NETID) would add more shared state and more
  locking.

**Recommendation: a single owner.** The TX thread owns all probe state; the
RX handler only reports what it received. Replace `heartbeat_sem` with a
small message queue:

```c
struct probe_event {
    uint16_t sequence;
    uint16_t source_id;
    int16_t rssi_dbm;
};

K_MSGQ_DEFINE(probe_event_queue, sizeof(struct probe_event), 2, 4);

/* RX thread, in handle_heartbeat(): validate the body, log, then */
struct probe_event ev = {
    .sequence = packet->sequence,
    .source_id = packet->source_id,
    .rssi_dbm = packet->value,
};
(void)k_msgq_put(&probe_event_queue, &ev, K_NO_WAIT);
```

```c
/* TX thread: all probe state is local; no mutex */
static void run_probe_sender(void)
{
    unsigned int k_seconds = PROBE_K_INITIAL_SECONDS;
    struct oz_packet probe;
    struct probe_event ev;
    int64_t deadline;
    bool answered;

    while (1) {
        build_packet(&probe, MSG_TYPE_PROBE, RECEIVER_ADDR,
                     (int16_t)k_seconds, tx_sequence++);

        k_msgq_purge(&probe_event_queue);
        (void)send_oz_packet(&probe, RECEIVER_ADDR);

        deadline = k_uptime_get() + (int64_t)k_seconds * 1000;
        answered = false;

        /* Wait for the matching sequence, discarding stale replies */
        while (!answered) {
            int64_t left = deadline - k_uptime_get();

            if ((left <= 0) ||
                (k_msgq_get(&probe_event_queue, &ev, K_MSEC(left)) != 0)) {
                break;
            }
            answered = (ev.sequence == probe.sequence);
        }

        if (answered) {
            k_seconds = PROBE_K_INITIAL_SECONDS;
        } else {
            k_seconds = MIN(k_seconds + PROBE_N_INCREMENT_SECONDS,
                            PROBE_K_MAX_SECONDS);
        }
    }
}
```

What this gains:

- `probe_outstanding`, `probe_outstanding_sequence`, `probe_state_mutex`
  and `heartbeat_sem` all go away.
- The stale-heartbeat race cannot happen at all, because the sequence check
  is done by the thread that owns the probe.
- A late reply for an old probe is dropped without ending the current wait
  early.
- The same pattern carries over to the JOIN and NETID handshakes: handlers
  put events on a queue, and one protocol thread owns `node_state` and the
  retry timers.

## Longer-term design direction

The overall threading model recommended for adding the JOIN and NETID
handlers:

```text
radio ISR / net RX thread
   packet_receive_callback  ── k_msgq_put(K_NO_WAIT) ──► receive_queue
                                                         │
RX/dispatch thread (prio 5)                              ▼
   process_frame → message_handlers[type](pkt, frame)
        │ handlers never block; they only:
        ├─ k_msgq_put(tx_queue, reply)      (e.g. PROBE → HEARTBEAT)
        └─ k_msgq_put(event_queue, event)   (e.g. HEARTBEAT ok, JOIN_RESPONSE)
                                                         │
Protocol/state thread (prio 6)                           ▼
   waits on event_queue with timeout = next deadline
   owns node_state, probe K, join/NETID retries
        └─ k_msgq_put(tx_queue, request)
                                                         │
TX thread (prio 6/7)                                     ▼
   only caller of net_context_sendto(), bounded timeout
```

Rules that keep this deadlock-free:

- The radio callback never blocks (already true).
- Handlers never wait for another message; they queue a reply or post an
  event and return.
- Each piece of state has exactly one owning thread.
- If any mutex remains: never hold it while sending, logging or waiting;
  use one lock rather than nesting; give every wait a timeout.
- The RX thread runs at a higher priority than the TX and protocol threads
  so incoming frames are drained first.

Suggested debug settings for `prj.conf` while developing the threads:

```text
CONFIG_THREAD_NAME=y
CONFIG_THREAD_ANALYZER=y
CONFIG_THREAD_ANALYZER_AUTO=y
CONFIG_INIT_STACKS=y
CONFIG_ASSERT=y
```

## Suggested next steps

1. Point `CMakeLists.txt` at `src/code.c`, or merge the threaded code into
   `main.c`.
2. Flash both boards and confirm PROBE / HEARTBEAT exchange and K back-off
   on hardware.
3. Implement recommendation B (single owner for probe state).
4. Implement recommendation A (separate probing and responding abilities).
5. Add the JOIN and NETID handlers on top of the event-queue design.
