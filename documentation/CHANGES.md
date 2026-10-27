# Probe / Heartbeat Protocol: Change Log

This document records every change made to the IEEE 802.15.4 send/receive
project to add the probe timer, the heartbeat reply, and the message-handler
function table. **No original code was deleted.** Code that is no longer used
was commented out or disabled with `#if 0` and left in place for future use.

Files touched:

| File | Status |
|------|--------|
| `src/main.c` | Modified (old code commented out, new code added) |
| `prj.conf` | Modified (HWINFO, ADC, larger RTT/log buffers) |
| `CMakeLists.txt` | Modified (adds `battery_adc.overlay` via `EXTRA_DTC_OVERLAY_FILE`) |
| `battery_adc.overlay` | **New** (enables the SAADC for battery measurement) |
| `CHANGES.md` | **New** (this file) |

---

## 1. Building and flashing

1. Set the role at the top of `src/main.c`:

   ```c
   #define NODE_ROLE         ROLE_SENDER     /* board 1 */
   #define NODE_ROLE         ROLE_RECEIVER   /* board 2 */
   ```

2. Build normally (incremental or pristine both work).

3. Flash the sender build to one board and the receiver build to the other.

Both roles compile with NCS v3.4.1 for `OzturkLab/nrf52840`, in both a
pristine build and an incremental build of the existing `OzturkLab` build
folder. They have **not** been tested on hardware yet.

### Why the overlay is added in `CMakeLists.txt`

The ADC overlay was first added as `app.overlay`, which Zephyr normally finds
automatically. Building the existing `OzturkLab` folder then failed with:

```
error: '__device_dts_ord_31' undeclared        (DEVICE_DT_GET(DT_NODELABEL(adc)))
error: 'struct adc_channel_cfg' has no member named 'input_positive'
```

Both errors meant the ADC was still disabled in the devicetree: the overlay
had not been applied, so no ADC device existed. Without an enabled SAADC, the
driver's `CONFIG_ADC_CONFIGURABLE_INPUTS` is not selected, which removes the
`input_positive` field. The overlay was missed because the build folder was
first configured before any overlay existed, so `DTC_OVERLAY_FILE` was cached
as an empty string. Zephyr only auto-detects `app.overlay` while that
variable is undefined (`zephyr/cmake/modules/configuration_files.cmake`).

Fix: the overlay was renamed to `battery_adc.overlay` and is added explicitly
in `CMakeLists.txt`:

```cmake
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/battery_adc.overlay)
```

`EXTRA_DTC_OVERLAY_FILE` is merged from every source, local CMake variables
included, so a cached value doesn't hide it. The new name also keeps a
pristine build from auto-detecting the file and applying it twice.

---

## 2. Protocol overview

```
Sender 0x0001                                  Receiver 0x0003
     |                                                |
     |  timer K expires                               |
     |------------------- PROBE (seq=n) ------------->|
     |                                                |  handle_probe()
     |<---------------- HEARTBEAT (seq=n) ------------|  reads GUID, battery
     |  handle_heartbeat()                            |
     |  K reset to initial                            |
     |                                                |
     |  timer K expires, no HEARTBEAT for seq=n+1     |
     |  K = K + N                                     |
     |------------------- PROBE (seq=n+2) ----------->|
```

### 2.1 Message types and the function-pointer table

Received packets are decoded, then dispatched through
`message_handlers[7]`, indexed by `message_type - MSG_TYPE_FIRST`.

| Type | Value | Handler | Status |
|------|-------|---------|--------|
| `MSG_TYPE_PROBE` | 1 | `handle_probe` | Implemented |
| `MSG_TYPE_HEARTBEAT` | 2 | `handle_heartbeat` | Implemented |
| `MSG_TYPE_JOIN_REQUEST` | 3 | `NULL` | Reserved |
| `MSG_TYPE_JOIN_RESPONSE` | 4 | `NULL` | Reserved |
| `MSG_TYPE_JOIN_ACK` | 5 | `NULL` | Reserved |
| `MSG_TYPE_NETID_REQUEST` | 6 | `NULL` | Reserved |
| `MSG_TYPE_NETID_RESPONSE` | 7 | `NULL` | Reserved |

A `NULL` entry logs `No handler implemented for <NAME> (type N)`. Types
outside 1–7 log `Unknown message type N ignored`.

Note: the old `MESSAGE_TYPE_DATA` is also `1`. It is only used by the
disabled legacy sender, so it does not conflict at runtime.

### 2.2 Probe timer (sender)

| Define | Default | Meaning |
|--------|---------|---------|
| `PROBE_K_INITIAL_SECONDS` | 2 | Initial timer K |
| `PROBE_N_INCREMENT_SECONDS` | 2 | N, added to K after a missed heartbeat |
| `PROBE_K_MAX_SECONDS` | 30 | Upper limit for K |

Behaviour:

- The first PROBE is sent when timer K first expires (K seconds after boot).
- Each time the timer expires, the sender first checks whether the previous
  PROBE is still unanswered. If it is, K = min(K + N, K max). Then the sender
  sends a new PROBE and restarts the timer with K.
- The receiver therefore has K seconds to answer each PROBE.
- A HEARTBEAT whose header sequence matches the outstanding PROBE resets K to
  `PROBE_K_INITIAL_SECONDS`.
- A HEARTBEAT with any other sequence (a late reply to an older probe) is
  logged as stale and ignored.
- A PROBE that failed to transmit still counts as unanswered.

These behaviours (reset on success, the 30 s cap, the K-second reply window)
were design choices. The assignment text did not specify them.

### 2.3 Packet formats

The original 12-byte OZ header is **unchanged**. The PROBE packet is exactly
the header. The HEARTBEAT packet is the header followed by a 13-byte body
(25 bytes total).

**OZ header (all messages, bytes 0–11), little endian:**

| Bytes | Field | PROBE use | HEARTBEAT use |
|-------|-------|-----------|---------------|
| 0–1 | Signature `"OZ"` | | |
| 2 | Version (`1`) | | |
| 3 | Message type | `1` | `2` |
| 4–5 | Source short address | sender | receiver |
| 6–7 | Destination short address | receiver | sender |
| 8–9 | Value (int16) | sender's current K (s) | **RSSI** of the PROBE at the receiver (dBm) |
| 10–11 | Sequence | probe sequence | **echoes** the PROBE sequence |

**HEARTBEAT body (frame bytes 12–24):**

| Frame bytes | Body bytes | Field | Source |
|-------------|------------|-------|--------|
| 12–19 | 0–7 | **GUID**, 64-bit | `hwinfo_get_device_id()` (nRF52840 FICR DEVICEID) |
| 20–21 | 8–9 | **Battery voltage**, mV (uint16 LE) | SAADC, internal VDD input |
| 22 | 10 | **Battery level**, % (`0xFF` = unknown) | Linear map, see below |
| 23–24 | 11–12 | **Heartbeat sequence #** (uint16 LE) | Receiver's own counter, +1 per heartbeat |

There are two sequence numbers:

- The **header sequence** echoes the PROBE, so the sender can match the reply
  to the outstanding probe.
- The **heartbeat sequence #** in the body counts heartbeats sent by the
  replying node. Gaps show lost heartbeats.

### 2.4 Battery measurement

| Define | Default | Meaning |
|--------|---------|---------|
| `BATTERY_FULL_MV` | 3000 | Voltage reported as 100 % |
| `BATTERY_EMPTY_MV` | 2000 | Voltage reported as 0 % |
| `BATTERY_PERCENT_UNKNOWN` | `0xFF` | Sent if the ADC is unavailable or a read fails |
| `BATTERY_ADC_CHANNEL` | 0 | SAADC channel used |
| `BATTERY_ADC_RESOLUTION` | 12 | Bits |
| `BATTERY_ADC_GAIN` | `ADC_GAIN_1_6` | With the 0.6 V internal reference, full scale = 3.6 V |

**Limitation:** the board's devicetree has no battery voltage divider, so
the code measures the nRF52840 **VDD** rail. This equals the battery voltage
only when the battery feeds VDD directly (coin cell, 2×AA). If the battery
goes through a regulator, VDD stays at the regulated voltage. In that case,
add a divider on an AIN pin and change `input_positive` in
`init_battery_adc()`. Adjust `BATTERY_FULL_MV` and `BATTERY_EMPTY_MV` for
the battery chemistry.

If the ADC fails to initialise, the node still runs and heartbeats report
battery as unknown.

---

## 3. Architecture change: one radio endpoint per node

**Before:** the sender only transmitted, through an AF_PACKET socket
(`packet_socket`). The receiver only received, through a packet-aware
`net_context` (`receive_context`) whose callback reads RSSI.

**Now:** both nodes must transmit **and** receive. Both roles use
`receive_context` for RX (callback → `receive_queue`) **and** for TX
(`net_context_sendto()`).

Why not keep the socket for TX: Zephyr gives every bound AF_PACKET endpoint
its own clone of each received frame (`conn_raw_socket_deliver()` in
`subsys/net/ip/connection.c`). If the sender had both the socket and the
context open, heartbeat clones would collect in the never-read socket queue
and slowly use up the network RX buffers. `net_context_sendto()` is the same
call the packet socket's `sendto()` makes internally
(`zpacket_sendto_ctx()` in `subsys/net/lib/sockets/sockets_packet.c`), so the
TX path is unchanged below the application.

---

## 4. `src/main.c`: detailed changes

### 4.1 Original code commented out / disabled (kept in file)

| Location | Original code | How it is disabled | Replaced by |
|----------|---------------|--------------------|-------------|
| Globals | `#if NODE_ROLE == ROLE_RECEIVER` / `#endif` around `receive_context`, `struct received_frame`, `receive_queue` | `//` on the two directive lines | Same declarations, now used by both roles |
| Above `packet_receive_callback()` | `#if NODE_ROLE == ROLE_RECEIVER` | `//` | Callback used by both roles |
| After `create_receive_context()` | `#endif` | `//` | |
| `send_oz_packet()` | `zsock_sendto(packet_socket, ...)` | `/* ... */` | `net_context_sendto(receive_context, ...)` |
| `send_oz_packet()` | `if (sent < 0) { ... errno ... return -errno; }` | `/* ... */` | `if (sent < 0) { ... return (int)sent; }` (`net_context_sendto` returns `-errno` directly) |
| `run_sender()` (periodic DATA sender) | Entire function | `#if 0 ... #endif /* legacy run_sender */` | `run_probe_sender()` |
| `run_receiver()` | Entire function | `#if 0 ... #endif /* legacy run_receiver */` | `run_probe_receiver()` |
| `main()` | Role-specific endpoint setup (`create_packet_socket()` for sender, `create_receive_context()` for receiver) | `/* ... */` | One `create_receive_context()` call for both roles |
| `main()` | `run_sender();` | `//` | `run_probe_sender();` |
| `main()` | `run_receiver();` | `//` | `run_probe_receiver();` |

`#if 0` is used instead of `/* */` for whole functions because those
functions contain their own `/* */` comments, and C block comments cannot
nest.

### 4.2 Original code kept active, with small in-place edits

| Item | Edit | Reason |
|------|------|--------|
| `create_packet_socket()` | Marked `__maybe_unused` | No longer called; avoids an unused-function warning. Body unchanged. |
| `receive_oz_packet()` | Marked `__maybe_unused` | Was already unused by both roles. Body unchanged. |
| `send_oz_packet()` | Marked `__maybe_unused` | Only the sender sends a bare PROBE with it; the receiver build doesn't call it. |
| `print_packet()` | Added a `Message Name = <NAME>` log line after `Message Type` | Readable logs |
| File header comment | Added probe/heartbeat description; original topology kept | Documentation |
| Payload comment block | Added how PROBE/HEARTBEAT use `value` and `sequence`; original lines kept | Documentation |
| `main()` banner | Added `HB Payload` line and the GUID print (`init_device_guid()`) | Diagnostics |
| `MESSAGE_TYPE_DATA`, `TX_PERIOD_MS`, `packet_socket`, `tx_sequence` | Unchanged | `tx_sequence` is now the probe sequence counter |

### 4.3 New code added

**Includes**

- `<zephyr/drivers/hwinfo.h>`: GUID
- `<zephyr/drivers/adc.h>`, `<zephyr/dt-bindings/adc/nrf-saadc.h>`: battery

**Defines**

- `MSG_TYPE_*` (7 types), `MSG_TYPE_FIRST`, `MSG_TYPE_LAST`, `MSG_HANDLER_COUNT`
- `PROBE_K_INITIAL_SECONDS`, `PROBE_N_INCREMENT_SECONDS`, `PROBE_K_MAX_SECONDS`
- `BATTERY_*` (see §2.4)
- `GUID_SIZE` (8), `OZ_MAX_BODY_SIZE` (64), `HEARTBEAT_BODY_SIZE` (13),
  `HEARTBEAT_FRAME_SIZE` (25)

**Types**

- `struct heartbeat_body`: GUID, battery mV, battery %, heartbeat sequence
- `message_handler_t`: `void (*)(const struct oz_packet *, const struct received_frame *)`

**Globals**

| Name | Purpose |
|------|---------|
| `probe_k_seconds` | Current K |
| `probe_outstanding`, `probe_outstanding_sequence` | Whether a PROBE is still waiting for a HEARTBEAT, and its sequence |
| `probe_timer` (sender only) | One-shot `k_timer` for K |
| `device_guid[8]` | This node's GUID, read at boot |
| `battery_adc`, `battery_adc_ready` | SAADC device and init status |
| `heartbeat_tx_sequence` | Heartbeat sequence # counter |
| `message_handlers[7]` | Function-pointer dispatch table |

All probe and heartbeat state is touched only by the main thread, which
runs both the timer logic and the handlers, so it needs no locking.

**Functions**

| Function | Role | Purpose |
|----------|------|---------|
| `message_type_name()` | both | Type number → name string for logs |
| `serialize_heartbeat_body()` / `deserialize_heartbeat_body()` | both | Little-endian encode/decode of the 13-byte body |
| `init_device_guid()` | both | Reads the 64-bit device ID via hwinfo |
| `init_battery_adc()` | both | Configures SAADC channel 0 on internal VDD |
| `read_battery()` | both | Returns mV and % (or 0 / `0xFF` on failure) |
| `send_oz_packet_with_body()` | both | Sends header + body through `receive_context` |
| `build_packet()` | both | Fills the OZ header with `source_id = LOCAL_SHORT_ADDR` |
| `print_heartbeat_body()` | both | Logs GUID, battery, heartbeat sequence, probe RSSI |
| `handle_probe()` | both (used by receiver) | Builds and sends the HEARTBEAT |
| `handle_heartbeat()` | both (used by sender) | Validates the body length, matches the sequence, resets K |
| `dispatch_message()` | both | Range check → table lookup → `NULL` check → call |
| `process_frame()` | both | RX hex dump → parse header from first 12 bytes → check destination → log MAC source/RSSI → dispatch |
| `probe_timer_expired()` | sender | Missed-heartbeat check, K += N, send PROBE, restart timer |
| `run_probe_sender()` | sender | Main loop: handle timer expiry, otherwise wait on `receive_queue` for at most the time left on K |
| `run_probe_receiver()` | receiver | Main loop: wait on `receive_queue`, `process_frame()` |

### 4.4 Revisions made inside the new code when heartbeat fields were added

The probe/heartbeat code was written first. Adding GUID, battery and
heartbeat sequence changed some of it, and those lines were also commented
out rather than deleted:

| Function | Commented out | Replaced by |
|----------|---------------|-------------|
| `handle_probe()` | `ret = send_oz_packet(&heartbeat, packet->source_id);` | `send_oz_packet_with_body(...)` with the serialized body |
| `handle_heartbeat()` | `ARG_UNUSED(frame);` | `frame` is now used to read the body |
| `handle_heartbeat()` | RSSI-only `LOG_INF` block | `"HEARTBEAT ... accepted"` + `print_heartbeat_body()` |
| `process_frame()` | `deserialize_packet(frame->data, frame->length, ...)` (exact 12-byte check) | Check `length >= 12`, then `deserialize_packet(frame->data, OZ_PACKET_SIZE, ...)` so longer messages with bodies are accepted |
| `process_frame()` | RX hex dump limit 20 bytes | 32 bytes (so the 25-byte heartbeat is shown in full) |

---

## 5. `prj.conf`

Added:

```
CONFIG_HWINFO=y   # hwinfo_get_device_id() for the GUID
CONFIG_ADC=y      # SAADC driver for battery level
```

## 6. `battery_adc.overlay` (new) and `CMakeLists.txt`

`CMakeLists.txt`, added after `set(BOARD ...)`, before `find_package(Zephyr)`:

```cmake
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/battery_adc.overlay)
```

`battery_adc.overlay`:

```dts
&adc {
	status = "okay";
};
```

The SAADC is disabled by default in `nrf52840.dtsi` and the board `.dts`
doesn't enable it. The channel itself is configured at runtime in
`init_battery_adc()`, so no channel child nodes are needed.

---

## 7. Adding the JOIN and NET ID handlers later

1. Write a handler with the common signature:

   ```c
   static void handle_join_request(
   	const struct oz_packet *packet,
   	const struct received_frame *frame)
   {
   	/* parse body from &frame->data[OZ_PACKET_SIZE],
   	 * length frame->length - OZ_PACKET_SIZE */
   }
   ```

2. Put it in its slot in `message_handlers[]`, replacing the `NULL`.
3. If the message carries data beyond the 12-byte header, define a body
   struct plus serialize/deserialize functions like the heartbeat ones, and
   send it with `send_oz_packet_with_body()`. The maximum body is
   `OZ_MAX_BODY_SIZE` (64 bytes).

---

## 8. What to look for in the logs

**Receiver** (on each PROBE):

- `PROBE seq=N from 0x0001 (sender K = K s)`
- `HEARTBEAT TX BODY` block with GUID, battery, HB sequence, probe RSSI
- `MAC TX -> 0x0001, payload=25 bytes`

**Sender:**

- `PROBE TX` every K seconds, `payload=12 bytes`
- `HEARTBEAT seq=N from 0x0003 accepted`, a `HEARTBEAT RX BODY` block, then
  `K reset to 2 s`
- With the receiver powered off:
  `No HEARTBEAT for PROBE seq=N, K incremented to ... s`, growing by 2 s up
  to 30 s. K returns to 2 s once the receiver is back.

## 9. Bug fixes found on hardware

### 9.1 Sender printed the banner, then went silent (no probes)

**Symptom:** the RTT output stopped mid-line just after
`Network interface found`, and no `PROBE TX` ever appeared.

**Diagnosis**, by reading target RAM through J-Link while the board ran:

- `tx_sequence` stayed 1 and `probe_timer` was never started, so no probe
  was ever sent.
- The main thread's state was `_THREAD_DEAD`: `main()` had returned.
- The deferred-log buffer still held the dropped messages:
  `SET_TX_POWER (0 dBm) failed: -16` and `IEEE 802.15.4 setup failed: -16`.

**Root cause (pre-existing code in `configure_ieee802154()`):** TX power was
set **after** PAN ID and short address. Once both of those are set, the
Zephyr IEEE 802.15.4 L2 treats the node as associated
(`is_associated()` in `subsys/net/l2/ieee802154/ieee802154_mgmt.c`) and
rejects any further parameter change with `-EBUSY`. This affected **both**
roles.

**Fix:** the TX-power step now runs right after the channel step, before
PAN ID. The original block after the short address is kept, commented out.
Order is now: channel → TX power → PAN ID → short address → `net_if_up()`.

**Verified on hardware** (sender, no receiver running): probes at 2 s, 4 s,
8 s and 14 s, with `No HEARTBEAT for PROBE seq=N, K incremented to 4 / 6 / 8 s`.

### 9.2 Boot errors were invisible over RTT

**Cause:** `CONFIG_LOG_BACKEND_RTT_MODE_BLOCK` with a 1 KB RTT up buffer. If
no RTT viewer is reading during boot, the buffer fills after about 1 KB.
The backend retries a few times, then marks the host as absent and drops
messages. Here that included the error that explained the failure, and the
output stopped mid-line.

**Fix (`prj.conf`):**

```
CONFIG_SEGGER_RTT_BUFFER_SIZE_UP=4096
CONFIG_LOG_BUFFER_SIZE=4096
```

The whole boot log now stays in the buffer until a viewer attaches. This was
verified by attaching the viewer several seconds after reset.

### 9.3 Channel range check

`IEEE802154_CHANNEL` was changed to `10`, which the 2.4 GHz O-QPSK PHY does
not support (valid: 11–26). At runtime this fails with
`SET_CHANNEL failed: -22` and `main()` returns. A compile-time check now
catches it:

```c
#if (IEEE802154_CHANNEL < 11) || (IEEE802154_CHANNEL > 26)
#error "IEEE802154_CHANNEL must be 11-26 for the 2.4 GHz band"
#endif
```

Both boards must use the same channel and PAN ID.

---

## 10. Known compiler warnings (pre-existing code)

- `receive_oz_packet`: `zsock_recvfrom` argument 5 discards `const`
  (original cast).
- `packet_receive_callback`: `source->addr != NULL` is always true
  (original check).
- Receiver build only: `tx_sequence` defined but not used (was present in
  the original receiver build).
