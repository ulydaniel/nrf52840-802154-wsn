# IEEE 802.15.4 Probe / Heartbeat — How the Code Works

This project runs on two **nRF52840** boards (OzturkLab board, nRF Connect SDK /
Zephyr). The two boards talk to each other directly over the **IEEE 802.15.4**
low-power radio.

- The **sender** (address `0x0001`) sends a **PROBE** message every *K* seconds.
  The message asks, in effect, "are you still alive?"
- The **receiver** (address `0x0003`) answers each PROBE with a **HEARTBEAT**
  message. The HEARTBEAT says "yes, I'm alive" and carries the receiver's
  unique ID, its battery level, how strongly it heard the probe, and a
  counter of heartbeats sent.
- When a PROBE goes unanswered, the sender **backs off** and waits longer before
  it probes again (K grows by N, up to a limit). Once a HEARTBEAT arrives, K
  goes back to its starting value.

All the application code is in one file, `src/main.c`. One `#define` near
the top selects which role a board plays.

This README explains:

1. The wireless sensor network (WSN) ideas the code is built on.
2. How a message travels through the system.
3. What every function does, and which WSN concept it implements.

For a detailed history of what was changed and why, see `CHANGES.md`.

---

## Table of contents

1. [Wireless sensor network concepts used here](#1-wireless-sensor-network-concepts-used-here)
2. [Project files](#2-project-files)
3. [Quick start](#3-quick-start)
4. [Configuration settings](#4-configuration-settings)
5. [The message formats](#5-the-message-formats)
6. [How the program runs, step by step](#6-how-the-program-runs-step-by-step)
7. [The probe timer and back-off, with an example](#7-the-probe-timer-and-back-off-with-an-example)
8. [Function reference](#8-function-reference)
9. [Global variables and data structures](#9-global-variables-and-data-structures)
10. [Disabled and unused code](#10-disabled-and-unused-code)
11. [Adding new message types](#11-adding-new-message-types)
12. [Reading the logs](#12-reading-the-logs)
13. [Glossary](#13-glossary)

---

## 1. Wireless sensor network concepts used here

A **wireless sensor network** is a group of small, battery-powered devices
(called **nodes** or **motes**) that measure something and report it over a
low-power radio. Usually one node collects the data. It is called the
**sink**, **coordinator**, or **gateway**. This project is the smallest
possible WSN: one collector (the sender, which asks questions) and one field
node (the receiver, which answers).

The ideas below show up throughout the code.

### 1.1 IEEE 802.15.4: the radio standard

IEEE 802.15.4 is the radio standard that Zigbee, Thread, and many WSNs are
built on. It was designed for **low power and low data rate** (250 kbit/s),
not for speed like Wi-Fi.

| Concept | Meaning | Where in the code |
|---|---|---|
| **Channel** | The 2.4 GHz band is split into 16 channels, numbered **11–26**. Both nodes must use the same one or they can't hear each other. | `IEEE802154_CHANNEL` (currently `11`). A compile-time check rejects values outside 11–26. |
| **PAN ID** | *Personal Area Network ID*, a 16-bit network name. Nodes ignore frames from other PANs, so several networks can share a channel. | `IEEE802154_PAN_ID` = `0x1234` |
| **Short address** | A 16-bit node address that is unique inside the PAN. It is much shorter than the 64-bit hardware address, which saves airtime and energy. | `SENDER_ADDR` = `0x0001`, `RECEIVER_ADDR` = `0x0003` |
| **TX power** | How loudly the radio transmits, in dBm. Higher power reaches farther but uses more battery. | `IEEE802154_TX_POWER_DBM` = `0` dBm (1 mW) |
| **Frame size** | A whole radio frame is at most **127 bytes**, including the MAC header. Messages must be small. | The application payload is 12 bytes (PROBE) or 25 bytes (HEARTBEAT). |

### 1.2 Layers: who builds what

Network software is built in layers. Each layer adds its own header in front
of the data it gets from the layer above.

```
 +-----------------------------------------------------------+
 |  APPLICATION (our code, main.c)                           |
 |  "OZ" header (12 bytes) + optional body (e.g. heartbeat)  |
 +-----------------------------------------------------------+
 |  MAC layer (Zephyr IEEE 802.15.4 "L2")                    |
 |  Adds/removes MAC header: PAN ID, MAC source/dest address |
 +-----------------------------------------------------------+
 |  PHY / radio (nRF52840 radio driver)                      |
 |  Sends bits over the air, measures RSSI                   |
 +-----------------------------------------------------------+
```

Our code only builds the **application payload**. The socket type
(`AF_PACKET` + `SOCK_DGRAM`) tells Zephyr to add the MAC header when we
transmit and remove it when we receive. So the bytes our code sends and
receives always start with `"OZ"`.

### 1.3 Two kinds of addresses: MAC (one hop) and application (end to end)

Each message carries addresses at two levels:

- **MAC address** (in the MAC header, filled in by Zephyr): who sends this
  radio frame and who should pick it up. This covers only **one hop**.
- **Application address** (`source_id` / `destination_id` in the OZ header,
  filled in by our code): who **originally** sent the message and who it is
  **finally** meant for.

With two nodes, both levels hold the same addresses. Keeping them separate
prepares the protocol for **multi-hop** networks, where a message is relayed
through intermediate nodes. In that case the MAC addresses change at each
hop, but the application addresses stay the same.

### 1.4 RSSI: link quality

**RSSI** (*Received Signal Strength Indicator*) is how strong a frame was
when it arrived, in dBm. It is always negative, and **closer to 0 means
stronger**:

| RSSI | Rough meaning |
|---|---|
| −30 to −60 dBm | Very strong (boards close together) |
| −60 to −80 dBm | Good |
| −80 to −95 dBm | Weak, some packets may be lost |
| below about −95 dBm | At the edge of reception |

The receiver measures the RSSI of each PROBE and **sends it back** in the
HEARTBEAT. That tells the sender how well its own transmissions reach the
receiver. WSNs use this to pick good neighbours and routes.

### 1.5 Liveness: probes and heartbeats

A sink needs to know which nodes are still working. A node can fail because
its battery is dead, it moved out of range, or it crashed. Two common
patterns:

- **Heartbeat:** the node periodically says "I'm alive".
- **Probe / poll:** the sink asks "are you alive?" and the node answers.

This project combines them: the sender **probes**, and the receiver answers
with a **heartbeat**. A missing heartbeat means either the node is down or
the link is bad.

### 1.6 Back-off: saving energy when nobody answers

If the receiver is gone, probing it every 2 seconds just wastes the sender's
battery and fills the shared channel with useless traffic. So the sender
**backs off**. Each unanswered probe makes the wait K longer by N seconds,
up to a maximum. One answered probe resets K. This is **additive (linear)
back-off**, a simple and common WSN technique.

### 1.7 Sequence numbers: matching replies and detecting loss

Every message has a **sequence number** that goes up by one each time.

- The HEARTBEAT header **echoes** the PROBE's sequence number. The sender
  can then tell *which* probe is being answered. A late reply to an old
  probe is ignored.
- The HEARTBEAT body also carries the receiver's **own** heartbeat counter.
  A gap in that counter (for example 5, 6, 8) shows that a heartbeat was
  lost on the way back.

### 1.8 Node identity (GUID) and battery: device health

- **GUID**: a 64-bit ID burned into every nRF52840 at the factory. Short
  addresses are assigned by configuration and can be duplicated by mistake.
  The GUID is guaranteed unique, so it identifies the physical board.
- **Battery level**: the receiver measures its supply voltage with the
  on-chip ADC (*analog-to-digital converter*) and reports it in millivolts
  and percent. The sink can then plan battery changes before a node dies.

### 1.9 Byte order (endianness)

A 16-bit number such as `0x1234` takes two bytes, and they can be stored in
either order. When devices exchange data, both sides must agree on the
order:

- The **OZ application fields** are always written **little endian** (low
  byte first: `34 12`), using `sys_put_le16()` / `sys_get_le16()`.
- **MAC addresses** passed to Zephyr's socket layer are **big endian** (high
  byte first), using `sys_put_be16()` / `sys_get_be16()`. Zephyr expects
  that format.

---

## 2. Project files

| File | Purpose |
|---|---|
| `src/main.c` | All application code (both roles) |
| `prj.conf` | Zephyr settings: networking, IEEE 802.15.4, sockets, HWINFO (GUID), ADC (battery), RTT logging |
| `CMakeLists.txt` | Build script. Also adds `battery_adc.overlay` |
| `battery_adc.overlay` | Device-tree change that turns on the nRF52840 SAADC (the ADC used for battery) |
| `CHANGES.md` | Detailed change log and bug-fix notes |

---

## 3. Quick start

1. Open `src/main.c` and set the role for the board you're about to flash:

   ```c
   #define NODE_ROLE         ROLE_SENDER     /* board 1 */
   /* or */
   #define NODE_ROLE         ROLE_RECEIVER   /* board 2 */
   ```

2. Build and flash board 1. Then change the role, build again, and flash
   board 2.
3. Connect an RTT viewer (for example J-Link RTT Viewer) to see the logs.
   Logs go out over **SEGGER RTT**, not UART.
4. The sender should log `PROBE TX` every 2 seconds and
   `HEARTBEAT ... accepted` for each reply. If you power off the receiver,
   K grows: 4 s, 6 s, 8 s, and so on, up to 30 s.

Both boards **must** use the same `IEEE802154_CHANNEL` and
`IEEE802154_PAN_ID`.

---

## 4. Configuration settings

Every setting is a `#define` near the top of `src/main.c`.

| Setting | Value | What it controls |
|---|---|---|
| `NODE_ROLE` | `ROLE_SENDER` / `ROLE_RECEIVER` | Which role this board runs |
| `IEEE802154_CHANNEL` | `11` | Radio channel (11–26) |
| `IEEE802154_PAN_ID` | `0x1234` | Network ID |
| `IEEE802154_TX_POWER_DBM` | `0` | Transmit power |
| `SENDER_ADDR` / `RECEIVER_ADDR` | `0x0001` / `0x0003` | Short addresses |
| `PROTOCOL_VERSION` | `1` | Version byte. Packets with another version are dropped |
| `PROBE_K_INITIAL_SECONDS` | `2` | Starting probe interval K |
| `PROBE_N_INCREMENT_SECONDS` | `2` | N, the amount added to K after a missed heartbeat |
| `PROBE_K_MAX_SECONDS` | `30` | Largest K allowed |
| `RX_QUEUE_DEPTH` | `4` | How many received frames can wait to be processed |
| `RX_WAIT_TIMEOUT_SECONDS` | `5` | Receiver logs "no packet" after this much silence |
| `BATTERY_FULL_MV` / `BATTERY_EMPTY_MV` | `3000` / `2000` | Voltages reported as 100 % and 0 % |
| `BATTERY_ADC_GAIN` / `_RESOLUTION` | `1/6`, `12` bits | ADC setup (measures 0–3.6 V) |

Note: the comment block at the top of `main.c` still says "Channel : 15".
The `#define` is what the code actually uses.

---

## 5. The message formats

### 5.1 The OZ header (every message, 12 bytes)

```
Byte:  0    1    2        3        4-5        6-7         8-9      10-11
     +----+----+--------+--------+----------+-----------+--------+----------+
     | 'O'| 'Z'| version| msg    | source   | final     | value  | sequence |
     |    |    |  (1)   | type   | address  | dest addr | (int16)|          |
     +----+----+--------+--------+----------+-----------+--------+----------+
```

- `"OZ"` is a **signature**. Frames that don't start with it come from some
  other device or protocol and are ignored.
- The **message type** tells the receiving side which handler should
  process the message (see 6.4).
- **value** is a general-purpose 16-bit field. Its meaning depends on the
  message type:
  - In a PROBE, it carries the sender's current K, in seconds.
  - In a HEARTBEAT, it carries the RSSI at which the receiver heard the
    probe, in dBm.

### 5.2 Message types

| Type | Value | Handler | Status |
|---|---|---|---|
| PROBE | 1 | `handle_probe()` | Implemented |
| HEARTBEAT | 2 | `handle_heartbeat()` | Implemented |
| JOIN_REQUEST | 3 | none | Reserved for later |
| JOIN_RESPONSE | 4 | none | Reserved |
| JOIN_ACK | 5 | none | Reserved |
| NETID_REQUEST | 6 | none | Reserved |
| NETID_RESPONSE | 7 | none | Reserved |

The JOIN and NETID types are placeholders for a future **network joining**
procedure. In a real WSN, a new node asks the coordinator for permission to
join and is given a network ID or address.

### 5.3 PROBE (12 bytes)

A PROBE is the OZ header with nothing after it:

| Field | Value |
|---|---|
| type | `1` |
| source → destination | `0x0001` → `0x0003` |
| value | sender's current K (seconds) |
| sequence | probe counter (1, 2, 3, …) |

### 5.4 HEARTBEAT (25 bytes = 12-byte header + 13-byte body)

| Frame bytes | Field | Notes |
|---|---|---|
| 0–11 | OZ header | type `2`, `0x0003` → `0x0001`, **value = probe RSSI**, **sequence = copied from the PROBE** |
| 12–19 | GUID | 8-byte factory device ID |
| 20–21 | Battery voltage | millivolts, little endian |
| 22 | Battery percent | 0–100, or `0xFF` if unknown |
| 23–24 | Heartbeat sequence | receiver's own counter, little endian |

Example of a heartbeat as raw bytes (as printed by `TX RAW:` / `RX:`):

```
4F 5A 01 02 03 00 01 00 D6 FF 07 00 | A1 B2 C3 D4 E5 F6 07 18 | BE 0A | 4B | 05 00
'O''Z' v  HB  src=3 dst=1 -42dBm s=7 |        GUID             | 2750mV| 75%| HB #5
```

---

## 6. How the program runs, step by step

### 6.1 Startup (`main()`), same for both roles

```
main()
 ├─ print banner (role, channel, PAN ID, TX power, address, payload sizes)
 ├─ init_device_guid()        read the 64-bit factory ID
 ├─ init_battery_adc()        set up the ADC (if this fails, battery is "unknown")
 ├─ configure_ieee802154()    channel → TX power → PAN ID → short addr → interface up
 ├─ create_receive_context()  open one radio endpoint used for both RX and TX
 └─ run_probe_sender()   or   run_probe_receiver()     (loops forever)
```

If the radio setup or the endpoint setup fails, `main()` logs the error and
returns. Nothing else happens after that, so check the log for the error
line.

### 6.2 The receive path: from the air to a handler

Zephyr's network stack has its own thread. Our receive callback runs in
that thread. Slow work such as logging shouldn't happen there, so the
callback just **copies the frame into a queue**. The main thread picks
frames up from the queue and does the actual processing.

```
 Radio receives a frame
        │
        ▼
 Zephyr MAC layer: checks PAN ID / MAC address, strips the MAC header
        │
        ▼
 packet_receive_callback()            (network thread)
   - reads RSSI and MAC source address from the packet's metadata
   - copies the payload bytes into a struct received_frame
   - k_msgq_put(&receive_queue, ...)  (drops the frame if the queue is full)
        │
        ▼
 receive_queue  (holds up to 4 frames)
        │
        ▼
 run_probe_sender() / run_probe_receiver()   (main thread)
   - k_msgq_get(&receive_queue, ...)
        │
        ▼
 process_frame()
   - is it at least 12 bytes?           no  → ignore
   - starts with "OZ", version 1?       no  → ignore
   - destination_id == my address?      no  → ignore
   - log MAC source, RSSI, header
        │
        ▼
 dispatch_message()
   - look up message_handlers[type - 1]
        │
        ├── PROBE      → handle_probe()      (receiver replies with a HEARTBEAT)
        └── HEARTBEAT  → handle_heartbeat()  (sender resets K)
```

### 6.3 The transmit path

```
 build_packet()              fill in the OZ header fields
        │
        ▼
 serialize_packet()          turn the struct into 12 bytes, little endian
 (+ serialize_heartbeat_body() for a HEARTBEAT, then appended after the header)
        │
        ▼
 send_oz_packet() / send_oz_packet_with_body()
   - build a sockaddr_ll holding the MAC destination (big endian)
   - net_context_sendto(receive_context, ...)
        │
        ▼
 Zephyr MAC layer adds the MAC header → radio transmits
```

### 6.4 The handler table: "function-pointer dispatch"

Instead of a long `if / else if` chain over message types, the code keeps
an **array of function pointers**. The message type selects the array
slot:

```c
message_handlers[0] = handle_probe       // type 1
message_handlers[1] = handle_heartbeat   // type 2
message_handlers[2..6] = NULL            // types 3-7, not written yet
```

`dispatch_message()` computes `index = type - 1` and calls whatever
function sits in that slot. Supporting a new message type then only means
writing a handler and putting it in its slot. The receive code doesn't
change at all.

### 6.5 Sender main loop (`run_probe_sender()`)

The sender has two jobs at once: **send a probe each time timer K runs
out**, and **handle any heartbeats that arrive** in between. The loop
handles both:

```
start timer K (one-shot)
loop forever:
    if timer K has expired:
        probe_timer_expired()   → maybe grow K, send PROBE, restart timer
        continue
    wait for a received frame, but at most until timer K expires
    if a frame arrived: process_frame()
```

Waiting on the queue only until the timer runs out means the sender never
sleeps through a timer expiry. Yet it doesn't spin and waste CPU time
either.

### 6.6 Receiver main loop (`run_probe_receiver()`)

The receiver is simpler. It only reacts:

```
loop forever:
    wait up to 5 s for a frame
    if nothing arrived: log "No packet callback in the last 5 seconds"
    else: process_frame()      → a PROBE leads to handle_probe() → HEARTBEAT sent
```

---

## 7. The probe timer and back-off, with an example

The rules, all enforced by `probe_timer_expired()` and `handle_heartbeat()`:

1. When timer K runs out, the sender first checks whether the **previous**
   probe is still unanswered (`probe_outstanding == true`). If it is, then
   `K = min(K + N, K_MAX)`.
2. Then it sends a new PROBE, remembers its sequence number as the
   outstanding probe, and restarts the timer with the current K.
3. A HEARTBEAT whose sequence number **matches** the outstanding probe
   clears it and resets K to the initial value.
4. A HEARTBEAT with any other sequence number is a late or unexpected reply.
   It is logged as stale and ignored.
5. A probe whose transmission failed still counts as sent but unanswered.

So the receiver always has **K seconds** to reply to each probe.

**Example: receiver switched off** (K starts at 2, N = 2):

| Time (s) | Event | K after |
|---|---|---|
| 0 | Boot, timer started with K = 2 | 2 |
| 2 | Timer runs out. No probe outstanding yet. Send PROBE #1 | 2 |
| 4 | #1 unanswered → K = 4. Send PROBE #2 | 4 |
| 8 | #2 unanswered → K = 6. Send PROBE #3 | 6 |
| 14 | #3 unanswered → K = 8. Send PROBE #4 | 8 |
| … | … keeps growing by 2 … | … |
| | Stops growing at 30 s | 30 |

**Example: receiver comes back.** As soon as one HEARTBEAT matching the
current probe arrives, K drops back to 2 s and probing speeds up again.

---

## 8. Function reference

Functions are grouped by job. Each entry says what the function does, the
WSN idea behind it, and anything worth knowing.

### 8.1 Logging helpers

#### `role_name()`
Returns `"SENDER"` or `"RECEIVER"` for the startup banner. Which string it
returns is fixed at compile time from `NODE_ROLE`.

#### `message_type_name(type)`
Converts a message type number into a readable name (`1` → `"PROBE"`), so
the logs show names instead of numbers.

#### `print_packet(title, packet)`
Logs every field of an OZ header (version, type, name, source, destination,
value, sequence) between two dashed lines. It is called for each packet
sent and received.

#### `print_heartbeat_body(title, body, probe_rssi_dbm)`
Logs the contents of a HEARTBEAT: GUID, battery (or "unknown"), heartbeat
sequence, and probe RSSI (or "unavailable"). Both sides use it: the
receiver before sending, the sender after receiving.

### 8.2 Converting messages to bytes and back (serialization)

Over the air everything is just bytes, while C code works with structs.
**Serializing** turns a struct into bytes. **Deserializing** turns bytes
back into a struct. The code copies each field explicitly instead of
sending the raw struct memory. The compiler may add hidden padding inside a
struct, and its byte order depends on the CPU, so sending raw struct memory
isn't safe between devices.

#### `serialize_packet(packet, buffer)`
Writes the 12-byte OZ header into `buffer`. Single bytes are copied
directly. 16-bit fields are written little endian with `sys_put_le16()`.

#### `deserialize_packet(buffer, length, packet)`
Reads 12 bytes back into a `struct oz_packet`. It returns `false` (the
packet is rejected) when:
- `length` isn't exactly 12,
- the first two bytes aren't `'O' 'Z'`, or
- the version isn't `PROTOCOL_VERSION`.

This is the first **filter**: it drops traffic from other devices or
protocols on the same channel.

#### `serialize_heartbeat_body(body, buffer)`
Writes the 13-byte HEARTBEAT body: GUID (8 bytes), battery mV (2), battery %
(1), heartbeat sequence (2).

#### `deserialize_heartbeat_body(buffer, length, body)`
Does the reverse. Returns `false` if the body isn't exactly 13 bytes.

#### `build_packet(packet, type, destination, value, sequence)`
A shortcut that fills in a complete OZ header. It zeroes the struct, sets
`"OZ"`, the version, and the type, and always uses **this node's** address
(`LOCAL_SHORT_ADDR`) as the source.

### 8.3 Node health: identity and battery

#### `init_device_guid()`
Reads the nRF52840's 64-bit factory device ID with
`hwinfo_get_device_id()` into `device_guid[]` and prints it. If the read
fails, the GUID stays all zeros and a warning is logged.
*WSN concept:* a permanent, unique identity that doesn't depend on how
addresses were configured.

#### `init_battery_adc()`
Prepares the nRF52840's ADC (the **SAADC**) to measure the chip's own
supply voltage (**VDD**). Settings:
- input = internal VDD (no external wiring needed),
- reference = internal 0.6 V, gain = 1/6, so the measurable range is
  0–3.6 V,
- 10 µs acquisition time.

On success it sets `battery_adc_ready = true`. On failure it logs a warning
and the node keeps running: heartbeats then report the battery as unknown.

> Limitation: VDD equals the battery voltage only when the battery powers
> the chip directly (coin cell, 2×AA). If there's a voltage regulator in
> between, this measures the regulated voltage instead.

#### `read_battery(&millivolts, &percent)`
Takes one ADC sample and converts it:
1. Raw reading (0–4095 for 12 bits) → millivolts with
   `adc_raw_to_millivolts()`.
2. Millivolts → percent using a straight line between the empty and full
   voltages:

   \[
   \text{percent} = \frac{(\text{mV} - 2000) \times 100}{3000 - 2000}
   \]

   clamped to 0–100. Example: 2750 mV → 75 %.

On any error it reports `0 mV` and `0xFF` (unknown).
*WSN concept:* energy monitoring, so dying nodes can be found before they
go silent.

### 8.4 Radio and network setup

#### `configure_ieee802154()`
Brings the radio up with our settings, using Zephyr's `net_mgmt()`
requests:

1. Find the default network interface (the 802.15.4 radio). Take it down if
   it's already up.
2. **Set channel** (`SET_CHANNEL`)
3. **Set TX power** (`SET_TX_POWER`)
4. **Set PAN ID** (`SET_PAN_ID`)
5. **Set short address** (`SET_SHORT_ADDR`)
6. **Bring the interface up** (`net_if_up()`)

**The order matters.** Once both the PAN ID and the short address are set,
Zephyr considers the node **associated** (joined to a network) and rejects
any further setting change with error `-16` (`-EBUSY`). That's why TX power
is set *before* the PAN ID. The original TX-power code further down is kept
commented out (see `CHANGES.md` §9.1).

#### `create_receive_context()`
Creates the node's **single radio endpoint** (a Zephyr `net_context`) that
is used for both receiving and transmitting:
1. `net_context_get(AF_PACKET, SOCK_DGRAM, ETH_P_IEEE802154)` creates a
   "raw IEEE 802.15.4, payload only" endpoint.
2. `net_context_recv()` registers `packet_receive_callback` for incoming
   frames.
3. `net_context_bind()` attaches the endpoint to our radio interface.
4. `net_context_recv()` is called again after binding. This mirrors what
   Zephyr's own packet sockets do internally.

If any step fails, the context is released (`net_context_put()`) and the
error is returned.

**Why one endpoint instead of two:** Zephyr gives *every* open raw endpoint
its own copy of each received frame. A second endpoint used only for
sending would never read its copies, and they would slowly use up all the
network receive buffers.

### 8.5 Sending

#### `send_oz_packet(packet, mac_destination)`
Sends a **header-only** OZ message (used for PROBE):
1. Serializes the header into 12 bytes.
2. Fills a `sockaddr_ll` (link-layer address) with the interface and the
   **MAC destination** short address (big endian).
3. Prints the raw bytes (`TX RAW:`).
4. Calls `net_context_sendto()` on `receive_context`. Zephyr adds the MAC
   header and transmits.
5. Checks the result: a negative value is an error, and a byte count other
   than 12 counts as a short send (`-EIO`).

#### `send_oz_packet_with_body(packet, body, body_length, mac_destination)`
Does the same, but adds a **message body** after the 12-byte header (used
for HEARTBEAT). Bodies larger than `OZ_MAX_BODY_SIZE` (64 bytes) are
refused with `-EMSGSIZE`, which keeps the frame well under the 127-byte
radio limit.

### 8.6 Receiving

#### `packet_receive_callback(context, pkt, ..., status, ...)`
Called by Zephyr's network thread for every received frame:
1. On an error status, or if there's no packet, it releases the packet
   and returns.
2. Reads the **RSSI** of this exact frame:
   `net_pkt_ieee802154_rssi_dbm(pkt)`.
3. Reads the **MAC source address**, if it's a 2-byte short address.
   Otherwise it stores `0xFFFF`.
4. Copies the payload bytes (MAC header already removed) into a
   `struct received_frame`.
5. Puts the frame into `receive_queue` without waiting. If the queue is
   full, the frame is dropped with a warning.
6. Always releases the Zephyr packet (`net_pkt_unref`) so its buffer can be
   reused.

*Why a callback instead of a socket read?* The callback gets Zephyr's
original packet object, which holds **RSSI metadata**. A plain socket
`recvfrom()` doesn't expose RSSI.

#### `process_frame(frame)`
The common "is this message for me?" check for both roles:
1. Prints the first 32 received bytes (`RX:`).
2. Rejects frames shorter than 12 bytes.
3. Decodes the first 12 bytes as an OZ header (`deserialize_packet`).
   Anything after those 12 bytes is a body that the handler will read.
4. Rejects the message if the **application** `destination_id` isn't
   this node's address.
5. Logs the MAC source, the RSSI, and the header.
6. Passes the message to `dispatch_message()`.

#### `dispatch_message(packet, frame)`
Looks up the handler for the message type and calls it:
- type outside 1–7 → `"Unknown message type N ignored"`
- slot is `NULL` → `"No handler implemented for NAME (type N)"`
- otherwise → `handler(packet, frame)`

### 8.7 Message handlers (the protocol logic)

All handlers share one signature,
`void handler(const struct oz_packet *packet, const struct received_frame *frame)`,
so they fit in the `message_handlers[]` table. `packet` is the decoded
header. `frame` holds the raw bytes (for reading a body), the RSSI, and the
MAC source.

#### `handle_probe(packet, frame)`: runs on the receiver
Answers a PROBE with a HEARTBEAT:
1. Logs `PROBE seq=N from 0x0001 (sender K = K s)`.
2. Builds the HEARTBEAT header with `build_packet()`:
   - destination = the probe's source (reply to whoever asked),
   - value = `frame->rssi_dbm` (how strongly the probe was heard),
   - sequence = the probe's sequence (so the sender can match the reply).
3. Fills the body: GUID, battery (`read_battery()`), and
   `heartbeat_tx_sequence++`.
4. Serializes the body, logs everything, and sends with
   `send_oz_packet_with_body()`.

#### `handle_heartbeat(packet, frame)`: runs on the sender
Accepts a HEARTBEAT and resets the back-off:
1. Decodes the 13-byte body that follows the header. If the length is
   wrong, it logs a warning and ignores the message.
2. Checks that a probe is outstanding **and** that the heartbeat's sequence
   number matches it. Otherwise it logs
   `Stale/unsolicited HEARTBEAT ... ignored`. This prevents a late answer to
   an old probe from hiding the fact that the newest probe was missed.
3. On a match: clears `probe_outstanding`, sets
   `K = PROBE_K_INITIAL_SECONDS`, and logs
   `HEARTBEAT seq=N ... accepted`, the body, and `K reset to 2 s`.

### 8.8 Role main loops

#### `probe_timer_expired()`: sender only
Runs each time timer K runs out:
1. If the last probe is still unanswered: `K += N`, capped at `K_MAX`, and
   logs `No HEARTBEAT for PROBE seq=N, K incremented to K s`.
2. Builds a PROBE (value = K, sequence = `tx_sequence++`) and sends it.
3. Marks it as outstanding, even if the send failed.
4. Restarts the one-shot timer with the current K.

#### `run_probe_sender()`: sender only
Starts the timer, then loops forever as described in 6.5.
`k_timer_status_get()` both checks whether the timer expired *and* resets
that flag. `k_timer_remaining_ticks()` gives the time left, which is used
as the maximum wait on the receive queue.

#### `run_probe_receiver()`: receiver only
Loops forever: waits up to 5 s on the receive queue, logs if nothing
arrived, otherwise calls `process_frame()` (see 6.6).

#### `main()`
The startup sequence in 6.1.

### 8.9 Unused or disabled functions (kept for reference)

| Function | Status | What it was for |
|---|---|---|
| `create_packet_socket()` | Compiled but never called (`__maybe_unused`) | Opened a separate AF_PACKET socket for sending. Replaced by the single shared context. |
| `receive_oz_packet()` | Compiled but never called | Read packets with `zsock_recvfrom()`. It can't get RSSI, so the callback approach replaced it. |
| `run_sender()` | Disabled with `#if 0` | The original demo: sends a DATA packet every second with a counter value. |
| `run_receiver()` | Disabled with `#if 0` | The original demo receiver: printed each DATA packet. |

---

## 9. Global variables and data structures

### Structures

| Name | Contents |
|---|---|
| `struct oz_packet` | The decoded 12-byte OZ header: signature, version, message type, source, destination, value, sequence |
| `struct heartbeat_body` | GUID[8], battery mV, battery %, heartbeat sequence |
| `struct received_frame` | One received payload (up to the MTU), its length, MAC source address, and RSSI. This is what goes through `receive_queue`. |
| `message_handler_t` | The function-pointer type every handler must match |

### Globals

| Name | Used by | Meaning |
|---|---|---|
| `iface` | both | The IEEE 802.15.4 network interface |
| `receive_context` | both | The one radio endpoint, used for RX and TX |
| `receive_queue` | both | Queue (4 slots) from the RX callback to the main thread |
| `tx_sequence` | sender | Next PROBE sequence number |
| `probe_k_seconds` | sender | Current interval K |
| `probe_outstanding` | sender | `true` while the last PROBE has no matching HEARTBEAT yet |
| `probe_outstanding_sequence` | sender | Sequence number of that PROBE |
| `probe_timer` | sender | One-shot Zephyr timer for K |
| `device_guid[8]` | both | This board's factory ID |
| `battery_adc`, `battery_adc_ready` | both | ADC device and whether setup succeeded |
| `heartbeat_tx_sequence` | receiver | Next heartbeat counter value |
| `message_handlers[7]` | both | The dispatch table |
| `packet_socket` | none | Belongs to the unused `create_packet_socket()` |

**No locking is needed:** all probe and heartbeat state is read and written
only by the main thread. The network thread only touches the queue, and
Zephyr's message queue is already thread-safe.

---

## 10. Disabled and unused code

The project keeps old code instead of deleting it:

- Whole old functions are wrapped in `#if 0 ... #endif`. `/* */` can't be
  used here because C block comments can't nest, and these functions
  contain their own comments.
- Single replaced lines are commented out with `//` or `/* */` next to their
  replacements.

`CHANGES.md` §4 lists each of these places.

---

## 11. Adding new message types

To implement, for example, `JOIN_REQUEST`:

1. Write a handler with the standard signature:

   ```c
   static void handle_join_request(
   	const struct oz_packet *packet,
   	const struct received_frame *frame)
   {
   	/* body bytes start at &frame->data[OZ_PACKET_SIZE],
   	 * body length is frame->length - OZ_PACKET_SIZE */
   }
   ```

2. Put it in its slot in `message_handlers[]`, replacing `NULL`.
3. If the message has a body, write a body struct plus
   serialize/deserialize functions (copy the heartbeat ones), and send the
   message with `send_oz_packet_with_body()`. Bodies can be up to 64 bytes.

No changes to the receive path are needed.

---

## 12. Reading the logs

**Receiver, for each probe:**

```
Packet received with size 12
RX: 4F 5A 01 01 01 00 03 00 02 00 07 00
MAC source = 0x0001
RSSI       = -42 dBm
PROBE seq=7 from 0x0001 (sender K = 2 s)
HEARTBEAT TX BODY ... GUID / Battery / HB Sequence / Probe RSSI
MAC TX -> 0x0001, payload=25 bytes
```

**Sender, normal operation:**

```
PROBE TX ... Sequence = 7
MAC TX -> 0x0003, payload=12 bytes
Packet received with size 25
HEARTBEAT seq=7 from 0x0003 accepted
HEARTBEAT RX BODY ...
K reset to 2 s
```

**Sender, receiver unreachable:**

```
No HEARTBEAT for PROBE seq=8, K incremented to 4 s
No HEARTBEAT for PROBE seq=9, K incremented to 6 s
...
```

**Common errors:**

| Log line | Likely cause |
|---|---|
| `SET_CHANNEL failed: -22` | Channel outside 11–26 (normally caught at compile time now) |
| `SET_TX_POWER ... failed: -16` | A setting was changed after PAN ID and short address were set (node already "associated") |
| `Packet ignored: destination=...` | The message was meant for a different node |
| `Non-OZ packet ignored` | Traffic from another device, or a version mismatch |
| `RX queue full; packet dropped` | Frames arrive faster than the main thread can process them |
| `Battery = unknown` | ADC not enabled or failed (check `battery_adc.overlay`) |
| Nothing at all after `Network interface found` | An early error was lost. Attach the RTT viewer and reset the board. The 4 KB RTT buffer keeps the boot log until a viewer connects. |

---

## 13. Glossary

| Term | Meaning |
|---|---|
| **WSN** | Wireless sensor network: small battery-powered nodes reporting over low-power radio |
| **Node / mote** | One device in the network |
| **Sink / coordinator** | The node that collects data from the others. Here, the sender plays this part. |
| **IEEE 802.15.4** | Low-power, low-rate radio standard (the base for Zigbee and Thread) |
| **PHY** | Physical layer: the radio itself |
| **MAC / L2** | Medium Access Control, layer 2: frames, addresses, PAN ID |
| **PAN / PAN ID** | Personal Area Network and its 16-bit identifier |
| **Short address** | A 16-bit node address inside a PAN |
| **Channel** | One of the 16 radio frequencies (11–26) in the 2.4 GHz band |
| **dBm** | Power relative to 1 milliwatt. 0 dBm = 1 mW, −30 dBm = 0.001 mW. |
| **RSSI** | Received signal strength. Closer to 0 is stronger. |
| **Probe** | A request asking a node to show it's alive |
| **Heartbeat** | A message saying "I'm alive", here sent in reply to a probe |
| **Back-off** | Waiting longer between attempts when there's no answer |
| **Sequence number** | A counter in each message, used to match replies and spot losses |
| **GUID** | Globally unique ID; here the nRF52840's 64-bit factory device ID |
| **SAADC** | The nRF52840's analog-to-digital converter, used to measure voltage |
| **VDD** | The chip's supply voltage |
| **Serialize** | Convert a struct into bytes to send. Deserialize is the reverse. |
| **Little / big endian** | Low byte first / high byte first, for multi-byte numbers |
| **`net_context`** | Zephyr's low-level network endpoint (what a socket uses internally) |
| **`AF_PACKET` / `SOCK_DGRAM`** | A raw link-layer endpoint where Zephyr builds the MAC header and the app supplies only the payload |
| **Message queue (`k_msgq`)** | Zephyr's thread-safe queue for passing fixed-size items between threads |
| **RTT** | SEGGER Real-Time Transfer: log output through the J-Link debugger instead of a UART |
