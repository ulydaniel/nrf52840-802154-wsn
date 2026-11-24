/*
 * OzturkLab nRF52840
 *
 * IEEE 802.15.4 data-collection tree (root network)
 *
 * One source file supports:
 *
 *      ROLE_ROOT      — fixed address 1:254 (0x01FE)
 *      ROLE_JOINING   — starts unassigned 255:254 (0xFFFE)
 *
 * Application addresses are network:node (see MAKE_ADDRESS).
 * Only the root address is hard-coded. Joining nodes receive
 * addresses 1:1 .. 1:253 dynamically through the join protocol
 * (see documentation/SECTION4_PROTOCOL_FLOW.md).
 *
 * Node FSM (assignment §5):
 *
 *      UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
 *
 *      Root boots already REGISTERED at 1:254.
 *      Joining nodes boot UNREGISTERED, then enter PROBING.
 *
 * Protocol flow (assignment §4.1–4.6):
 *
 *      Joining (PROBING)  --PROBE broadcast-->  Registered node
 *      Joining (PROBING)  <-HEARTBEAT (MAC broadcast; OZ dest=joiner)--
 *                           Registered node
 *      Joining selects parent (lowest hop, then strongest RSSI,
 *      then lowest address), then:
 *      Joining (WAITING)  --JOIN_REQUEST----->  Root
 *      Joining (WAITING)  <-JOIN_RESPONSE-----  Root (assigns 1:n)
 *      Joining (REGISTERED)--JOIN_ACK-------->  Root (marks active)
 *
 *      Joining node: every K seconds a PROBE is broadcast.
 *      Heartbeats collected during a selection window drive
 *      parent choice. Join response timeouts retransmit the
 *      request or revert to UNREGISTERED / PROBING.
 *
 *      HEARTBEAT carries: probe RSSI, GUID, battery level
 *      and a heartbeat sequence number (see Message bodies).
 *
 *      Both roles decode received packets and dispatch them
 *      through a function-pointer table indexed by message type.
 *
 *      After REGISTERED, joining nodes send DATA every 1 s
 *      (sequential sensor value) toward their parent. Intermediate
 *      parents forward toward the root; the root reports missing
 *      sequences per child source.
 *
 * Threading (Zephyr):
 *      Radio callback  → receive_queue (never blocks)
 *      RX/protocol thread → process_frame / FSM / timers;
 *                          submit TX via tx_queue
 *      TX thread       → sole caller of net_context_sendto()
 *
 *      See documentation/RX_TX_THREADS.md.
 *
 * Channel : 11
 * PAN ID  : 0x1234
 *
 * Zephyr / NCS:
 *      nRF Connect SDK 3.4.0
 *      Zephyr 4.4.0
 *
 * Application uses AF_PACKET + SOCK_DGRAM.
 *
 * The IEEE 802.15.4 L2 constructs/parses the MAC header.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ieee802154.h>
#include <zephyr/net/ieee802154_mgmt.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ieee802154_pkt.h>

#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


/* =========================================================
 * USER CONFIGURATION
 * ========================================================= */

#define ROLE_ROOT         1
#define ROLE_JOINING      2


/*
 * Change ONLY this line for each board:
 *
 * ROLE_ROOT     — tree root, fixed address 1:254
 * ROLE_JOINING  — starts unassigned; gets 1:1..1:253 via join
 */

#define NODE_ROLE         ROLE_ROOT


/* =========================================================
 * IEEE 802.15.4 configuration
 * ========================================================= */

#define IEEE802154_CHANNEL       11
#define IEEE802154_PAN_ID        0x1234

/* 2.4 GHz O-QPSK (nRF52840) only supports channels 11-26. */
#if (IEEE802154_CHANNEL < 11) || (IEEE802154_CHANNEL > 26)
#error "IEEE802154_CHANNEL must be 11-26 for the 2.4 GHz band"
#endif

/* Select the requested radio transmit power in dBm.
 * Common nRF52840 settings are:
 *  8, 7, 6, 5, 4, 3, 2, 0,
 * -4, -8, -12, -16, -20, -40
 */
#define IEEE802154_TX_POWER_DBM   4


/* =========================================================
 * Application addressing model (assignment §4)
 *
 * A network address is network_id:node_id, packed as 16 bits:
 *
 *      bits 15..8 = network ID
 *      bits  7..0 = node ID
 *
 * Only ROOT_ADDRESS is fixed at compile time. All other node
 * addresses are assigned at run time by the root (join protocol).
 * ========================================================= */

#define MAKE_ADDRESS(network, node) \
	((uint16_t)((((uint16_t)(network) & 0xFFU) << 8) | \
		    ((uint16_t)(node) & 0xFFU)))

#define ADDRESS_NETWORK(addr) \
	((uint8_t)(((uint16_t)(addr) >> 8) & 0xFFU))

#define ADDRESS_NODE(addr) \
	((uint8_t)((uint16_t)(addr) & 0xFFU))

#define NETWORK_ID_ROOT          1

#define ROOT_ADDRESS             MAKE_ADDRESS(NETWORK_ID_ROOT, 254)
#define BROADCAST_ADDRESS        0xFFFF
#define UNASSIGNED_ADDRESS       0xFFFE
#define RESERVED_ADDRESS         MAKE_ADDRESS(NETWORK_ID_ROOT, 0)

#define NODE_ID_FIRST_ASSIGNABLE 1
#define NODE_ID_LAST_ASSIGNABLE  253

#define ADDRESS_FIRST_ASSIGNABLE \
	MAKE_ADDRESS(NETWORK_ID_ROOT, NODE_ID_FIRST_ASSIGNABLE)
#define ADDRESS_LAST_ASSIGNABLE \
	MAKE_ADDRESS(NETWORK_ID_ROOT, NODE_ID_LAST_ASSIGNABLE)


/* =========================================================
 * Application protocol
 * ========================================================= */

#define PROTOCOL_VERSION         2
#define MESSAGE_TYPE_DATA        1

#define TX_PERIOD_MS             1000

#define RX_QUEUE_DEPTH           8
#define TX_QUEUE_DEPTH           8
#define RX_WAIT_TIMEOUT_SECONDS  5

/*
 * Application RX / TX threads (Zephyr k_thread).
 * RX is higher priority so the receive queue drains first.
 * TX uses a bounded send timeout so a stuck radio cannot
 * stall the TX thread forever.
 */
#define APP_RX_STACK_SIZE        2048
#define APP_TX_STACK_SIZE        2048
#define APP_RX_PRIORITY          5
#define APP_TX_PRIORITY          6
#define TX_SEND_TIMEOUT_MS       100

/*
 * Application message types (assignment §3). Values are
 * contiguous so (type - MSG_TYPE_FIRST) indexes
 * message_handlers[].
 *
 *      1 PROBE
 *      2 HEARTBEAT
 *      3 JOIN_REQUEST
 *      4 JOIN_RESPONSE
 *      5 JOIN_ACKNOWLEDGE
 *      6 DATA
 *      7 NETIDREQ
 *      8 NETIDRESP
 */
#define MSG_TYPE_PROBE              1
#define MSG_TYPE_HEARTBEAT          2
#define MSG_TYPE_JOIN_REQUEST       3
#define MSG_TYPE_JOIN_RESPONSE      4
#define MSG_TYPE_JOIN_ACKNOWLEDGE   5
#define MSG_TYPE_DATA               6
#define MSG_TYPE_NETID_REQUEST      7
#define MSG_TYPE_NETID_RESPONSE     8

#define MSG_TYPE_FIRST              MSG_TYPE_PROBE
#define MSG_TYPE_LAST               MSG_TYPE_NETID_RESPONSE
#define MSG_HANDLER_COUNT           (MSG_TYPE_LAST - MSG_TYPE_FIRST + 1)

/*
 * Sender probe timer:
 *
 * K starts at PROBE_K_INITIAL_SECONDS. Each time the timer
 * expires without a heartbeat for the previous probe, K is
 * incremented by PROBE_N_INCREMENT_SECONDS, capped at
 * PROBE_K_MAX_SECONDS.
 */
#define PROBE_K_INITIAL_SECONDS    2
#define PROBE_N_INCREMENT_SECONDS  2
#define PROBE_K_MAX_SECONDS        30

/*
 * Parent selection + join timers (assignment §4.3–4.6).
 *
 * After the first eligible HEARTBEAT, the joining node waits
 * PARENT_SELECTION_WINDOW_MS for additional replies, then picks
 * a parent and sends JOIN_REQUEST toward the root.
 */
#define PARENT_SELECTION_WINDOW_MS       2000
#define MAX_PARENT_CANDIDATES            8
#define JOIN_RESPONSE_TIMEOUT_SECONDS    5
#define JOIN_REQUEST_MAX_RETRIES         3
#define JOIN_ACK_PENDING_TIMEOUT_SECONDS 10
#define ADDRESS_TABLE_CAPACITY           16
#define JOINING_LOOP_POLL_MS             100

/*
 * Battery level is measured on the nRF52840 VDD rail with the
 * SAADC internal VDD input. This equals the battery voltage
 * only when the battery feeds VDD directly (coin cell, 2xAA);
 * behind a regulator it reports the regulated voltage.
 *
 * Percent is a linear map from EMPTY_MV (0 %) to FULL_MV
 * (100 %). Adjust for the battery chemistry in use.
 */
#define BATTERY_FULL_MV            3000
#define BATTERY_EMPTY_MV           2000
#define BATTERY_PERCENT_UNKNOWN    0xFF

#define BATTERY_ADC_CHANNEL        0
#define BATTERY_ADC_RESOLUTION     12
#define BATTERY_ADC_GAIN           ADC_GAIN_1_6

/* =========================================================
 * Application payload (common OZ header)
 *
 * 17 bytes total — PROTOCOL_VERSION 2
 *
 * Byte  0-1   Signature "OZ"
 * Byte  2     Version
 * Byte  3     Message type
 * Byte  4-5   Source address
 * Byte  6-7   Destination address
 * Byte  8-9   Parent address
 * Byte  10-11 Assigned address
 * Byte  12    Hop count (0xFF = unknown)
 * Byte  13-14 Value / sensor / type-specific int16
 * Byte  15-16 Sequence
 *
 * Type-specific use of shared fields:
 *
 *      PROBE           value    = sender's current K (seconds)
 *                      parent   = UNASSIGNED until joined
 *                      hop      = HOP_COUNT_UNKNOWN until joined
 *      HEARTBEAT       value    = RSSI of the probe at receiver
 *                      sequence = echoes the PROBE sequence
 *                      parent   = responder's parent toward root
 *                      hop      = responder's hop count to root
 *      JOIN_REQUEST    parent   = selected parent
 *                      hop      = proposed child hop (parent+1)
 *                      value    = join nonce (concurrent-joiner id)
 *      JOIN_RESPONSE   assigned = address granted by root
 *                      parent   = parent the joiner should use
 *                      hop      = hop count for the new child
 *      JOIN_ACKNOWLEDGE assigned = address the joiner accepted
 *                      parent   = parent the joiner installed
 *      DATA            value    = sensor reading
 *                      hop      = hops remaining / sender hop
 *
 * Multi-byte application fields are serialized explicitly
 * as little endian. Message bodies (e.g. HEARTBEAT GUID /
 * battery) still append after this header.
 * ========================================================= */

#define OZ_PACKET_SIZE   17


struct oz_packet {

	uint8_t signature[2];

	uint8_t version;
	uint8_t message_type;

	uint16_t source_id;
	uint16_t destination_id;

	uint16_t parent_id;
	uint16_t assigned_id;
	uint8_t hop_count;

	int16_t value;

	uint16_t sequence;
};


/* =========================================================
 * Message bodies
 *
 * Some message types append a body after the common OZ
 * header. The header is always parsed first; the handler
 * for the message type then parses its own body.
 *
 * HEARTBEAT body, 13 bytes (frame bytes 17-29):
 *
 * Body  0-7   GUID (64-bit factory device ID, hwinfo order)
 * Body  8-9   Battery voltage, millivolts
 * Body  10    Battery level, percent (0xFF = unknown)
 * Body  11-12 Heartbeat sequence (receiver's own counter)
 *
 * The OZ header of a HEARTBEAT still carries:
 *
 *      value    = RSSI of the probe at the receiver (dBm)
 *      sequence = echoes the PROBE sequence
 *      parent   = responder parent address
 *      hop      = responder hop count to root
 * ========================================================= */

#define GUID_SIZE                8
#define OZ_MAX_BODY_SIZE         64

#define HEARTBEAT_BODY_SIZE      13
#define HEARTBEAT_FRAME_SIZE     (OZ_PACKET_SIZE + HEARTBEAT_BODY_SIZE)


struct heartbeat_body {

	uint8_t guid[GUID_SIZE];

	uint16_t battery_mv;
	uint8_t battery_percent;

	uint16_t heartbeat_sequence;
};


/* =========================================================
 * Node finite-state machine (assignment §5)
 *
 * UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
 *
 * Root boots REGISTERED. Joining nodes boot UNREGISTERED and
 * enter PROBING once the radio is up (see node_fsm_init).
 * ========================================================= */

enum node_state {
	NODE_UNREGISTERED = 0,
	NODE_PROBING,
	NODE_WAITING_FOR_JOIN_RESPONSE,
	NODE_REGISTERED,
};

/* Hop count used when the node has no route toward the root. */
#define HOP_COUNT_UNKNOWN        0xFF


/* =========================================================
 * Role → boot address and boot state
 *
 * Root discovery is assumed complete: root starts at 1:254
 * already REGISTERED. Every other node boots unassigned and
 * UNREGISTERED until join succeeds.
 * ========================================================= */

#if NODE_ROLE == ROLE_ROOT

#define BOOT_ADDRESS         ROOT_ADDRESS
#define BOOT_NODE_STATE      NODE_REGISTERED
#define BOOT_PARENT_ADDRESS  ROOT_ADDRESS
#define BOOT_HOP_COUNT       0

#elif NODE_ROLE == ROLE_JOINING

#define BOOT_ADDRESS         UNASSIGNED_ADDRESS
#define BOOT_NODE_STATE      NODE_UNREGISTERED
#define BOOT_PARENT_ADDRESS  UNASSIGNED_ADDRESS
#define BOOT_HOP_COUNT       HOP_COUNT_UNKNOWN

#else

#error "Invalid NODE_ROLE: use ROLE_ROOT or ROLE_JOINING"

#endif


/* =========================================================
 * Globals
 * ========================================================= */

static struct net_if *iface;

/*
 * Application / MAC short address for this node.
 * Root: fixed ROOT_ADDRESS for the life of the node.
 * Joining: starts UNASSIGNED_ADDRESS; set_local_address()
 * will install the join-assigned value and push it to L2.
 */
static uint16_t local_address = BOOT_ADDRESS;

/*
 * Protocol FSM state and registration context.
 * parent_address / hop_count are meaningful once REGISTERED
 * (and while WAITING_FOR_JOIN_RESPONSE after a parent is chosen).
 */
static enum node_state current_state = BOOT_NODE_STATE;
static uint16_t parent_address = BOOT_PARENT_ADDRESS;
static uint8_t hop_count = BOOT_HOP_COUNT;

static int packet_socket = -1;

/*
 * Both roles now receive (the sender listens for heartbeats),
 * so the receiver-only guard is disabled.
 *
 * receive_context is also used for TX (TX thread only;
 * see radio_send_buffer / send_oz_packet).
 * Every bound AF_PACKET endpoint gets its own clone of each
 * received frame, so a second, never-read endpoint such as
 * packet_socket would slowly exhaust the network RX buffers.
 */
// #if NODE_ROLE == ROLE_RECEIVER
static struct net_context *receive_context;

struct received_frame {
	uint8_t data[IEEE802154_MTU];
	size_t length;
	uint16_t mac_source;
	int16_t rssi_dbm;
};

/*
 * Serialized TX job for the TX thread. The submitting thread
 * (RX/protocol) copies the full payload here so the TX thread
 * never reads shared oz_packet / FSM state.
 */
struct tx_request {
	uint16_t mac_destination;
	uint16_t length;
	uint8_t data[OZ_PACKET_SIZE + OZ_MAX_BODY_SIZE];
};

K_MSGQ_DEFINE(receive_queue,
	      sizeof(struct received_frame),
	      RX_QUEUE_DEPTH,
	      4);

K_MSGQ_DEFINE(tx_queue,
	      sizeof(struct tx_request),
	      TX_QUEUE_DEPTH,
	      4);

/*
 * Static + app_-prefixed: Zephyr's net_tc.c already exports
 * global rx_stack / tx_stack symbols.
 */
static K_THREAD_STACK_DEFINE(app_rx_stack, APP_RX_STACK_SIZE);
static K_THREAD_STACK_DEFINE(app_tx_stack, APP_TX_STACK_SIZE);
static struct k_thread app_rx_thread;
static struct k_thread app_tx_thread;
// #endif


/*
 * Used by sender only.
 */
static uint16_t tx_sequence = 1;


/*
 * Probe state. The joining node originates probes until parent
 * selection starts JOIN_REQUEST. Owned by the RX/protocol
 * thread, which also runs the message handlers, so no locking
 * is needed between protocol and handlers.
 */
static unsigned int probe_k_seconds = PROBE_K_INITIAL_SECONDS;
static uint16_t probe_outstanding_sequence;
static bool probe_outstanding;

#if NODE_ROLE == ROLE_JOINING
K_TIMER_DEFINE(probe_timer, NULL, NULL);
K_TIMER_DEFINE(selection_timer, NULL, NULL);
K_TIMER_DEFINE(join_timer, NULL, NULL);

/*
 * HEARTBEAT candidates collected during the selection window.
 * rssi_dbm is how well we hear the responder (RX frame RSSI).
 */
struct parent_candidate {
	uint16_t address;
	uint16_t parent_id;
	uint8_t hop_count;
	int16_t rssi_dbm;
	bool valid;
};

static struct parent_candidate parent_candidates[MAX_PARENT_CANDIDATES];
static size_t parent_candidate_count;
static bool selection_window_active;

/* Outstanding JOIN_REQUEST while WAITING_FOR_JOIN_RESPONSE. */
static uint16_t join_request_sequence;
static uint16_t join_request_nonce;
static uint8_t join_request_retries;
static bool join_request_outstanding;
static uint16_t selected_parent_address;
static uint8_t selected_parent_hop;

/*
 * Post-registration DATA collection (§4.7).
 * Sensor value is a sequential counter sent every TX_PERIOD_MS.
 */
K_TIMER_DEFINE(data_timer, NULL, NULL);
static int16_t sensor_value = 1;
static uint16_t data_tx_sequence = 1;
static bool data_collection_active;
#endif

#if NODE_ROLE == ROLE_ROOT
/*
 * Root address-allocation table (assignment §5 / §4.5–4.6).
 * Pending rows wait for JOIN_ACKNOWLEDGE; active rows are live.
 *
 * Joiners share IEEE short address 0xFFFE before registration, so
 * duplicate detection keys on join_nonce (JOIN_REQUEST.value) rather
 * than MAC alone.
 */
struct address_record {
	uint16_t address;
	uint16_t parent;
	uint8_t hop_count;
	uint16_t request_sequence;
	uint16_t join_nonce;
	uint16_t requester_mac;
	bool pending;
	bool active;
	bool in_use;
	int64_t pending_deadline_ms;
};

static struct address_record address_table[ADDRESS_TABLE_CAPACITY];

/*
 * Per-originator DATA collection stats on the root (§4.7).
 * Keyed by the original child's source_id (preserved across forwards).
 */
struct data_collector_entry {
	uint16_t source;
	uint16_t last_sequence;
	bool sequence_valid;
	uint32_t received;
	uint32_t lost;
	int16_t last_value;
	bool in_use;
};

static struct data_collector_entry data_collectors[ADDRESS_TABLE_CAPACITY];
#endif


/*
 * Heartbeat state: this node's GUID, the battery ADC, and the
 * count of heartbeats this node has sent.
 */
static uint8_t device_guid[GUID_SIZE];

static const struct device *const battery_adc =
	DEVICE_DT_GET(DT_NODELABEL(adc));

static bool battery_adc_ready;

static uint16_t heartbeat_tx_sequence = 1;


/*
 * Link statistics, owned by the RX/protocol thread like the probe
 * state. Loss is detected two ways:
 *   - sender: a probe whose timer K expired unanswered
 *     (round-trip loss, either direction)
 *   - sequence gaps: the receiver watches PROBE sequences and
 *     the sender watches heartbeat sequences, so each side
 *     sees the loss on its own incoming direction.
 */
struct link_stats {
	uint32_t probes_sent;
	uint32_t probes_tx_failed;
	uint32_t probes_missed;
	uint32_t probes_received;
	uint32_t probes_lost;
	uint32_t heartbeats_sent;
	uint32_t heartbeats_tx_failed;
	uint32_t heartbeats_accepted;
	uint32_t heartbeats_received;
	uint32_t heartbeats_lost;
	uint32_t heartbeats_stale;
};

static struct link_stats link_stats;

static uint16_t last_probe_rx_sequence;
static bool probe_rx_sequence_valid;

static uint16_t last_heartbeat_rx_sequence;
static bool heartbeat_rx_sequence_valid;


/* =========================================================
 * Addressing helpers
 * ========================================================= */

static bool address_is_broadcast(uint16_t addr)
{
	return addr == BROADCAST_ADDRESS;
}

static bool address_is_unassigned(uint16_t addr)
{
	return addr == UNASSIGNED_ADDRESS;
}

static bool address_is_root(uint16_t addr)
{
	return addr == ROOT_ADDRESS;
}

static bool address_is_reserved(uint16_t addr)
{
	return (addr == RESERVED_ADDRESS) ||
	       address_is_broadcast(addr) ||
	       address_is_unassigned(addr) ||
	       address_is_root(addr);
}

/*
 * True for addresses the root may hand out: 1:1 .. 1:253.
 */
static bool address_is_assignable(uint16_t addr)
{
	uint8_t network = ADDRESS_NETWORK(addr);
	uint8_t node = ADDRESS_NODE(addr);

	if (network != NETWORK_ID_ROOT) {
		return false;
	}

	if ((node < NODE_ID_FIRST_ASSIGNABLE) ||
	    (node > NODE_ID_LAST_ASSIGNABLE)) {
		return false;
	}

	return true;
}

/*
 * True if this node should accept an application-layer frame.
 * Unicast to our address, or broadcast discovery traffic.
 */
static bool address_is_for_me(uint16_t destination)
{
	return (destination == local_address) ||
	       address_is_broadcast(destination);
}

/*
 * Push the current local_address into the IEEE 802.15.4 L2
 * short-address register. Used at boot and again when join
 * assigns a new address (platform requirement).
 *
 * Zephyr treats (PAN ID set + short != 0xFFFF) as associated.
 * Booting a joiner at 0xFFFE with a PAN ID already counts as
 * associated, so a later SET_SHORT_ADDR(1:n) returns -EBUSY.
 * The only allowed change while associated is SET_SHORT_ADDR to
 * IEEE802154_SHORT_ADDRESS_NOT_ASSOCIATED (0xFFFF), which clears
 * both short address and PAN ID. We then restore the PAN and set
 * the new short address (see ieee802154_mgmt.c).
 */
static int ieee802154_apply_short_address(uint16_t addr)
{
	uint16_t value = addr;
	uint16_t disassociate = IEEE802154_SHORT_ADDRESS_NOT_ASSOCIATED;
	uint16_t pan_id = IEEE802154_PAN_ID;
	int ret;

	if (iface == NULL) {
		return -ENODEV;
	}

	ret = net_mgmt(
		NET_REQUEST_IEEE802154_SET_SHORT_ADDR,
		iface,
		&value,
		sizeof(value)
	);

	if ((ret == -EBUSY) || (ret == EBUSY)) {
		LOG_WRN(
			"SET_SHORT_ADDR (0x%04X) busy; disassociate then retry",
			(unsigned int)addr
		);

		/*
		 * Disassociate: only this short-addr value is accepted
		 * while associated; it also clears the PAN ID.
		 */
		ret = net_mgmt(
			NET_REQUEST_IEEE802154_SET_SHORT_ADDR,
			iface,
			&disassociate,
			sizeof(disassociate)
		);
		if (ret) {
			LOG_ERR(
				"Disassociate SET_SHORT_ADDR (0x%04X) failed: %d",
				(unsigned int)disassociate,
				ret
			);
			return ret;
		}

		ret = net_mgmt(
			NET_REQUEST_IEEE802154_SET_PAN_ID,
			iface,
			&pan_id,
			sizeof(pan_id)
		);
		if (ret) {
			LOG_ERR(
				"SET_PAN_ID (0x%04X) after disassociate failed: %d",
				(unsigned int)pan_id,
				ret
			);
			return ret;
		}

		ret = net_mgmt(
			NET_REQUEST_IEEE802154_SET_SHORT_ADDR,
			iface,
			&value,
			sizeof(value)
		);
	}

	if (ret) {
		LOG_ERR(
			"SET_SHORT_ADDR (0x%04X) failed: %d",
			(unsigned int)addr,
			ret
		);
		return ret;
	}

	LOG_INF(
		"Short Addr  = 0x%04X (%u:%u)",
		(unsigned int)addr,
		(unsigned int)ADDRESS_NETWORK(addr),
		(unsigned int)ADDRESS_NODE(addr)
	);

	return 0;
}

/*
 * Choose a MAC destination for a peer that is still unassigned.
 *
 * IEEE 802.15.4 treats 0xFFFE as "no short address": radios
 * configured with that short address typically do not accept
 * unicasts to 0xFFFE. LL source recovery may also report 0xFFFF
 * when the stack omits a short source. In both cases broadcast
 * is the reliable way to reach pre-registration joiners.
 */
static uint16_t mac_dest_for_unassigned_peer(uint16_t mac_source)
{
	if (address_is_broadcast(mac_source) ||
	    address_is_unassigned(mac_source) ||
	    (mac_source == 0U)) {
		return BROADCAST_ADDRESS;
	}

	return mac_source;
}

#if NODE_ROLE == ROLE_JOINING
/*
 * Install a new application address and mirror it to the radio.
 * Returns 0 on success. On L2 failure, local_address is left
 * unchanged.
 *
 * Called by node_enter_registered() / node_revert_to_unregistered()
 * when join assigns or revokes an address.
 */
static int set_local_address(uint16_t addr)
{
	uint16_t previous = local_address;
	int ret;

	local_address = addr;

	ret = ieee802154_apply_short_address(addr);
	if (ret) {
		local_address = previous;
		return ret;
	}

	LOG_INF(
		"Local address set to %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(addr),
		(unsigned int)ADDRESS_NODE(addr),
		(unsigned int)addr
	);

	return 0;
}
#endif


/* =========================================================
 * Node FSM helpers
 * ========================================================= */

static const char *node_state_name(enum node_state state)
{
	switch (state) {
	case NODE_UNREGISTERED:
		return "UNREGISTERED";
	case NODE_PROBING:
		return "PROBING";
	case NODE_WAITING_FOR_JOIN_RESPONSE:
		return "WAITING_FOR_JOIN_RESPONSE";
	case NODE_REGISTERED:
		return "REGISTERED";
	default:
		return "UNKNOWN";
	}
}

static void log_node_state(const char *title)
{
	LOG_INF(
		"%s state=%s addr=%u:%u parent=%u:%u hop=%u",
		title,
		node_state_name(current_state),
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)ADDRESS_NETWORK(parent_address),
		(unsigned int)ADDRESS_NODE(parent_address),
		(unsigned int)hop_count
	);
}

/*
 * Central transition point. Logs every change so serial demos
 * can show the FSM progression required by the assignment.
 */
static void set_node_state(enum node_state next)
{
	if (current_state == next) {
		return;
	}

	LOG_INF(
		"STATE %s -> %s",
		node_state_name(current_state),
		node_state_name(next)
	);

	current_state = next;
	log_node_state("FSM");
}

/* Normal DATA only after registration is complete. */
static bool node_can_send_data(void)
{
	return (current_state == NODE_REGISTERED) &&
	       !address_is_unassigned(local_address);
}

/* Discovery probes are owned by the PROBING state. */
static bool node_can_send_probe(void)
{
	return current_state == NODE_PROBING;
}

/*
 * Only registered nodes (root today; later also joined routers)
 * answer probes with HEARTBEAT.
 */
static bool node_can_reply_heartbeat(void)
{
	return current_state == NODE_REGISTERED;
}

#if NODE_ROLE == ROLE_JOINING
static void clear_parent_candidates(void)
{
	memset(parent_candidates, 0, sizeof(parent_candidates));
	parent_candidate_count = 0;
	selection_window_active = false;
	k_timer_stop(&selection_timer);
}

static void clear_join_request_state(void)
{
	join_request_outstanding = false;
	join_request_retries = 0;
	join_request_sequence = 0;
	join_request_nonce = 0;
	selected_parent_address = UNASSIGNED_ADDRESS;
	selected_parent_hop = HOP_COUNT_UNKNOWN;
	k_timer_stop(&join_timer);
}

static void stop_data_collection(void)
{
	data_collection_active = false;
	k_timer_stop(&data_timer);
}

static void start_data_collection(void)
{
	if (data_collection_active) {
		return;
	}

	sensor_value = 1;
	data_tx_sequence = 1;
	data_collection_active = true;
	k_timer_start(&data_timer, K_MSEC(TX_PERIOD_MS), K_NO_WAIT);

	LOG_INF(
		"DATA collection started (period=%u ms, value starts at 1)",
		(unsigned int)TX_PERIOD_MS
	);
}

/*
 * UNREGISTERED → PROBING. Clears any half-finished join context
 * and starts listening/sending discovery traffic.
 */
static void node_enter_probing(void)
{
	parent_address = UNASSIGNED_ADDRESS;
	hop_count = HOP_COUNT_UNKNOWN;
	probe_outstanding = false;
	probe_outstanding_sequence = 0;
	probe_k_seconds = PROBE_K_INITIAL_SECONDS;

	clear_parent_candidates();
	clear_join_request_state();
	stop_data_collection();

	set_node_state(NODE_PROBING);
}

/*
 * Join failure / give-up path: drop parent, restore unassigned
 * address context, and return to UNREGISTERED. The caller may
 * immediately call node_enter_probing() to retry.
 */
static void node_revert_to_unregistered(void)
{
	parent_address = UNASSIGNED_ADDRESS;
	hop_count = HOP_COUNT_UNKNOWN;
	probe_outstanding = false;
	probe_outstanding_sequence = 0;

	clear_parent_candidates();
	clear_join_request_state();
	stop_data_collection();
	k_timer_stop(&probe_timer);

	if (!address_is_unassigned(local_address)) {
		/*
		 * A previously assigned joining node that lost its
		 * place must fall back to the unassigned short
		 * address before probing again.
		 */
		(void)set_local_address(UNASSIGNED_ADDRESS);
	}

	set_node_state(NODE_UNREGISTERED);
	log_node_state("Join failed; reverted");
}

/*
 * PROBING → WAITING_FOR_JOIN_RESPONSE after parent selection.
 */
static void node_enter_waiting_for_join_response(
	uint16_t selected_parent,
	uint8_t parent_hop_count)
{
	parent_address = selected_parent;
	/*
	 * Child hop count will be parent_hop + 1 once the response
	 * confirms membership; stash provisional hop for now.
	 */
	hop_count = (parent_hop_count == HOP_COUNT_UNKNOWN) ?
		    HOP_COUNT_UNKNOWN :
		    (uint8_t)(parent_hop_count + 1U);

	selected_parent_address = selected_parent;
	selected_parent_hop = parent_hop_count;
	probe_outstanding = false;
	k_timer_stop(&probe_timer);
	clear_parent_candidates();

	set_node_state(NODE_WAITING_FOR_JOIN_RESPONSE);
}

/*
 * WAITING_FOR_JOIN_RESPONSE → REGISTERED after JOIN_RESPONSE
 * + local short-address update. JOIN_ACKNOWLEDGE is sent by the caller.
 */
static void node_enter_registered(
	uint16_t assigned_address,
	uint16_t assigned_parent,
	uint8_t assigned_hop_count)
{
	int ret;

	ret = set_local_address(assigned_address);
	if (ret) {
		LOG_ERR(
			"Registration aborted: short address update failed (%d)",
			ret
		);
		node_revert_to_unregistered();
		return;
	}

	parent_address = assigned_parent;
	hop_count = assigned_hop_count;

	clear_join_request_state();
	k_timer_stop(&probe_timer);

	set_node_state(NODE_REGISTERED);
	log_node_state("Registered");

	start_data_collection();
}
#endif /* ROLE_JOINING */

/*
 * Apply boot role to the FSM after the radio is up.
 * Root stays REGISTERED. Joining nodes move UNREGISTERED → PROBING.
 */
static void node_fsm_init(void)
{
	current_state = BOOT_NODE_STATE;
	parent_address = BOOT_PARENT_ADDRESS;
	hop_count = BOOT_HOP_COUNT;

	log_node_state("FSM boot");

#if NODE_ROLE == ROLE_JOINING
	if (current_state == NODE_UNREGISTERED) {
		node_enter_probing();
	}
#endif
}


/* =========================================================
 * Role name
 * ========================================================= */

static const char *role_name(void)
{
#if NODE_ROLE == ROLE_ROOT

	return "ROOT";

#elif NODE_ROLE == ROLE_JOINING

	return "JOINING";

#else

	return "UNKNOWN";

#endif
}


/* =========================================================
 * Message type name
 * ========================================================= */

static const char *message_type_name(uint8_t type)
{
	switch (type) {
	case MSG_TYPE_PROBE:
		return "PROBE";
	case MSG_TYPE_HEARTBEAT:
		return "HEARTBEAT";
	case MSG_TYPE_JOIN_REQUEST:
		return "JOIN_REQUEST";
	case MSG_TYPE_JOIN_RESPONSE:
		return "JOIN_RESPONSE";
	case MSG_TYPE_JOIN_ACKNOWLEDGE:
		return "JOIN_ACKNOWLEDGE";
	case MSG_TYPE_DATA:
		return "DATA";
	case MSG_TYPE_NETID_REQUEST:
		return "NETID_REQUEST";
	case MSG_TYPE_NETID_RESPONSE:
		return "NETID_RESPONSE";
	default:
		return "UNKNOWN";
	}
}


/* =========================================================
 * Print application packet
 * ========================================================= */

static void print_packet(
	const char *title,
	const struct oz_packet *packet)
{
	unsigned int source =
		packet->source_id;

	unsigned int destination =
		packet->destination_id;

	unsigned int sequence =
		packet->sequence;

	unsigned int version =
		packet->version;

	unsigned int type =
		packet->message_type;

	int value =
		packet->value;


	LOG_INF("--------------------------------");

	LOG_INF("%s", title);

	LOG_INF(
		"Version       = %u",
		version
	);

	LOG_INF(
		"Message Type  = %u",
		type
	);

	LOG_INF(
		"Message Name  = %s",
		message_type_name(packet->message_type)
	);

	LOG_INF(
		"Source        = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(source),
		(unsigned int)ADDRESS_NODE(source),
		source
	);

	LOG_INF(
		"Destination   = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(destination),
		(unsigned int)ADDRESS_NODE(destination),
		destination
	);

	LOG_INF(
		"Parent        = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(packet->parent_id),
		(unsigned int)ADDRESS_NODE(packet->parent_id),
		(unsigned int)packet->parent_id
	);

	LOG_INF(
		"Assigned      = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(packet->assigned_id),
		(unsigned int)ADDRESS_NODE(packet->assigned_id),
		(unsigned int)packet->assigned_id
	);

	if (packet->hop_count == HOP_COUNT_UNKNOWN) {
		LOG_INF("Hop Count     = unknown");
	} else {
		LOG_INF(
			"Hop Count     = %u",
			(unsigned int)packet->hop_count
		);
	}

	LOG_INF(
		"Value         = %d",
		value
	);

	LOG_INF(
		"Sequence      = %u",
		sequence
	);

	LOG_INF("--------------------------------");
}


/* =========================================================
 * Serialize OZ application packet
 * ========================================================= */

static void serialize_packet(
	const struct oz_packet *packet,
	uint8_t buffer[OZ_PACKET_SIZE])
{
	buffer[0] =
		packet->signature[0];

	buffer[1] =
		packet->signature[1];

	buffer[2] =
		packet->version;

	buffer[3] =
		packet->message_type;


	sys_put_le16(
		packet->source_id,
		&buffer[4]
	);


	sys_put_le16(
		packet->destination_id,
		&buffer[6]
	);


	sys_put_le16(
		packet->parent_id,
		&buffer[8]
	);


	sys_put_le16(
		packet->assigned_id,
		&buffer[10]
	);


	buffer[12] =
		packet->hop_count;


	sys_put_le16(
		(uint16_t)packet->value,
		&buffer[13]
	);


	sys_put_le16(
		packet->sequence,
		&buffer[15]
	);
}


/* =========================================================
 * Deserialize OZ application packet
 * ========================================================= */

static bool deserialize_packet(
	const uint8_t *buffer,
	size_t length,
	struct oz_packet *packet)
{
	/* -----------------------------------------------------
	 * Common header must be present; trailing bytes are
	 * an optional type-specific body (e.g. HEARTBEAT).
	 * ----------------------------------------------------- */

	if (length < OZ_PACKET_SIZE) {

		return false;
	}


	/* -----------------------------------------------------
	 * Signature
	 * ----------------------------------------------------- */

	if ((buffer[0] != 'O') ||
	    (buffer[1] != 'Z')) {

		return false;
	}


	packet->signature[0] =
		buffer[0];

	packet->signature[1] =
		buffer[1];


	packet->version =
		buffer[2];

	packet->message_type =
		buffer[3];


	packet->source_id =
		sys_get_le16(
			&buffer[4]
		);


	packet->destination_id =
		sys_get_le16(
			&buffer[6]
		);


	packet->parent_id =
		sys_get_le16(
			&buffer[8]
		);


	packet->assigned_id =
		sys_get_le16(
			&buffer[10]
		);


	packet->hop_count =
		buffer[12];


	packet->value =
		(int16_t)sys_get_le16(
			&buffer[13]
		);


	packet->sequence =
		sys_get_le16(
			&buffer[15]
		);


	/* -----------------------------------------------------
	 * Version validation
	 * ----------------------------------------------------- */

	if (packet->version !=
	    PROTOCOL_VERSION) {

		return false;
	}


	return true;
}


/* =========================================================
 * Serialize / deserialize HEARTBEAT body
 *
 * Same explicit little-endian layout rules as the OZ header.
 * ========================================================= */

static void serialize_heartbeat_body(
	const struct heartbeat_body *body,
	uint8_t buffer[HEARTBEAT_BODY_SIZE])
{
	memcpy(&buffer[0], body->guid, GUID_SIZE);

	sys_put_le16(body->battery_mv, &buffer[8]);

	buffer[10] = body->battery_percent;

	sys_put_le16(body->heartbeat_sequence, &buffer[11]);
}


static bool deserialize_heartbeat_body(
	const uint8_t *buffer,
	size_t length,
	struct heartbeat_body *body)
{
	if (length != HEARTBEAT_BODY_SIZE) {
		return false;
	}

	memcpy(body->guid, &buffer[0], GUID_SIZE);

	body->battery_mv = sys_get_le16(&buffer[8]);

	body->battery_percent = buffer[10];

	body->heartbeat_sequence = sys_get_le16(&buffer[11]);

	return true;
}


/* =========================================================
 * Device GUID
 *
 * 64-bit factory-programmed device ID (nRF52840 FICR
 * DEVICEID), read once at boot through the hwinfo driver.
 * Left as all zeros if it cannot be read.
 * ========================================================= */

static void init_device_guid(void)
{
	ssize_t length;

	memset(device_guid, 0, sizeof(device_guid));

	length = hwinfo_get_device_id(device_guid, sizeof(device_guid));

	if (length < 0) {
		LOG_WRN("hwinfo_get_device_id() failed: %d; GUID = 0", (int)length);
		return;
	}

	LOG_INF(
		"GUID       : %02X%02X%02X%02X%02X%02X%02X%02X",
		device_guid[0], device_guid[1], device_guid[2], device_guid[3],
		device_guid[4], device_guid[5], device_guid[6], device_guid[7]
	);
}


/* =========================================================
 * Battery level (SAADC, internal VDD input)
 * ========================================================= */

static int init_battery_adc(void)
{
	struct adc_channel_cfg channel_cfg = {
		.gain = BATTERY_ADC_GAIN,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
		.channel_id = BATTERY_ADC_CHANNEL,
		.input_positive = NRF_SAADC_VDD,
	};
	int ret;

	if (!device_is_ready(battery_adc)) {
		LOG_WRN("Battery ADC not ready; battery level unknown");
		return -ENODEV;
	}

	ret = adc_channel_setup(battery_adc, &channel_cfg);
	if (ret) {
		LOG_WRN("adc_channel_setup() failed: %d; battery level unknown", ret);
		return ret;
	}

	battery_adc_ready = true;
	return 0;
}


/*
 * On any failure: millivolts = 0, percent = BATTERY_PERCENT_UNKNOWN.
 */
static void read_battery(uint16_t *millivolts, uint8_t *percent)
{
	int16_t sample;
	int32_t value_mv;
	int ret;

	struct adc_sequence sequence = {
		.channels = BIT(BATTERY_ADC_CHANNEL),
		.buffer = &sample,
		.buffer_size = sizeof(sample),
		.resolution = BATTERY_ADC_RESOLUTION,
	};

	*millivolts = 0;
	*percent = BATTERY_PERCENT_UNKNOWN;

	if (!battery_adc_ready) {
		return;
	}

	ret = adc_read(battery_adc, &sequence);
	if (ret) {
		LOG_WRN("adc_read() failed: %d", ret);
		return;
	}

	value_mv = sample;

	ret = adc_raw_to_millivolts(
		adc_ref_internal(battery_adc),
		BATTERY_ADC_GAIN,
		BATTERY_ADC_RESOLUTION,
		&value_mv
	);
	if (ret) {
		LOG_WRN("adc_raw_to_millivolts() failed: %d", ret);
		return;
	}

	if (value_mv < 0) {
		value_mv = 0;
	}

	*millivolts = (uint16_t)value_mv;

	if (value_mv >= BATTERY_FULL_MV) {
		*percent = 100;
	} else if (value_mv <= BATTERY_EMPTY_MV) {
		*percent = 0;
	} else {
		*percent = (uint8_t)(((value_mv - BATTERY_EMPTY_MV) * 100) /
				     (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
	}
}


/* =========================================================
 * Configure IEEE 802.15.4
 * ========================================================= */

static int configure_ieee802154(void)
{
    uint16_t value;
    int16_t tx_power_dbm;
    int ret;

    iface = net_if_get_default();

    if (iface == NULL) {
        LOG_ERR("No default network interface");
        return -ENODEV;
    }

    LOG_INF("Network interface found");

    if (net_if_is_up(iface)) {
        net_if_down(iface);
    }

    /* Channel */
    value = IEEE802154_CHANNEL;

    ret = net_mgmt(
        NET_REQUEST_IEEE802154_SET_CHANNEL,
        iface,
        &value,
        sizeof(value)
    );

    if (ret) {
        LOG_ERR("SET_CHANNEL failed: %d", ret);
        return ret;
    }

    LOG_INF("Channel     = %u", IEEE802154_CHANNEL);

    /*
     * Transmit power must be set before PAN ID + short address:
     * once both are set the L2 treats the node as associated and
     * rejects any further parameter change with -EBUSY.
     */
    tx_power_dbm = IEEE802154_TX_POWER_DBM;

    ret = net_mgmt(
        NET_REQUEST_IEEE802154_SET_TX_POWER,
        iface,
        &tx_power_dbm,
        sizeof(tx_power_dbm)
    );

    if (ret) {
        LOG_ERR(
            "SET_TX_POWER (%d dBm) failed: %d",
            IEEE802154_TX_POWER_DBM,
            ret
        );
        return ret;
    }

    LOG_INF("TX Power    = %d dBm", IEEE802154_TX_POWER_DBM);

    /* PAN ID */
    value = IEEE802154_PAN_ID;

    ret = net_mgmt(
        NET_REQUEST_IEEE802154_SET_PAN_ID,
        iface,
        &value,
        sizeof(value)
    );

    if (ret) {
        LOG_ERR("SET_PAN_ID failed: %d", ret);
        return ret;
    }

    LOG_INF("PAN ID      = 0x%04X", IEEE802154_PAN_ID);

    /* Short address: root = 1:254, joining = unassigned. */
    ret = ieee802154_apply_short_address(local_address);
    if (ret) {
        return ret;
    }

    /*
     * Original position of the transmit-power step, moved above
     * PAN ID (returned -EBUSY here, see comment there).
     *
    tx_power_dbm = IEEE802154_TX_POWER_DBM;

    ret = net_mgmt(
        NET_REQUEST_IEEE802154_SET_TX_POWER,
        iface,
        &tx_power_dbm,
        sizeof(tx_power_dbm)
    );

    if (ret) {
        LOG_ERR(
            "SET_TX_POWER (%d dBm) failed: %d",
            IEEE802154_TX_POWER_DBM,
            ret
        );
        return ret;
    }

    LOG_INF("TX Power    = %d dBm", IEEE802154_TX_POWER_DBM);
     */

	printk("Calling net_if_up()...\n");


	ret = net_if_up(iface);


	printk("net_if_up() returned: %d\n", ret);

	if (ret) {
    	LOG_ERR("net_if_up failed: %d", ret);
    return ret;
	}

	LOG_INF("IEEE 802.15.4 interface UP");

    return 0;
}

/* =========================================================
 * Create AF_PACKET / SOCK_DGRAM socket
 *
 * Important:
 *
 * SOCK_DGRAM means the application supplies only the
 * application payload.
 *
 * Zephyr's IEEE 802.15.4 L2 generates the MAC header.
 *
 * Currently unused: TX goes through receive_context so each
 * node has a single AF_PACKET endpoint. Kept for future use.
 * ========================================================= */

static __maybe_unused int create_packet_socket(void)
{
	struct sockaddr_ll local_addr;

	int ret;


	/* -----------------------------------------------------
	 * Create IEEE 802.15.4 packet socket
	 * ----------------------------------------------------- */

	packet_socket =
		zsock_socket(
			AF_PACKET,
			SOCK_DGRAM,
			htons(ETH_P_IEEE802154)
		);


	if (packet_socket < 0) {

		LOG_ERR(
			"socket() failed, errno=%d",
			errno
		);

		return -errno;
	}


	LOG_INF(
		"AF_PACKET SOCK_DGRAM created"
	);


	/* -----------------------------------------------------
	 * Bind socket to our IEEE 802.15.4 interface
	 * ----------------------------------------------------- */

	memset(
		&local_addr,
		0,
		sizeof(local_addr)
	);


	local_addr.sll_family =
		AF_PACKET;


	local_addr.sll_protocol =
		htons(ETH_P_IEEE802154);


	local_addr.sll_ifindex =
		net_if_get_by_iface(iface);


	ret =
		zsock_bind(
			packet_socket,
			(const struct sockaddr *)&local_addr,
			sizeof(local_addr)
		);


	if (ret < 0) {

		LOG_ERR(
			"bind() failed, errno=%d",
			errno
		);

		zsock_close(
			packet_socket
		);

		packet_socket = -1;

		return -errno;
	}


	LOG_INF(
		"Packet socket bound to interface %d",
		local_addr.sll_ifindex
	);


	return 0;
}


/* =========================================================
 * Radio TX (TX thread only)
 *
 * Sole caller of net_context_sendto(). Bounded timeout keeps
 * the TX thread responsive if buffers are exhausted.
 * ========================================================= */

static int radio_send_buffer(
	const uint8_t *buffer,
	size_t length,
	uint16_t mac_destination)
{
	struct sockaddr_ll destination;
	int sent;

	memset(&destination, 0, sizeof(destination));
	destination.sll_family = AF_PACKET;
	destination.sll_protocol = htons(ETH_P_IEEE802154);
	destination.sll_ifindex = net_if_get_by_iface(iface);
	destination.sll_halen = IEEE802154_SHORT_ADDR_LENGTH;
	sys_put_be16(mac_destination, destination.sll_addr);

	printk("TX RAW: ");
	for (size_t i = 0; i < length; i++) {
		printk("%02X ", buffer[i]);
	}
	printk("\n");

	sent = net_context_sendto(
		receive_context,
		buffer,
		length,
		(const struct sockaddr *)&destination,
		sizeof(destination),
		NULL,
		K_MSEC(TX_SEND_TIMEOUT_MS),
		NULL
	);

	LOG_INF("sendto() returned %d", sent);

	if (sent < 0) {
		LOG_ERR("net_context_sendto() failed: %d", sent);
		return sent;
	}

	if (sent != (int)length) {
		LOG_ERR("Short TX: %d of %d bytes", sent, (int)length);
		return -EIO;
	}

	LOG_INF(
		"MAC TX -> 0x%04X, payload=%d bytes",
		mac_destination,
		sent
	);

	return 0;
}


/*
 * Queue a serialized payload for the TX thread.
 * Safe to call from the RX/protocol thread (and only that
 * thread submits today). Returns 0 if queued, -ENOBUFS if
 * the TX queue is full.
 */
static int tx_queue_submit(
	const uint8_t *buffer,
	size_t length,
	uint16_t mac_destination)
{
	struct tx_request request;

	if ((buffer == NULL) ||
	    (length == 0) ||
	    (length > sizeof(request.data))) {
		return -EINVAL;
	}

	memset(&request, 0, sizeof(request));
	request.mac_destination = mac_destination;
	request.length = (uint16_t)length;
	memcpy(request.data, buffer, length);

	if (k_msgq_put(&tx_queue, &request, K_NO_WAIT) != 0) {
		LOG_WRN("TX queue full; packet dropped");
		return -ENOBUFS;
	}

	return 0;
}


/* =========================================================
 * Submit application packet (RX/protocol thread)
 *
 * Serializes and enqueues; the TX thread performs the radio
 * send. mac_destination is the IEEE 802.15.4 MAC destination.
 * ========================================================= */

static int send_oz_packet(
	const struct oz_packet *packet,
	uint16_t mac_destination)
{
	uint8_t buffer[OZ_PACKET_SIZE];

	serialize_packet(packet, buffer);
	return tx_queue_submit(buffer, sizeof(buffer), mac_destination);
}


/* =========================================================
 * Submit application packet + message body
 *
 * Same as send_oz_packet(), but appends a message-specific
 * body (e.g. the HEARTBEAT body) after the OZ header.
 * ========================================================= */

static int send_oz_packet_with_body(
	const struct oz_packet *packet,
	const uint8_t *body,
	size_t body_length,
	uint16_t mac_destination)
{
	uint8_t buffer[OZ_PACKET_SIZE + OZ_MAX_BODY_SIZE];
	size_t length;

	if (body_length > OZ_MAX_BODY_SIZE) {
		LOG_ERR("Body too large: %d bytes", (int)body_length);
		return -EMSGSIZE;
	}

	length = OZ_PACKET_SIZE + body_length;
	serialize_packet(packet, buffer);
	memcpy(&buffer[OZ_PACKET_SIZE], body, body_length);

	return tx_queue_submit(buffer, length, mac_destination);
}


/* =========================================================
 * Receive one application packet
 *
 * SOCK_DGRAM strips the IEEE 802.15.4 MAC header, so the
 * receive buffer contains the application payload.
 * ========================================================= */

static __maybe_unused int receive_oz_packet(
    struct oz_packet *packet,
    uint16_t *mac_source,
    int8_t *rssi_dbm)
{
    struct sockaddr_ll source;
    socklen_t source_len = sizeof(source);

    uint8_t buffer[IEEE802154_MTU];

    ssize_t received;

    memset(
        &source,
        0,
        sizeof(source)
    );

    LOG_INF("Before calling packet receive");

	received = zsock_recvfrom(
    	packet_socket,
    	buffer,
    	sizeof(buffer),
    	0,
    	(const struct sockaddr *)&source,
    	&source_len
	);

if (received < 0) {
    LOG_ERR(
        "recvfrom() failed, errno=%d",
        errno
    );

    return -errno;
}

/* AF_PACKET recvfrom() does not expose packet RSSI metadata. */
if (rssi_dbm != NULL) {
    *rssi_dbm = INT8_MAX;
}

LOG_INF(
    "Packet received with size %d",
    (int)received
);

printk("RX: ");

int dump_len = received;

if (dump_len > 20) {
    dump_len = 20;
}

for (int i = 0; i < dump_len; i++) {
    printk("%02X ", buffer[i]);
}

printk("\n");


    if (received < 0) {

        LOG_ERR(
            "recvfrom() failed, errno=%d",
            errno
        );

        return -errno;
    }

    /*
     * Ignore anything shorter than the common OZ header.
     * Longer frames may carry a type-specific body.
     */
    if (received < OZ_PACKET_SIZE) {

        LOG_INF(
            "Ignoring packet length %d, need at least %d",
            (int)received,
            OZ_PACKET_SIZE
        );

        return -EMSGSIZE;
    }

    if (!deserialize_packet(
            buffer,
            (size_t)received,
            packet)) {

        LOG_INF(
            "Non-OZ packet ignored"
        );

        return -EINVAL;
    }

    /*
     * Recover MAC source short address when available.
     */
    if ((mac_source != NULL) &&
        (source.sll_halen ==
         IEEE802154_SHORT_ADDR_LENGTH)) {

        *mac_source =
            sys_get_be16(
                source.sll_addr
            );

    } else if (mac_source != NULL) {

        *mac_source = 0xffff;
    }

    return 0;
}


/* Used by both roles now: the sender receives heartbeats. */
// #if NODE_ROLE == ROLE_RECEIVER
/* =========================================================
 * Packet-aware receiver
 *
 * The callback receives Zephyr's original net_pkt, allowing
 * RSSI to be read from the metadata for that exact frame.
 * The frame is copied into a message queue so processing and
 * logging occur in the RX/protocol thread.
 * ========================================================= */

static void packet_receive_callback(
	struct net_context *context,
	struct net_pkt *pkt,
	union net_ip_header *ip_hdr,
	union net_proto_header *proto_hdr,
	int status,
	void *user_data)
{
	struct received_frame frame;
	struct net_linkaddr *source;
	size_t length;

	ARG_UNUSED(context);
	ARG_UNUSED(ip_hdr);
	ARG_UNUSED(proto_hdr);
	ARG_UNUSED(user_data);

	if ((status != 0) || (pkt == NULL)) {
		if (pkt != NULL) {
			net_pkt_unref(pkt);
		}
		return;
	}

	memset(&frame, 0, sizeof(frame));
	frame.rssi_dbm = net_pkt_ieee802154_rssi_dbm(pkt);
	frame.mac_source = 0xffff;

	source = net_pkt_lladdr_src(pkt);
	if ((source != NULL) &&
	    (source->addr != NULL) &&
	    (source->len == IEEE802154_SHORT_ADDR_LENGTH)) {
		frame.mac_source = sys_get_be16(source->addr);
	}

	length = net_pkt_get_len(pkt);
	if (length > sizeof(frame.data)) {
		length = sizeof(frame.data);
	}

	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, frame.data, length) == 0) {
		frame.length = length;

		if (k_msgq_put(&receive_queue, &frame, K_NO_WAIT) != 0) {
			LOG_WRN("RX queue full; packet dropped");
		}
	}

	net_pkt_unref(pkt);
}


static int create_receive_context(void)
{
	struct sockaddr_ll local_addr;
	int ret;

	printk("Calling net_context_get()...\n");
	ret = net_context_get(
		AF_PACKET,
		SOCK_DGRAM,
		ETH_P_IEEE802154,
		&receive_context
	);

	if (ret != 0) {
		LOG_ERR("net_context_get() failed: %d", ret);
		return ret;
	}
	printk("net_context_get() returned: %d\n", ret);

	/* Match Zephyr's packet-socket setup: register once when
	 * the context is created and again after it is bound.
	 */
	ret = net_context_recv(
		receive_context,
		packet_receive_callback,
		K_NO_WAIT,
		NULL
	);

	if (ret != 0) {
		LOG_ERR("Initial net_context_recv() failed: %d", ret);
		net_context_put(receive_context);
		receive_context = NULL;
		return ret;
	}

	memset(&local_addr, 0, sizeof(local_addr));
	local_addr.sll_family = AF_PACKET;
	local_addr.sll_protocol = htons(ETH_P_IEEE802154);
	local_addr.sll_ifindex = net_if_get_by_iface(iface);

	printk("Calling net_context_bind()...\n");
	ret = net_context_bind(
		receive_context,
		(const struct sockaddr *)&local_addr,
		sizeof(local_addr)
	);

	if (ret != 0) {
		LOG_ERR("net_context_bind() failed: %d", ret);
		net_context_put(receive_context);
		receive_context = NULL;
		return ret;
	}
	printk("net_context_bind() returned: %d\n", ret);

	printk("Calling net_context_recv()...\n");
	ret = net_context_recv(
		receive_context,
		packet_receive_callback,
		K_NO_WAIT,
		NULL
	);

	if (ret != 0) {
		LOG_ERR("net_context_recv() failed: %d", ret);
		net_context_put(receive_context);
		receive_context = NULL;
		return ret;
	}
	printk("net_context_recv() returned: %d\n", ret);

	LOG_INF("Packet-aware receiver context created");
	return 0;
}
// #endif


/* =========================================================
 * Build OZ packet
 * ========================================================= */

static void build_packet(
	struct oz_packet *packet,
	uint8_t message_type,
	uint16_t destination_id,
	int16_t value,
	uint16_t sequence)
{
	memset(packet, 0, sizeof(*packet));

	packet->signature[0] = 'O';
	packet->signature[1] = 'Z';
	packet->version = PROTOCOL_VERSION;
	packet->message_type = message_type;
	packet->source_id = local_address;
	packet->destination_id = destination_id;
	/*
	 * Default join context from local FSM state. JOIN_RESPONSE
	 * / ACK builders override assigned_id (and often parent /
	 * hop) after calling this helper.
	 */
	packet->parent_id = parent_address;
	packet->assigned_id = UNASSIGNED_ADDRESS;
	packet->hop_count = hop_count;
	packet->value = value;
	packet->sequence = sequence;
}


#if NODE_ROLE == ROLE_ROOT

/* =========================================================
 * Root address allocation table (§4.5 / §5)
 * ========================================================= */

static int address_table_find_by_nonce(uint16_t join_nonce)
{
	int i;

	if (join_nonce == 0) {
		return -1;
	}

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (address_table[i].in_use &&
		    (address_table[i].join_nonce == join_nonce)) {
			return i;
		}
	}

	return -1;
}

static int address_table_find_by_assigned(uint16_t assigned)
{
	int i;

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (address_table[i].in_use &&
		    (address_table[i].address == assigned)) {
			return i;
		}
	}

	return -1;
}

static uint8_t address_table_parent_hop(uint16_t parent)
{
	int idx;

	if (address_is_root(parent)) {
		return 0;
	}

	idx = address_table_find_by_assigned(parent);
	if ((idx >= 0) && address_table[idx].active) {
		return address_table[idx].hop_count;
	}

	return HOP_COUNT_UNKNOWN;
}

static bool address_table_parent_ok(uint16_t parent)
{
	int idx;

	if (address_is_root(parent)) {
		return true;
	}

	idx = address_table_find_by_assigned(parent);
	return (idx >= 0) && address_table[idx].active;
}

static int address_table_alloc_slot(void)
{
	int i;

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (!address_table[i].in_use) {
			return i;
		}
	}

	return -1;
}

static bool address_table_address_taken(uint16_t addr)
{
	return address_table_find_by_assigned(addr) >= 0;
}

/*
 * Allocate the lowest free assignable address 1:1 .. 1:253.
 * Returns 0 on success.
 */
static int address_table_allocate(
	uint16_t parent,
	uint8_t child_hop,
	uint16_t request_sequence,
	uint16_t join_nonce,
	uint16_t requester_mac,
	struct address_record **out_record)
{
	int slot;
	uint8_t node_id;
	uint16_t candidate;

	slot = address_table_alloc_slot();
	if (slot < 0) {
		LOG_ERR("Address table full (%d entries)", ADDRESS_TABLE_CAPACITY);
		return -ENOMEM;
	}

	for (node_id = NODE_ID_FIRST_ASSIGNABLE;
	     node_id <= NODE_ID_LAST_ASSIGNABLE;
	     node_id++) {
		candidate = MAKE_ADDRESS(NETWORK_ID_ROOT, node_id);
		if (!address_table_address_taken(candidate)) {
			break;
		}
	}

	if (node_id > NODE_ID_LAST_ASSIGNABLE) {
		LOG_ERR("Address pool exhausted (1:1 .. 1:253)");
		return -ENOSPC;
	}

	memset(&address_table[slot], 0, sizeof(address_table[slot]));
	address_table[slot].address = candidate;
	address_table[slot].parent = parent;
	address_table[slot].hop_count = child_hop;
	address_table[slot].request_sequence = request_sequence;
	address_table[slot].join_nonce = join_nonce;
	address_table[slot].requester_mac = requester_mac;
	address_table[slot].pending = true;
	address_table[slot].active = false;
	address_table[slot].in_use = true;
	address_table[slot].pending_deadline_ms =
		k_uptime_get() +
		(JOIN_ACK_PENDING_TIMEOUT_SECONDS * 1000);

	*out_record = &address_table[slot];

	LOG_INF(
		"Allocated pending %u:%u parent=%u:%u hop=%u nonce=%u mac=0x%04X seq=%u",
		(unsigned int)ADDRESS_NETWORK(candidate),
		(unsigned int)ADDRESS_NODE(candidate),
		(unsigned int)ADDRESS_NETWORK(parent),
		(unsigned int)ADDRESS_NODE(parent),
		(unsigned int)child_hop,
		(unsigned int)join_nonce,
		(unsigned int)requester_mac,
		(unsigned int)request_sequence
	);

	return 0;
}

static void address_table_release(int idx)
{
	if ((idx < 0) || (idx >= ADDRESS_TABLE_CAPACITY)) {
		return;
	}

	LOG_INF(
		"Released address %u:%u (pending=%d active=%d)",
		(unsigned int)ADDRESS_NETWORK(address_table[idx].address),
		(unsigned int)ADDRESS_NODE(address_table[idx].address),
		(int)address_table[idx].pending,
		(int)address_table[idx].active
	);

	memset(&address_table[idx], 0, sizeof(address_table[idx]));
}

static void address_table_check_timeouts(void)
{
	int64_t now = k_uptime_get();
	int i;

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (!address_table[i].in_use || !address_table[i].pending) {
			continue;
		}

		if (now >= address_table[i].pending_deadline_ms) {
			LOG_WRN(
				"JOIN_ACK timeout for pending %u:%u; releasing",
				(unsigned int)ADDRESS_NETWORK(
					address_table[i].address),
				(unsigned int)ADDRESS_NODE(
					address_table[i].address)
			);
			address_table_release(i);
		}
	}
}

static int send_join_response_for_record(
	const struct address_record *record,
	uint16_t request_sequence,
	uint16_t mac_destination)
{
	struct oz_packet response;
	int ret;

	build_packet(
		&response,
		MSG_TYPE_JOIN_RESPONSE,
		UNASSIGNED_ADDRESS,
		0,
		request_sequence
	);

	response.parent_id = record->parent;
	response.assigned_id = record->address;
	response.hop_count = record->hop_count;

	print_packet("JOIN_RESPONSE TX", &response);

	ret = send_oz_packet(&response, mac_destination);
	if (ret) {
		LOG_ERR("JOIN_RESPONSE TX failed: %d", ret);
	}

	return ret;
}

#endif /* ROLE_ROOT */


#if NODE_ROLE == ROLE_JOINING

/* =========================================================
 * Parent selection + JOIN_REQUEST (§4.3–4.4)
 * ========================================================= */

static void record_parent_candidate(
	uint16_t address,
	uint16_t parent_id,
	uint8_t hop,
	int16_t rssi_dbm)
{
	size_t i;

	if (address_is_unassigned(address) ||
	    address_is_broadcast(address)) {
		return;
	}

	for (i = 0; i < parent_candidate_count; i++) {
		if (parent_candidates[i].address == address) {
			parent_candidates[i].parent_id = parent_id;
			parent_candidates[i].hop_count = hop;
			parent_candidates[i].rssi_dbm = rssi_dbm;
			parent_candidates[i].valid = true;
			return;
		}
	}

	if (parent_candidate_count >= MAX_PARENT_CANDIDATES) {
		LOG_WRN("Parent candidate table full; dropping %u:%u",
			(unsigned int)ADDRESS_NETWORK(address),
			(unsigned int)ADDRESS_NODE(address));
		return;
	}

	parent_candidates[parent_candidate_count].address = address;
	parent_candidates[parent_candidate_count].parent_id = parent_id;
	parent_candidates[parent_candidate_count].hop_count = hop;
	parent_candidates[parent_candidate_count].rssi_dbm = rssi_dbm;
	parent_candidates[parent_candidate_count].valid = true;
	parent_candidate_count++;
}

/*
 * Return true if `a` is a better parent than `b`.
 * Policy: lowest hop, then strongest RSSI, then lowest address.
 */
static bool parent_candidate_better(
	const struct parent_candidate *a,
	const struct parent_candidate *b)
{
	bool a_hop_known = (a->hop_count != HOP_COUNT_UNKNOWN);
	bool b_hop_known = (b->hop_count != HOP_COUNT_UNKNOWN);

	if (a_hop_known != b_hop_known) {
		return a_hop_known;
	}

	if (a_hop_known && (a->hop_count != b->hop_count)) {
		return a->hop_count < b->hop_count;
	}

	if ((a->rssi_dbm != IEEE802154_MAC_RSSI_DBM_UNDEFINED) &&
	    (b->rssi_dbm != IEEE802154_MAC_RSSI_DBM_UNDEFINED) &&
	    (a->rssi_dbm != b->rssi_dbm)) {
		return a->rssi_dbm > b->rssi_dbm;
	}

	if ((a->rssi_dbm != IEEE802154_MAC_RSSI_DBM_UNDEFINED) !=
	    (b->rssi_dbm != IEEE802154_MAC_RSSI_DBM_UNDEFINED)) {
		return a->rssi_dbm != IEEE802154_MAC_RSSI_DBM_UNDEFINED;
	}

	return a->address < b->address;
}

static int select_best_parent(struct parent_candidate *out)
{
	size_t i;
	int best = -1;

	for (i = 0; i < parent_candidate_count; i++) {
		if (!parent_candidates[i].valid) {
			continue;
		}

		if ((best < 0) ||
		    parent_candidate_better(
			    &parent_candidates[i],
			    &parent_candidates[best])) {
			best = (int)i;
		}
	}

	if (best < 0) {
		return -ENOENT;
	}

	*out = parent_candidates[best];
	return 0;
}

static int send_join_request(void)
{
	struct oz_packet request;
	int ret;

	build_packet(
		&request,
		MSG_TYPE_JOIN_REQUEST,
		ROOT_ADDRESS,
		(int16_t)join_request_nonce,
		join_request_sequence
	);

	request.parent_id = selected_parent_address;
	request.assigned_id = UNASSIGNED_ADDRESS;
	request.hop_count = (selected_parent_hop == HOP_COUNT_UNKNOWN) ?
			    HOP_COUNT_UNKNOWN :
			    (uint8_t)(selected_parent_hop + 1U);

	print_packet("JOIN_REQUEST TX", &request);

	ret = send_oz_packet(&request, ROOT_ADDRESS);
	if (ret) {
		LOG_ERR("JOIN_REQUEST TX failed: %d", ret);
		return ret;
	}

	join_request_outstanding = true;
	k_timer_start(
		&join_timer,
		K_SECONDS(JOIN_RESPONSE_TIMEOUT_SECONDS),
		K_NO_WAIT
	);

	LOG_INF(
		"JOIN_REQUEST seq=%u nonce=%u parent=%u:%u hop=%u (retry %u/%u)",
		(unsigned int)join_request_sequence,
		(unsigned int)join_request_nonce,
		(unsigned int)ADDRESS_NETWORK(selected_parent_address),
		(unsigned int)ADDRESS_NODE(selected_parent_address),
		(unsigned int)request.hop_count,
		(unsigned int)join_request_retries,
		(unsigned int)JOIN_REQUEST_MAX_RETRIES
	);

	return 0;
}

static void start_join_with_parent(const struct parent_candidate *chosen)
{
	LOG_INF(
		"Parent selected %u:%u hop=%u rssi=%d (policy: hop, RSSI, addr)",
		(unsigned int)ADDRESS_NETWORK(chosen->address),
		(unsigned int)ADDRESS_NODE(chosen->address),
		(unsigned int)chosen->hop_count,
		(int)chosen->rssi_dbm
	);

	node_enter_waiting_for_join_response(
		chosen->address,
		chosen->hop_count
	);

	join_request_sequence = tx_sequence++;
	/*
	 * Non-zero nonce distinguishes concurrent joiners that still
	 * share the unassigned short address 0xFFFE.
	 */
	join_request_nonce = (uint16_t)(k_cycle_get_32() ^
					(k_uptime_get_32() << 1) ^
					tx_sequence);
	if (join_request_nonce == 0) {
		join_request_nonce = 1;
	}
	join_request_retries = 0;

	if (send_join_request()) {
		LOG_ERR("Initial JOIN_REQUEST failed; reverting to PROBING");
		node_revert_to_unregistered();
		node_enter_probing();
		k_timer_start(
			&probe_timer,
			K_SECONDS(probe_k_seconds),
			K_NO_WAIT
		);
	}
}

static void parent_selection_window_expired(void)
{
	struct parent_candidate chosen;

	selection_window_active = false;

	if (current_state != NODE_PROBING) {
		return;
	}

	if (select_best_parent(&chosen)) {
		LOG_WRN("Selection window ended with no candidates");
		return;
	}

	start_join_with_parent(&chosen);
}

static void join_response_timeout(void)
{
	if (current_state != NODE_WAITING_FOR_JOIN_RESPONSE) {
		return;
	}

	join_request_retries++;

	if (join_request_retries > JOIN_REQUEST_MAX_RETRIES) {
		LOG_WRN(
			"JOIN_RESPONSE timeout after %u retries; revert to PROBING",
			(unsigned int)JOIN_REQUEST_MAX_RETRIES
		);
		node_revert_to_unregistered();
		node_enter_probing();
		k_timer_start(
			&probe_timer,
			K_SECONDS(probe_k_seconds),
			K_NO_WAIT
		);
		return;
	}

	LOG_WRN(
		"JOIN_RESPONSE timeout; retransmitting JOIN_REQUEST (%u/%u)",
		(unsigned int)join_request_retries,
		(unsigned int)JOIN_REQUEST_MAX_RETRIES
	);

	/*
	 * If the radio TX fails, restart the join timer so we are
	 * not stuck forever in WAITING_FOR_JOIN_RESPONSE with no
	 * outstanding deadline. Exhausted retries still revert.
	 */
	if (send_join_request()) {
		LOG_ERR("JOIN_REQUEST retransmit failed; restarting join timer");
		k_timer_start(
			&join_timer,
			K_SECONDS(JOIN_RESPONSE_TIMEOUT_SECONDS),
			K_NO_WAIT
		);
	}
}

static int send_join_acknowledge(
	uint16_t assigned,
	uint16_t parent,
	uint8_t hop,
	uint16_t sequence)
{
	struct oz_packet ack;
	int ret;

	build_packet(
		&ack,
		MSG_TYPE_JOIN_ACKNOWLEDGE,
		ROOT_ADDRESS,
		0,
		sequence
	);

	ack.parent_id = parent;
	ack.assigned_id = assigned;
	ack.hop_count = hop;

	print_packet("JOIN_ACKNOWLEDGE TX", &ack);

	ret = send_oz_packet(&ack, ROOT_ADDRESS);
	if (ret) {
		LOG_ERR("JOIN_ACKNOWLEDGE TX failed: %d", ret);
	}

	return ret;
}

/*
 * DATA TX toward parent (§4.7). Source stays this node's
 * assigned address; destination / MAC are the parent so
 * intermediate nodes can forward hop-by-hop to the root.
 */
static int send_sensor_data(void)
{
	struct oz_packet data;
	int ret;

	if (!node_can_send_data()) {
		return -EPERM;
	}

	if (address_is_unassigned(parent_address) ||
	    address_is_broadcast(parent_address)) {
		LOG_ERR("DATA aborted: parent not set");
		return -EINVAL;
	}

	build_packet(
		&data,
		MSG_TYPE_DATA,
		parent_address,
		sensor_value,
		data_tx_sequence
	);

	data.parent_id = parent_address;
	data.assigned_id = UNASSIGNED_ADDRESS;
	data.hop_count = hop_count;

	print_packet("DATA TX", &data);

	ret = send_oz_packet(&data, parent_address);
	if (ret) {
		LOG_ERR("DATA TX failed: %d", ret);
		return ret;
	}

	LOG_INF(
		"DATA seq=%u value=%d -> parent %u:%u",
		(unsigned int)data.sequence,
		(int)data.value,
		(unsigned int)ADDRESS_NETWORK(parent_address),
		(unsigned int)ADDRESS_NODE(parent_address)
	);

	sensor_value++;
	data_tx_sequence++;

	return 0;
}

static void data_timer_expired(void)
{
	if (!node_can_send_data()) {
		stop_data_collection();
		return;
	}

	(void)send_sensor_data();
	k_timer_start(&data_timer, K_MSEC(TX_PERIOD_MS), K_NO_WAIT);
}

/*
 * Forward a child's DATA one hop toward the root. Preserves the
 * original source_id / sequence / value so the root can attribute
 * the reading and detect gaps.
 */
static int forward_data_to_parent(const struct oz_packet *packet)
{
	struct oz_packet forward;
	int ret;

	if (!node_can_send_data()) {
		return -EPERM;
	}

	if (address_is_unassigned(parent_address) ||
	    address_is_broadcast(parent_address)) {
		LOG_ERR("DATA forward aborted: parent not set");
		return -EINVAL;
	}

	if (packet->source_id == local_address) {
		return 0;
	}

	forward = *packet;
	forward.destination_id = parent_address;
	forward.parent_id = parent_address;
	forward.hop_count = hop_count;

	print_packet("DATA FWD", &forward);

	ret = send_oz_packet(&forward, parent_address);
	if (ret) {
		LOG_ERR("DATA forward failed: %d", ret);
		return ret;
	}

	LOG_INF(
		"DATA forwarded origin=%u:%u seq=%u value=%d -> %u:%u",
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)packet->sequence,
		(int)packet->value,
		(unsigned int)ADDRESS_NETWORK(parent_address),
		(unsigned int)ADDRESS_NODE(parent_address)
	);

	return 0;
}

#endif /* ROLE_JOINING */


static uint32_t track_sequence_gap(
	uint16_t sequence,
	uint16_t *last_sequence,
	bool *valid);

#if NODE_ROLE == ROLE_ROOT

static int data_collector_find(uint16_t source)
{
	int i;

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (data_collectors[i].in_use &&
		    (data_collectors[i].source == source)) {
			return i;
		}
	}

	return -1;
}

static int data_collector_alloc(uint16_t source)
{
	int i;

	for (i = 0; i < ADDRESS_TABLE_CAPACITY; i++) {
		if (!data_collectors[i].in_use) {
			memset(&data_collectors[i], 0, sizeof(data_collectors[i]));
			data_collectors[i].source = source;
			data_collectors[i].in_use = true;
			return i;
		}
	}

	return -1;
}

/*
 * Record one DATA sample at the root and log any sequence gaps
 * for that originator since the previous sample.
 */
static void root_collect_data(const struct oz_packet *packet)
{
	int idx;
	uint32_t gap;

	idx = data_collector_find(packet->source_id);
	if (idx < 0) {
		idx = data_collector_alloc(packet->source_id);
		if (idx < 0) {
			LOG_ERR("DATA collector table full; dropping sample");
			return;
		}

		LOG_INF(
			"DATA collector started for %u:%u",
			(unsigned int)ADDRESS_NETWORK(packet->source_id),
			(unsigned int)ADDRESS_NODE(packet->source_id)
		);
	}

	gap = track_sequence_gap(
		packet->sequence,
		&data_collectors[idx].last_sequence,
		&data_collectors[idx].sequence_valid
	);

	data_collectors[idx].received++;
	data_collectors[idx].lost += gap;
	data_collectors[idx].last_value = packet->value;

	if (gap > 0) {
		LOG_WRN(
			"DATA missing from %u:%u: %u packet(s) before seq=%u (total lost=%u / rx=%u)",
			(unsigned int)ADDRESS_NETWORK(packet->source_id),
			(unsigned int)ADDRESS_NODE(packet->source_id),
			(unsigned int)gap,
			(unsigned int)packet->sequence,
			(unsigned int)data_collectors[idx].lost,
			(unsigned int)data_collectors[idx].received
		);
	}

	LOG_INF(
		"DATA collected origin=%u:%u seq=%u value=%d (lost=%u / rx=%u)",
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)packet->sequence,
		(int)packet->value,
		(unsigned int)data_collectors[idx].lost,
		(unsigned int)data_collectors[idx].received
	);
}

#endif /* ROLE_ROOT */


/* =========================================================
 * Message handlers
 *
 * Every handler has the same signature so it can be stored
 * in message_handlers[]. Handlers run in the RX/protocol
 * thread; any TX goes through send_oz_packet() → tx_queue.
 * ========================================================= */

typedef void (*message_handler_t)(
	const struct oz_packet *packet,
	const struct received_frame *frame);


/*
 * Print a HEARTBEAT body together with the probe RSSI carried
 * in the OZ header value field.
 */
static void print_heartbeat_body(
	const char *title,
	const struct heartbeat_body *body,
	int16_t probe_rssi_dbm)
{
	LOG_INF("--------------------------------");

	LOG_INF("%s", title);

	LOG_INF(
		"GUID          = %02X%02X%02X%02X%02X%02X%02X%02X",
		body->guid[0], body->guid[1], body->guid[2], body->guid[3],
		body->guid[4], body->guid[5], body->guid[6], body->guid[7]
	);

	if (body->battery_percent == BATTERY_PERCENT_UNKNOWN) {
		LOG_INF("Battery       = unknown");
	} else {
		LOG_INF(
			"Battery       = %u mV (%u %%)",
			(unsigned int)body->battery_mv,
			(unsigned int)body->battery_percent
		);
	}

	LOG_INF(
		"HB Sequence   = %u",
		(unsigned int)body->heartbeat_sequence
	);

	if (probe_rssi_dbm == IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
		LOG_INF("Probe RSSI    = unavailable");
	} else {
		LOG_INF("Probe RSSI    = %d dBm", (int)probe_rssi_dbm);
	}

	LOG_INF("--------------------------------");
}


/*
 * Count the frames missing between the last sequence seen and
 * this one. A sequence that does not move forward (signed
 * 16-bit difference <= 0) means the peer rebooted or the frame
 * is a duplicate, so tracking restarts without counting loss.
 * The signed difference also handles uint16_t wraparound.
 */
static uint32_t track_sequence_gap(
	uint16_t sequence,
	uint16_t *last_sequence,
	bool *valid)
{
	int16_t delta;
	uint32_t lost = 0;

	if (*valid) {
		delta = (int16_t)(sequence - *last_sequence);
		if (delta > 1) {
			lost = (uint32_t)(delta - 1);
		}
	}

	*last_sequence = sequence;
	*valid = true;

	return lost;
}


/*
 * Log "lost / total (x.y %)" with integer math, so it does
 * not depend on floating point printf support.
 */
static void log_loss_line(const char *label, uint32_t lost, uint32_t total)
{
	uint32_t permille = 0;

	if (total > 0) {
		permille = (uint32_t)(((uint64_t)lost * 1000U) / total);
	}

	LOG_INF(
		"%s = %u / %u (%u.%u %%)",
		label,
		(unsigned int)lost,
		(unsigned int)total,
		(unsigned int)(permille / 10U),
		(unsigned int)(permille % 10U)
	);
}


static void print_link_stats(void)
{
	LOG_INF("--------------------------------");

#if NODE_ROLE == ROLE_JOINING
	LOG_INF("LINK STATS (joining / probe)");

	LOG_INF(
		"Probes sent   = %u (TX failed %u)",
		(unsigned int)link_stats.probes_sent,
		(unsigned int)link_stats.probes_tx_failed
	);

	LOG_INF(
		"HB accepted   = %u",
		(unsigned int)link_stats.heartbeats_accepted
	);

	/* Round trip: resolved probes = answered + timed out. */
	log_loss_line(
		"Probe missed ",
		link_stats.probes_missed,
		link_stats.heartbeats_accepted + link_stats.probes_missed
	);

	/* Return path: heartbeat sequence gaps. */
	log_loss_line(
		"HB lost      ",
		link_stats.heartbeats_lost,
		link_stats.heartbeats_received + link_stats.heartbeats_lost
	);

	LOG_INF(
		"HB stale      = %u",
		(unsigned int)link_stats.heartbeats_stale
	);
#elif NODE_ROLE == ROLE_ROOT
	LOG_INF("LINK STATS (root / heartbeat)");

	LOG_INF(
		"Probes RX     = %u",
		(unsigned int)link_stats.probes_received
	);

	/* Forward path: PROBE sequence gaps. */
	log_loss_line(
		"Probes lost  ",
		link_stats.probes_lost,
		link_stats.probes_received + link_stats.probes_lost
	);

	LOG_INF(
		"HB sent       = %u (TX failed %u)",
		(unsigned int)link_stats.heartbeats_sent,
		(unsigned int)link_stats.heartbeats_tx_failed
	);
#endif

	LOG_INF("--------------------------------");
}


/*
 * PROBE: reply with a HEARTBEAT to the probing node. The
 * heartbeat header echoes the probe sequence and carries the
 * RSSI at which the probe was received; the heartbeat body
 * carries GUID, battery level and heartbeat sequence.
 *
 * MAC destination is broadcast so unassigned joiners (short
 * address 0xFFFE) still receive the frame; see
 * mac_dest_for_unassigned_peer() / 11-23-code-report.md.
 */
static void handle_probe(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	struct oz_packet heartbeat;
	struct heartbeat_body body;
	uint8_t body_buffer[HEARTBEAT_BODY_SIZE];
	int ret;

	if (!node_can_reply_heartbeat()) {
		LOG_INF(
			"PROBE seq=%u ignored (state=%s; only REGISTERED replies)",
			(unsigned int)packet->sequence,
			node_state_name(current_state)
		);
		return;
	}

	LOG_INF(
		"PROBE seq=%u from 0x%04X (sender K = %d s)",
		(unsigned int)packet->sequence,
		(unsigned int)packet->source_id,
		(int)packet->value
	);

	link_stats.probes_received++;
	link_stats.probes_lost += track_sequence_gap(
		packet->sequence,
		&last_probe_rx_sequence,
		&probe_rx_sequence_valid
	);

	build_packet(
		&heartbeat,
		MSG_TYPE_HEARTBEAT,
		packet->source_id,
		frame->rssi_dbm,
		packet->sequence
	);

	memcpy(body.guid, device_guid, GUID_SIZE);
	read_battery(&body.battery_mv, &body.battery_percent);
	body.heartbeat_sequence = heartbeat_tx_sequence++;

	serialize_heartbeat_body(&body, body_buffer);

	print_packet("HEARTBEAT TX", &heartbeat);
	print_heartbeat_body("HEARTBEAT TX BODY", &body, heartbeat.value);

	/*
	 * OZ destination stays the probe source (usually 0xFFFE) so
	 * only unassigned joiners accept at the app filter. MAC must
	 * not unicast to 0xFFFE: that address is filtered out by
	 * 802.15.4 RX on unassigned nodes (see helper).
	 */
	ret = send_oz_packet_with_body(
		&heartbeat,
		body_buffer,
		sizeof(body_buffer),
		mac_dest_for_unassigned_peer(packet->source_id)
	);
	link_stats.heartbeats_sent++;
	if (ret) {
		link_stats.heartbeats_tx_failed++;
		LOG_ERR("HEARTBEAT transmission failed: %d", ret);
	}

	print_link_stats();
}


/*
 * HEARTBEAT: while PROBING, record the responder as a parent
 * candidate. The first accepted reply starts the selection window;
 * when it expires the best candidate (hop, then RSSI, then address)
 * receives a JOIN_REQUEST.
 */
static void handle_heartbeat(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	struct heartbeat_body body;

	if (current_state != NODE_PROBING) {
		LOG_INF(
			"HEARTBEAT seq=%u ignored (state=%s; expected PROBING)",
			(unsigned int)packet->sequence,
			node_state_name(current_state)
		);
		return;
	}

	/* process_frame() guarantees frame->length >= OZ_PACKET_SIZE. */
	if (!deserialize_heartbeat_body(
		    &frame->data[OZ_PACKET_SIZE],
		    frame->length - OZ_PACKET_SIZE,
		    &body)) {
		LOG_WRN(
			"HEARTBEAT seq=%u from 0x%04X ignored: frame %d bytes, expected %d",
			(unsigned int)packet->sequence,
			(unsigned int)packet->source_id,
			(int)frame->length,
			HEARTBEAT_FRAME_SIZE
		);
		return;
	}

	/* Any valid heartbeat, stale or not, advances the gap tracking. */
	link_stats.heartbeats_received++;
	link_stats.heartbeats_lost += track_sequence_gap(
		body.heartbeat_sequence,
		&last_heartbeat_rx_sequence,
		&heartbeat_rx_sequence_valid
	);

#if NODE_ROLE == ROLE_JOINING
	/*
	 * Accept matching replies for the outstanding probe, and also
	 * additional replies while the selection window is already open
	 * (other registered nodes answering the same / later probe).
	 */
	if (probe_outstanding &&
	    (packet->sequence == probe_outstanding_sequence)) {
		probe_outstanding = false;
		probe_k_seconds = PROBE_K_INITIAL_SECONDS;
	} else if (!selection_window_active) {
		link_stats.heartbeats_stale++;
		LOG_WRN(
			"Stale/unsolicited HEARTBEAT seq=%u from 0x%04X ignored",
			(unsigned int)packet->sequence,
			(unsigned int)packet->source_id
		);
		return;
	}

	LOG_INF(
		"HEARTBEAT seq=%u from %u:%u accepted (parent=%u:%u hop=%u rssi=%d)",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)ADDRESS_NETWORK(packet->parent_id),
		(unsigned int)ADDRESS_NODE(packet->parent_id),
		(unsigned int)packet->hop_count,
		(int)frame->rssi_dbm
	);

	print_packet("HEARTBEAT RX", packet);
	print_heartbeat_body("HEARTBEAT RX BODY", &body, packet->value);

	record_parent_candidate(
		packet->source_id,
		packet->parent_id,
		packet->hop_count,
		frame->rssi_dbm
	);

	if (!selection_window_active) {
		selection_window_active = true;
		k_timer_start(
			&selection_timer,
			K_MSEC(PARENT_SELECTION_WINDOW_MS),
			K_NO_WAIT
		);
		LOG_INF(
			"Parent selection window open for %u ms",
			(unsigned int)PARENT_SELECTION_WINDOW_MS
		);
	}

	link_stats.heartbeats_accepted++;
	print_link_stats();
#else
	ARG_UNUSED(frame);
	link_stats.heartbeats_stale++;
	LOG_INF("HEARTBEAT ignored on non-joining role");
#endif
}


/*
 * JOIN_REQUEST: root validates, allocates (or finds pending),
 * and transmits JOIN_RESPONSE. Duplicate requests with the same
 * join nonce retransmit the original assignment — never a second
 * address (nonce distinguishes concurrent 0xFFFE joiners).
 */
static void handle_join_request(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	print_packet("JOIN_REQUEST RX", packet);

	LOG_INF(
		"JOIN_REQUEST seq=%u from %u:%u parent=%u:%u hop=%u mac=0x%04X",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)ADDRESS_NETWORK(packet->parent_id),
		(unsigned int)ADDRESS_NODE(packet->parent_id),
		(unsigned int)packet->hop_count,
		(unsigned int)frame->mac_source
	);

#if NODE_ROLE == ROLE_ROOT
	struct address_record *record = NULL;
	uint16_t join_nonce = (uint16_t)packet->value;
	uint8_t parent_hop;
	uint8_t child_hop;
	int idx;
	int ret;

	if (!address_is_root(local_address) ||
	    (current_state != NODE_REGISTERED)) {
		LOG_INF(
			"JOIN_REQUEST ignored (local=%u:%u state=%s)",
			(unsigned int)ADDRESS_NETWORK(local_address),
			(unsigned int)ADDRESS_NODE(local_address),
			node_state_name(current_state)
		);
		return;
	}

	if (join_nonce == 0) {
		LOG_WRN("JOIN_REQUEST rejected: join nonce is zero");
		return;
	}

	if (!address_table_parent_ok(packet->parent_id)) {
		LOG_WRN(
			"JOIN_REQUEST rejected: parent %u:%u not root/active",
			(unsigned int)ADDRESS_NETWORK(packet->parent_id),
			(unsigned int)ADDRESS_NODE(packet->parent_id)
		);
		return;
	}

	idx = address_table_find_by_nonce(join_nonce);
	if (idx >= 0) {
		record = &address_table[idx];
		record->request_sequence = packet->sequence;
		record->requester_mac = frame->mac_source;

		if (record->pending) {
			record->pending_deadline_ms =
				k_uptime_get() +
				(JOIN_ACK_PENDING_TIMEOUT_SECONDS * 1000);
		}

		LOG_INF(
			"Duplicate JOIN_REQUEST nonce=%u; retransmit %u:%u",
			(unsigned int)join_nonce,
			(unsigned int)ADDRESS_NETWORK(record->address),
			(unsigned int)ADDRESS_NODE(record->address)
		);

		(void)send_join_response_for_record(
			record,
			packet->sequence,
			mac_dest_for_unassigned_peer(frame->mac_source)
		);
		return;
	}

	parent_hop = address_table_parent_hop(packet->parent_id);
	if (parent_hop == HOP_COUNT_UNKNOWN) {
		LOG_WRN("JOIN_REQUEST rejected: parent hop unknown");
		return;
	}

	child_hop = (uint8_t)(parent_hop + 1U);

	ret = address_table_allocate(
		packet->parent_id,
		child_hop,
		packet->sequence,
		join_nonce,
		frame->mac_source,
		&record
	);
	if (ret) {
		LOG_ERR("JOIN_REQUEST failed: cannot allocate (%d)", ret);
		return;
	}

	(void)send_join_response_for_record(
		record,
		packet->sequence,
		mac_dest_for_unassigned_peer(frame->mac_source)
	);
#else
	ARG_UNUSED(frame);
	LOG_INF("JOIN_REQUEST ignored (not root)");
#endif
}


/*
 * JOIN_RESPONSE: joiner installs the assigned address, enters
 * REGISTERED, and confirms with JOIN_ACKNOWLEDGE.
 */
static void handle_join_response(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	ARG_UNUSED(frame);

	print_packet("JOIN_RESPONSE RX", packet);

	LOG_INF(
		"JOIN_RESPONSE seq=%u assigned=%u:%u parent=%u:%u hop=%u",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->assigned_id),
		(unsigned int)ADDRESS_NODE(packet->assigned_id),
		(unsigned int)ADDRESS_NETWORK(packet->parent_id),
		(unsigned int)ADDRESS_NODE(packet->parent_id),
		(unsigned int)packet->hop_count
	);

#if NODE_ROLE == ROLE_JOINING
	if (current_state != NODE_WAITING_FOR_JOIN_RESPONSE) {
		LOG_INF(
			"JOIN_RESPONSE ignored (state=%s; expected WAITING_FOR_JOIN_RESPONSE)",
			node_state_name(current_state)
		);
		return;
	}

	if (!join_request_outstanding ||
	    (packet->sequence != join_request_sequence)) {
		LOG_WRN(
			"JOIN_RESPONSE seq=%u ignored (expected seq=%u)",
			(unsigned int)packet->sequence,
			(unsigned int)join_request_sequence
		);
		return;
	}

	if (!address_is_assignable(packet->assigned_id)) {
		LOG_ERR(
			"JOIN_RESPONSE has invalid assigned address 0x%04X",
			(unsigned int)packet->assigned_id
		);
		return;
	}

	node_enter_registered(
		packet->assigned_id,
		packet->parent_id,
		packet->hop_count
	);

	if (current_state != NODE_REGISTERED) {
		LOG_ERR("Registration failed; returning to PROBING");
		node_enter_probing();
		k_timer_start(
			&probe_timer,
			K_SECONDS(probe_k_seconds),
			K_NO_WAIT
		);
		return;
	}

	(void)send_join_acknowledge(
		packet->assigned_id,
		packet->parent_id,
		packet->hop_count,
		packet->sequence
	);

	LOG_INF(
		"Node REGISTERED at %u:%u parent=%u:%u hop=%u",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)ADDRESS_NETWORK(parent_address),
		(unsigned int)ADDRESS_NODE(parent_address),
		(unsigned int)hop_count
	);
#else
	LOG_INF("JOIN_RESPONSE ignored (not joining)");
#endif
}


/*
 * JOIN_ACKNOWLEDGE: root marks the pending allocation active.
 */
static void handle_join_acknowledge(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	print_packet("JOIN_ACKNOWLEDGE RX", packet);

	LOG_INF(
		"JOIN_ACKNOWLEDGE seq=%u assigned=%u:%u parent=%u:%u hop=%u",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->assigned_id),
		(unsigned int)ADDRESS_NODE(packet->assigned_id),
		(unsigned int)ADDRESS_NETWORK(packet->parent_id),
		(unsigned int)ADDRESS_NODE(packet->parent_id),
		(unsigned int)packet->hop_count
	);

#if NODE_ROLE == ROLE_ROOT
	int idx;

	if (!address_is_root(local_address) ||
	    (current_state != NODE_REGISTERED)) {
		LOG_INF(
			"JOIN_ACKNOWLEDGE ignored (local=%u:%u state=%s)",
			(unsigned int)ADDRESS_NETWORK(local_address),
			(unsigned int)ADDRESS_NODE(local_address),
			node_state_name(current_state)
		);
		return;
	}

	idx = address_table_find_by_assigned(packet->assigned_id);
	if (idx < 0) {
		LOG_WRN(
			"JOIN_ACKNOWLEDGE for unknown address %u:%u",
			(unsigned int)ADDRESS_NETWORK(packet->assigned_id),
			(unsigned int)ADDRESS_NODE(packet->assigned_id)
		);
		return;
	}

	address_table[idx].pending = false;
	address_table[idx].active = true;
	address_table[idx].requester_mac = frame->mac_source;
	address_table[idx].parent = packet->parent_id;
	address_table[idx].hop_count = packet->hop_count;

	LOG_INF(
		"Address %u:%u marked ACTIVE (parent=%u:%u hop=%u)",
		(unsigned int)ADDRESS_NETWORK(address_table[idx].address),
		(unsigned int)ADDRESS_NODE(address_table[idx].address),
		(unsigned int)ADDRESS_NETWORK(address_table[idx].parent),
		(unsigned int)ADDRESS_NODE(address_table[idx].parent),
		(unsigned int)address_table[idx].hop_count
	);
#else
	ARG_UNUSED(frame);
	LOG_INF("JOIN_ACKNOWLEDGE ignored (not root)");
#endif
}


/*
 * DATA: sensor traffic toward the root (§4.7).
 * Root collects and reports sequence gaps. Registered non-root
 * parents forward hop-by-hop, preserving the originator source_id.
 */
static void handle_data(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	ARG_UNUSED(frame);

	print_packet("DATA RX", packet);

	LOG_INF(
		"DATA seq=%u value=%d from %u:%u hop=%u",
		(unsigned int)packet->sequence,
		(int)packet->value,
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)packet->hop_count
	);

#if NODE_ROLE == ROLE_ROOT
	if (!address_is_root(local_address) ||
	    (current_state != NODE_REGISTERED)) {
		LOG_INF(
			"DATA ignored at non-collecting node (state=%s)",
			node_state_name(current_state)
		);
		return;
	}

	root_collect_data(packet);
	return;
#elif NODE_ROLE == ROLE_JOINING
	if (!node_can_send_data()) {
		LOG_INF(
			"DATA ignored (state=%s; only REGISTERED nodes forward)",
			node_state_name(current_state)
		);
		return;
	}

	/*
	 * Accept DATA addressed to this node (child → parent hop).
	 * Re-address toward our parent while keeping origin fields.
	 */
	if (packet->destination_id != local_address) {
		LOG_INF(
			"DATA ignored: dest=%u:%u local=%u:%u",
			(unsigned int)ADDRESS_NETWORK(packet->destination_id),
			(unsigned int)ADDRESS_NODE(packet->destination_id),
			(unsigned int)ADDRESS_NETWORK(local_address),
			(unsigned int)ADDRESS_NODE(local_address)
		);
		return;
	}

	(void)forward_data_to_parent(packet);
#else
	ARG_UNUSED(packet);
#endif
}


/*
 * NETIDREQ / NETIDRESP: reserved for multi-network ID
 * discovery. Decode + log only until that design lands.
 */
static void handle_netid_request(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	ARG_UNUSED(frame);

	print_packet("NETID_REQUEST RX", packet);

	LOG_INF(
		"NETID_REQUEST seq=%u from %u:%u (multi-net reserved; not implemented)",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id)
	);
}


static void handle_netid_response(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	ARG_UNUSED(frame);

	print_packet("NETID_RESPONSE RX", packet);

	LOG_INF(
		"NETID_RESPONSE seq=%u from %u:%u assigned_hint=%u:%u (multi-net reserved; not implemented)",
		(unsigned int)packet->sequence,
		(unsigned int)ADDRESS_NETWORK(packet->source_id),
		(unsigned int)ADDRESS_NODE(packet->source_id),
		(unsigned int)ADDRESS_NETWORK(packet->assigned_id),
		(unsigned int)ADDRESS_NODE(packet->assigned_id)
	);
}


/*
 * Indexed by (message_type - MSG_TYPE_FIRST).
 * Every assignment type 1–8 has a handler; join / DATA /
 * NETID bodies still evolve in later milestones.
 */
static const message_handler_t message_handlers[MSG_HANDLER_COUNT] = {
	[MSG_TYPE_PROBE            - MSG_TYPE_FIRST] = handle_probe,
	[MSG_TYPE_HEARTBEAT        - MSG_TYPE_FIRST] = handle_heartbeat,
	[MSG_TYPE_JOIN_REQUEST     - MSG_TYPE_FIRST] = handle_join_request,
	[MSG_TYPE_JOIN_RESPONSE    - MSG_TYPE_FIRST] = handle_join_response,
	[MSG_TYPE_JOIN_ACKNOWLEDGE - MSG_TYPE_FIRST] = handle_join_acknowledge,
	[MSG_TYPE_DATA             - MSG_TYPE_FIRST] = handle_data,
	[MSG_TYPE_NETID_REQUEST    - MSG_TYPE_FIRST] = handle_netid_request,
	[MSG_TYPE_NETID_RESPONSE   - MSG_TYPE_FIRST] = handle_netid_response,
};


static void dispatch_message(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	message_handler_t handler;

	if ((packet->message_type < MSG_TYPE_FIRST) ||
	    (packet->message_type > MSG_TYPE_LAST)) {
		LOG_WRN(
			"Unknown message type %u ignored",
			(unsigned int)packet->message_type
		);
		return;
	}

	handler = message_handlers[packet->message_type - MSG_TYPE_FIRST];

	if (handler == NULL) {
		LOG_WRN(
			"No handler implemented for %s (type %u)",
			message_type_name(packet->message_type),
			(unsigned int)packet->message_type
		);
		return;
	}

	handler(packet, frame);
}


/* =========================================================
 * Process one received frame (both roles)
 *
 * Decode -> validate destination -> dispatch.
 * ========================================================= */

static void process_frame(const struct received_frame *frame)
{
	struct oz_packet packet;

	LOG_INF("Packet received with size %d", (int)frame->length);
	printk("RX: ");
	for (size_t i = 0; (i < frame->length) && (i < 32); i++) {
		printk("%02X ", frame->data[i]);
	}
	printk("\n");

	/*
	if (!deserialize_packet(frame->data, frame->length, &packet)) {
		LOG_INF("Non-OZ packet ignored");
		return;
	}
	*/

	/*
	 * The OZ header is the first OZ_PACKET_SIZE bytes. Any bytes
	 * after it are a message body that the handler parses.
	 */
	if (frame->length < OZ_PACKET_SIZE) {
		LOG_INF("Short packet ignored: %d bytes", (int)frame->length);
		return;
	}

	if (!deserialize_packet(frame->data, OZ_PACKET_SIZE, &packet)) {
		/* §6: invalid signature / version → ignore + diagnostic. */
		if ((frame->data[0] != 'O') || (frame->data[1] != 'Z')) {
			LOG_INF(
				"Invalid packet ignored: bad signature 0x%02X 0x%02X",
				(unsigned int)frame->data[0],
				(unsigned int)frame->data[1]
			);
		} else {
			LOG_INF(
				"Invalid packet ignored: bad version %u (expected %u)",
				(unsigned int)frame->data[2],
				(unsigned int)PROTOCOL_VERSION
			);
		}
		return;
	}

	if (!address_is_for_me(packet.destination_id)) {
		LOG_INF(
			"Packet ignored: destination=0x%04X (%u:%u), local=0x%04X (%u:%u)",
			(unsigned int)packet.destination_id,
			(unsigned int)ADDRESS_NETWORK(packet.destination_id),
			(unsigned int)ADDRESS_NODE(packet.destination_id),
			(unsigned int)local_address,
			(unsigned int)ADDRESS_NETWORK(local_address),
			(unsigned int)ADDRESS_NODE(local_address)
		);
		return;
	}

	LOG_INF(
		"MAC source = 0x%04X",
		frame->mac_source
	);

	if (frame->rssi_dbm == IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
		LOG_INF(
			"RSSI       = unavailable"
		);
	} else {
		LOG_INF(
			"RSSI       = %d dBm",
			(int)frame->rssi_dbm
		);
	}

	print_packet("RX", &packet);

	dispatch_message(&packet, frame);
}


/* =========================================================
 * Joining node loop (§4.1–4.7)
 *
 * PROBING: broadcast PROBE, collect HEARTBEATs, select parent.
 * WAITING_FOR_JOIN_RESPONSE: JOIN_REQUEST / timeout / retry.
 * REGISTERED: 1 Hz DATA toward parent + forward child DATA;
 *             also answers probes as a registered node.
 * ========================================================= */

#if NODE_ROLE == ROLE_JOINING

/*
 * Runs each time timer K expires. A probe still outstanding
 * here received no heartbeat during the last K seconds, so K
 * is incremented by N before the next probe is sent.
 */
static void probe_timer_expired(void)
{
	struct oz_packet probe;
	int ret;

	if (!node_can_send_probe()) {
		LOG_INF(
			"Probe timer ignored (state=%s)",
			node_state_name(current_state)
		);
		return;
	}

	/* Do not emit a new probe once parent selection is underway. */
	if (selection_window_active) {
		k_timer_start(
			&probe_timer,
			K_SECONDS(probe_k_seconds),
			K_NO_WAIT
		);
		return;
	}

	if (probe_outstanding) {
		link_stats.probes_missed++;
		probe_k_seconds += PROBE_N_INCREMENT_SECONDS;

		if (probe_k_seconds > PROBE_K_MAX_SECONDS) {
			probe_k_seconds = PROBE_K_MAX_SECONDS;
		}

		LOG_WRN(
			"No HEARTBEAT for PROBE seq=%u, K incremented to %u s",
			(unsigned int)probe_outstanding_sequence,
			probe_k_seconds
		);

		print_link_stats();
	}

	build_packet(
		&probe,
		MSG_TYPE_PROBE,
		BROADCAST_ADDRESS,
		(int16_t)probe_k_seconds,
		tx_sequence++
	);

	/* Explicit assignment fields for an unregistered probe. */
	probe.parent_id = UNASSIGNED_ADDRESS;
	probe.assigned_id = UNASSIGNED_ADDRESS;
	probe.hop_count = HOP_COUNT_UNKNOWN;

	print_packet("PROBE TX", &probe);

	ret = send_oz_packet(&probe, BROADCAST_ADDRESS);
	link_stats.probes_sent++;
	if (ret) {
		link_stats.probes_tx_failed++;
		LOG_ERR("PROBE transmission failed: %d", ret);
	}

	/* A failed transmission still counts as an unanswered probe. */
	probe_outstanding = true;
	probe_outstanding_sequence = probe.sequence;

	/* One-shot timer, restarted with the (possibly new) K. */
	k_timer_start(&probe_timer, K_SECONDS(probe_k_seconds), K_NO_WAIT);
}


static void run_probe_sender(void)
{
	struct received_frame frame;
	int ret;

	LOG_INF("Joining node started (PROBE → JOIN → REGISTERED → DATA)");
	log_node_state("Joining loop");
	LOG_INF(
		"Local       = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)local_address
	);
	LOG_INF(
		"PROBE dest  = %u:%u (0x%04X broadcast)",
		(unsigned int)ADDRESS_NETWORK(BROADCAST_ADDRESS),
		(unsigned int)ADDRESS_NODE(BROADCAST_ADDRESS),
		(unsigned int)BROADCAST_ADDRESS
	);
	LOG_INF(
		"K = %u s, N = %u s, K max = %u s, select window = %u ms",
		PROBE_K_INITIAL_SECONDS,
		PROBE_N_INCREMENT_SECONDS,
		PROBE_K_MAX_SECONDS,
		(unsigned int)PARENT_SELECTION_WINDOW_MS
	);
	LOG_INF("DATA period = %u ms after REGISTERED", (unsigned int)TX_PERIOD_MS);

	k_timer_start(&probe_timer, K_SECONDS(probe_k_seconds), K_NO_WAIT);

	while (1) {
		ret = k_msgq_get(
			&receive_queue,
			&frame,
			K_MSEC(JOINING_LOOP_POLL_MS)
		);
		if (ret == 0) {
			process_frame(&frame);
		}

		if (current_state == NODE_PROBING) {
			if (k_timer_status_get(&probe_timer) > 0) {
				probe_timer_expired();
			}

			if (selection_window_active &&
			    (k_timer_status_get(&selection_timer) > 0)) {
				parent_selection_window_expired();
			}
		} else if (current_state == NODE_WAITING_FOR_JOIN_RESPONSE) {
			if (k_timer_status_get(&join_timer) > 0) {
				join_response_timeout();
			}
		} else if (current_state == NODE_REGISTERED) {
			if (data_collection_active &&
			    (k_timer_status_get(&data_timer) > 0)) {
				data_timer_expired();
			}
		}
	}
}

#endif


/* =========================================================
 * Root node loop (PROBE reply + join allocation)
 * ========================================================= */

#if NODE_ROLE == ROLE_ROOT

static void run_probe_receiver(void)
{
	struct received_frame frame;
	int ret;

	LOG_INF("Root started (HEARTBEAT + JOIN allocation)");
	log_node_state("Root loop");
	LOG_INF(
		"Local       = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)local_address
	);
	LOG_INF("Waiting for PROBE / JOIN_REQUEST / JOIN_ACK...");

	while (1) {
		address_table_check_timeouts();

		ret = k_msgq_get(
			&receive_queue,
			&frame,
			K_SECONDS(RX_WAIT_TIMEOUT_SECONDS)
		);

		if (ret == -EAGAIN) {
			LOG_INF(
				"No packet callback in the last %d seconds",
				RX_WAIT_TIMEOUT_SECONDS
			);
			continue;
		}

		if (ret != 0) {
			LOG_ERR("RX queue error: %d", ret);
			continue;
		}

		process_frame(&frame);
	}
}

#endif


/* =========================================================
 * Sender role
 *
 * Legacy periodic DATA sender, disabled. Kept for future use;
 * run_probe_sender() is the active sender.
 * ========================================================= */

#if 0
#if NODE_ROLE == ROLE_SENDER

static void run_sender(void)
{
	struct oz_packet packet;

	int16_t value =
		100;

	int ret;


	LOG_INF(
		"Sender started"
	);


	LOG_INF(
		"Destination = 0x%04X",
		RECEIVER_ADDR
	);


	while (1) {


		/* -------------------------------------------------
		 * Build source packet
		 * ------------------------------------------------- */

		memset(
			&packet,
			0,
			sizeof(packet)
		);


		packet.signature[0] =
			'O';

		packet.signature[1] =
			'Z';


		packet.version =
			PROTOCOL_VERSION;


		packet.message_type =
			MESSAGE_TYPE_DATA;


		packet.source_id =
			SENDER_ADDR;


		packet.destination_id =
			RECEIVER_ADDR;


		packet.value =
			value;


		packet.sequence =
			tx_sequence++;


		print_packet(
			"SENDER TX",
			&packet
		);


		/* -------------------------------------------------
		 * MAC destination = receiver
		 * ------------------------------------------------- */

		ret =
			send_oz_packet(
				&packet,
				RECEIVER_ADDR
			);


		if (ret) {

			LOG_ERR(
				"Sender transmission failed: %d",
				ret
			);
		}


		value++;


		k_sleep(
			K_MSEC(
				TX_PERIOD_MS
			)
		);
	}
}

#endif
#endif /* legacy run_sender */


/* =========================================================
 * Receiver role
 *
 * Legacy receiver loop, disabled. Kept for future use;
 * run_probe_receiver() is the active receiver.
 * ========================================================= */

#if 0
#if NODE_ROLE == ROLE_RECEIVER


static void run_receiver(void)
{
	struct oz_packet packet;
	struct received_frame frame;

	int ret;

	
	LOG_INF(
		"Receiver started"
	);


	LOG_INF(
		"Waiting for packets..."
	);


	while (1) {

		
		ret = k_msgq_get(
			&receive_queue,
			&frame,
			K_SECONDS(RX_WAIT_TIMEOUT_SECONDS)
		);

		if (ret == -EAGAIN) {
			LOG_INF(
				"No packet callback in the last %d seconds",
				RX_WAIT_TIMEOUT_SECONDS
			);
			continue;
		}

		if (ret != 0) {
			LOG_ERR("RX queue error: %d", ret);
			continue;
		}

		LOG_INF("Packet received with size %d", (int)frame.length);
		printk("RX: ");
		for (size_t i = 0; (i < frame.length) && (i < 20); i++) {
			printk("%02X ", frame.data[i]);
		}
		printk("\n");

		if (!deserialize_packet(frame.data, frame.length, &packet)) {
			LOG_INF("Non-OZ packet ignored");
			continue;
		}
		


		/* -------------------------------------------------
		 * Application destination validation
		 * -------------------------------------------------*/ 

		if (packet.destination_id != RECEIVER_ADDR) {

   			 LOG_INF(
        		"Packet ignored: destination=0x%04X, expected=0x%04X",
        		packet.destination_id,RECEIVER_ADDR );

   				 continue;
		}

		LOG_INF(
			"MAC source = 0x%04X",
			frame.mac_source
		);

		if (frame.rssi_dbm == IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
			LOG_INF(
				"RSSI       = unavailable"
			);
		} else {
			LOG_INF(
				"RSSI       = %d dBm",
				(int)frame.rssi_dbm
			);
		}
		

		print_packet(
			"RECEIVER RX",
			&packet
		);
	}
}

#endif
#endif /* legacy run_receiver */


/* =========================================================
 * Application threads
 * ========================================================= */

static void rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("RX/protocol thread started");

#if NODE_ROLE == ROLE_JOINING
	run_probe_sender();
#elif NODE_ROLE == ROLE_ROOT
	LOG_INF("Root will answer probes and allocate join addresses");
	run_probe_receiver();
#endif
}

static void tx_thread_entry(void *p1, void *p2, void *p3)
{
	struct tx_request request;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("TX thread started");

	while (1) {
		ret = k_msgq_get(&tx_queue, &request, K_FOREVER);
		if (ret != 0) {
			continue;
		}

		(void)radio_send_buffer(
			request.data,
			request.length,
			request.mac_destination
		);
	}
}


/* =========================================================
 * Main — init radio, then start RX / TX threads
 * ========================================================= */

int main(void)
{
	int ret;


	LOG_INF("");

	LOG_INF(
		"========================================"
	);

	LOG_INF(
		"OzturkLab IEEE 802.15.4"
	);

	LOG_INF(
		"Role       : %s",
		role_name()
	);

	LOG_INF(
		"FSM State  : %s",
		node_state_name(current_state)
	);

	LOG_INF(
		"Channel    : %u",
		IEEE802154_CHANNEL
	);

	LOG_INF(
		"PAN ID     : 0x%04X",
		IEEE802154_PAN_ID
	);

	LOG_INF(
		"TX Power   : %d dBm",
		IEEE802154_TX_POWER_DBM
	);

	LOG_INF(
		"App Addr   : %u:%u (0x%04X)%s",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)local_address,
		address_is_root(local_address) ? " [ROOT]" :
		address_is_unassigned(local_address) ? " [UNASSIGNED]" : ""
	);

	LOG_INF(
		"Root Addr  : %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(ROOT_ADDRESS),
		(unsigned int)ADDRESS_NODE(ROOT_ADDRESS),
		(unsigned int)ROOT_ADDRESS
	);

	LOG_INF(
		"Assignable : %u:%u .. %u:%u",
		(unsigned int)ADDRESS_NETWORK(ADDRESS_FIRST_ASSIGNABLE),
		(unsigned int)ADDRESS_NODE(ADDRESS_FIRST_ASSIGNABLE),
		(unsigned int)ADDRESS_NETWORK(ADDRESS_LAST_ASSIGNABLE),
		(unsigned int)ADDRESS_NODE(ADDRESS_LAST_ASSIGNABLE)
	);

	LOG_INF(
		"Reserved   : 1:0  | Unassigned 255:254 | Broadcast 255:255"
	);

	LOG_INF(
		"Parent     : %u:%u | Hop = %u | DATA allowed = %s",
		(unsigned int)ADDRESS_NETWORK(parent_address),
		(unsigned int)ADDRESS_NODE(parent_address),
		(unsigned int)hop_count,
		node_can_send_data() ? "yes" : "no"
	);

	/* Sanity: boot address must match role; pool helpers ready for join. */
	if (address_is_reserved(ADDRESS_FIRST_ASSIGNABLE) ||
	    !address_is_assignable(ADDRESS_FIRST_ASSIGNABLE) ||
	    !address_is_assignable(ADDRESS_LAST_ASSIGNABLE)) {
		LOG_ERR("Address model self-check failed");
		return 0;
	}

	LOG_INF(
		"OZ Payload : %u bytes",
		OZ_PACKET_SIZE
	);

	LOG_INF(
		"HB Payload : %u bytes",
		HEARTBEAT_FRAME_SIZE
	);

	init_device_guid();

	LOG_INF(
		"========================================"
	);


	/* =====================================================
	 * Battery ADC (heartbeat battery level)
	 *
	 * Not fatal: heartbeats report battery as unknown.
	 * ===================================================== */

	(void)init_battery_adc();


	/* =====================================================
	 * Configure IEEE 802.15.4 radio / MAC
	 * ===================================================== */

	ret =
		configure_ieee802154();


	if (ret) {

		LOG_ERR(
			"IEEE 802.15.4 setup failed: %d",
			ret
		);

		return 0;
	}


	/* =====================================================
	 * Create the role-specific networking endpoint
	 * ===================================================== */

	/*
#if NODE_ROLE == ROLE_JOINING
	ret = create_packet_socket();
	if (ret) {
		LOG_ERR("Packet socket setup failed: %d", ret);
		return 0;
	}
#elif NODE_ROLE == ROLE_ROOT
	ret = create_receive_context();
	if (ret) {
		LOG_ERR("Receiver context setup failed: %d", ret);
		return 0;
	}
#endif
	*/

	/*
	 * Both roles send and receive, so both use the single
	 * packet-aware context (RX callback + TX).
	 */
	ret = create_receive_context();
	if (ret) {
		LOG_ERR("Radio context setup failed: %d", ret);
		return 0;
	}


	/* =====================================================
	 * Node FSM: root stays REGISTERED; joining → PROBING
	 * ===================================================== */

	node_fsm_init();


	/* =====================================================
	 * Start RX/protocol and TX threads, then idle.
	 * Role loops never return; main must not call them.
	 * ===================================================== */

	k_thread_create(
		&app_rx_thread,
		app_rx_stack,
		K_THREAD_STACK_SIZEOF(app_rx_stack),
		rx_thread_entry,
		NULL,
		NULL,
		NULL,
		APP_RX_PRIORITY,
		0,
		K_NO_WAIT
	);
	k_thread_name_set(&app_rx_thread, "app_rx");

	k_thread_create(
		&app_tx_thread,
		app_tx_stack,
		K_THREAD_STACK_SIZEOF(app_tx_stack),
		tx_thread_entry,
		NULL,
		NULL,
		NULL,
		APP_TX_PRIORITY,
		0,
		K_NO_WAIT
	);
	k_thread_name_set(&app_tx_thread, "app_tx");

	LOG_INF(
		"Threads started (RX prio %d, TX prio %d)",
		APP_RX_PRIORITY,
		APP_TX_PRIORITY
	);

	return 0;
}
