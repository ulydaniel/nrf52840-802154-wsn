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
 * (not yet implemented; see documentation/ASSIGNMENT_CHECKLIST.md).
 *
 * Node FSM (assignment §5):
 *
 *      UNREGISTERED → PROBING → WAITING_FOR_JOIN_RESPONSE → REGISTERED
 *
 *      Root boots already REGISTERED at 1:254.
 *      Joining nodes boot UNREGISTERED, then enter PROBING.
 *
 * Transitional probe / heartbeat (join REQUEST/RESPONSE still pending):
 *
 *      Joining (PROBING)  --PROBE broadcast-->  Root (REGISTERED)
 *      Joining (PROBING)  <-HEARTBEAT---------  Root (REGISTERED)
 *
 *      Joining node: every K seconds a PROBE is broadcast.
 *      If no HEARTBEAT arrives before the next expiry, K grows
 *      by N. A matching HEARTBEAT resets K.
 *
 *      HEARTBEAT carries: probe RSSI, GUID, battery level
 *      and a heartbeat sequence number (see Message bodies).
 *
 *      Both roles decode received packets and dispatch them
 *      through a function-pointer table indexed by message type.
 *      Only PROBE and HEARTBEAT are implemented; JOIN / NET ID
 *      / DATA entries are reserved for later milestones.
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

#define NODE_ROLE         ROLE_JOINING


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

#define PROTOCOL_VERSION         1
#define MESSAGE_TYPE_DATA        1

#define TX_PERIOD_MS             1000

#define RX_QUEUE_DEPTH           4
#define RX_WAIT_TIMEOUT_SECONDS  5

/*
 * Probe protocol message types. Values are contiguous so
 * (type - MSG_TYPE_FIRST) indexes message_handlers[].
 */
#define MSG_TYPE_PROBE           1
#define MSG_TYPE_HEARTBEAT       2
#define MSG_TYPE_JOIN_REQUEST    3
#define MSG_TYPE_JOIN_RESPONSE   4
#define MSG_TYPE_JOIN_ACK        5
#define MSG_TYPE_NETID_REQUEST   6
#define MSG_TYPE_NETID_RESPONSE  7

#define MSG_TYPE_FIRST           MSG_TYPE_PROBE
#define MSG_TYPE_LAST            MSG_TYPE_NETID_RESPONSE
#define MSG_HANDLER_COUNT        (MSG_TYPE_LAST - MSG_TYPE_FIRST + 1)

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
 * Application payload
 *
 * 12 bytes total
 *
 * Byte  0-1   Signature "OZ"
 * Byte  2     Version
 * Byte  3     Message type
 * Byte  4-5   Original source
 * Byte  6-7   Final destination
 * Byte  8-9   User/sensor value
 * Byte  10-11 Sequence
 *
 * Probe protocol use of the same fields:
 *
 *      PROBE     value    = sender's current K (seconds)
 *      HEARTBEAT value    = RSSI of the probe at receiver
 *      HEARTBEAT sequence = echoes the PROBE sequence
 *
 * Multi-byte application fields are serialized explicitly
 * as little endian.
 * ========================================================= */

#define OZ_PACKET_SIZE   12


struct oz_packet {

	uint8_t signature[2];

	uint8_t version;
	uint8_t message_type;

	uint16_t source_id;
	uint16_t destination_id;

	int16_t value;

	uint16_t sequence;
};


/* =========================================================
 * Message bodies
 *
 * Some message types append a body after the 12-byte OZ
 * header. The header is always parsed first; the handler
 * for the message type then parses its own body.
 *
 * HEARTBEAT body, 13 bytes (frame bytes 12-24):
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
 * receive_context is also used for TX (see send_oz_packet).
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

K_MSGQ_DEFINE(receive_queue,
	      sizeof(struct received_frame),
	      RX_QUEUE_DEPTH,
	      4);
// #endif


/*
 * Used by sender only.
 */
static uint16_t tx_sequence = 1;


/*
 * Probe state. The joining node originates probes until the
 * join FSM takes over. Owned by the main thread, which also
 * runs the message handlers, so no locking is needed.
 */
static unsigned int probe_k_seconds = PROBE_K_INITIAL_SECONDS;
static uint16_t probe_outstanding_sequence;
static bool probe_outstanding;

#if NODE_ROLE == ROLE_JOINING
K_TIMER_DEFINE(probe_timer, NULL, NULL);
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
 * Link statistics, owned by the main thread like the probe
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
 * Note: after PAN ID + short address are both set, Zephyr may
 * treat the node as associated and reject further parameter
 * changes with -EBUSY. Callers that need a later change must
 * be prepared to bring the interface down or reconfigure.
 */
static int ieee802154_apply_short_address(uint16_t addr)
{
	uint16_t value = addr;
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
 * Install a new application address and mirror it to the radio.
 * Returns 0 on success. On L2 failure, local_address is left
 * unchanged.
 *
 * Called by node_enter_registered() / node_revert_to_unregistered()
 * once join assigns or revokes an address (Milestone 5).
 */
static int __maybe_unused set_local_address(uint16_t addr)
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

	set_node_state(NODE_PROBING);
}

/*
 * Join failure / give-up path: drop parent, restore unassigned
 * address context, and return to UNREGISTERED. The caller may
 * immediately call node_enter_probing() to retry.
 */
static void __maybe_unused node_revert_to_unregistered(void)
{
	parent_address = UNASSIGNED_ADDRESS;
	hop_count = HOP_COUNT_UNKNOWN;
	probe_outstanding = false;
	probe_outstanding_sequence = 0;

	if (!address_is_root(BOOT_ADDRESS) &&
	    !address_is_unassigned(local_address)) {
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
 * Join TX / timers land in a later milestone.
 */
static void __maybe_unused node_enter_waiting_for_join_response(
	uint16_t selected_parent,
	uint8_t parent_hop_count)
{
	parent_address = selected_parent;
	/*
	 * Child hop count will be parent_hop + 1 once the response
	 * confirms membership; stash parent hop for now.
	 */
	hop_count = (parent_hop_count == HOP_COUNT_UNKNOWN) ?
		    HOP_COUNT_UNKNOWN :
		    (uint8_t)(parent_hop_count + 1U);

	set_node_state(NODE_WAITING_FOR_JOIN_RESPONSE);
}

/*
 * WAITING_FOR_JOIN_RESPONSE → REGISTERED after JOIN_RESPONSE
 * + local short-address update. JOIN_ACK is sent by the caller.
 */
static void __maybe_unused node_enter_registered(
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
	set_node_state(NODE_REGISTERED);
	log_node_state("Registered");
}

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
	case MSG_TYPE_JOIN_ACK:
		return "JOIN_ACK";
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
		(uint16_t)packet->value,
		&buffer[8]
	);


	sys_put_le16(
		packet->sequence,
		&buffer[10]
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
	 * Exact application payload length required
	 * ----------------------------------------------------- */

	if (length != OZ_PACKET_SIZE) {

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


	packet->value =
		(int16_t)sys_get_le16(
			&buffer[8]
		);


	packet->sequence =
		sys_get_le16(
			&buffer[10]
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
 * Send application packet
 *
 * mac_destination is the IEEE 802.15.4 MAC destination.
 * ========================================================= */

static __maybe_unused int send_oz_packet(
	const struct oz_packet *packet,
	uint16_t mac_destination)
{
	struct sockaddr_ll destination;

	uint8_t buffer[OZ_PACKET_SIZE];

	ssize_t sent;


	/* -----------------------------------------------------
	 * Serialize application payload
	 * ----------------------------------------------------- */

	serialize_packet(
		packet,
		buffer
	);


	/* -----------------------------------------------------
	 * Build destination link-layer address
	 *
	 * Zephyr packet/L2 addresses are represented in
	 * network/big-endian order at this interface.
	 * ----------------------------------------------------- */

	memset(
		&destination,
		0,
		sizeof(destination)
	);


	destination.sll_family =
		AF_PACKET;


	destination.sll_protocol =
		htons(ETH_P_IEEE802154);


	destination.sll_ifindex =
		net_if_get_by_iface(iface);


	destination.sll_halen =
		IEEE802154_SHORT_ADDR_LENGTH;


	sys_put_be16(
		mac_destination,
		destination.sll_addr
	);


	/* -----------------------------------------------------
	 * Send application payload.
	 *
	 * IEEE 802.15.4 L2 creates the MAC frame.
	 * ----------------------------------------------------- */

	printk("TX RAW: ");

	for (int i = 0; i < OZ_PACKET_SIZE; i++) {
		printk("%02X ", buffer[i]);
	}
	printk("\n");

	/*
	sent =
		zsock_sendto(
			packet_socket,
			buffer,
			sizeof(buffer),
			0,
			(const struct sockaddr *)&destination,
			sizeof(destination)
		);
	*/

	/*
	 * Same call a packet socket's sendto() makes internally.
	 * Returns bytes sent or a negative errno value.
	 */
	sent =
		net_context_sendto(
			receive_context,
			buffer,
			sizeof(buffer),
			(const struct sockaddr *)&destination,
			sizeof(destination),
			NULL,
			K_FOREVER,
			NULL
		);


	LOG_INF(
		"sendto() returned %d",(int)sent
	);


	/*
	if (sent < 0) {

		LOG_ERR(
			"sendto() failed, errno=%d",
			errno
		);

		return -errno;
	}
	*/

	if (sent < 0) {

		LOG_ERR(
			"net_context_sendto() failed: %d",
			(int)sent
		);

		return (int)sent;
	}


	if (sent != OZ_PACKET_SIZE) {

		LOG_ERR(
			"Short TX: %d bytes",
			(int)sent
		);

		return -EIO;
	}


	LOG_INF(
		"MAC TX -> 0x%04X, payload=%d bytes",
		mac_destination,
		(int)sent
	);


	return 0;
}


/* =========================================================
 * Send application packet + message body
 *
 * Same as send_oz_packet(), but appends a message-specific
 * body (e.g. the HEARTBEAT body) after the 12-byte header.
 * ========================================================= */

static int send_oz_packet_with_body(
	const struct oz_packet *packet,
	const uint8_t *body,
	size_t body_length,
	uint16_t mac_destination)
{
	struct sockaddr_ll destination;
	uint8_t buffer[OZ_PACKET_SIZE + OZ_MAX_BODY_SIZE];
	size_t length;
	int sent;

	if (body_length > OZ_MAX_BODY_SIZE) {
		LOG_ERR("Body too large: %d bytes", (int)body_length);
		return -EMSGSIZE;
	}

	length = OZ_PACKET_SIZE + body_length;

	serialize_packet(packet, buffer);
	memcpy(&buffer[OZ_PACKET_SIZE], body, body_length);

	/* Destination link-layer address, big-endian (see send_oz_packet). */
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
		K_FOREVER,
		NULL
	);

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
     * Ignore anything that is not our OZ packet.
     */
    if (received != OZ_PACKET_SIZE) {

        LOG_INF(
            "Ignoring packet length %d, expected %d",
            (int)received,
            OZ_PACKET_SIZE
        );

        return -EMSGSIZE;
    }

    if (!deserialize_packet(
            buffer,
            received,
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
 * logging occur in the application thread.
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
	packet->value = value;
	packet->sequence = sequence;
}


/* =========================================================
 * Message handlers
 *
 * Every handler has the same signature so it can be stored
 * in message_handlers[]. Handlers run in the main thread.
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

	/* ret = send_oz_packet(&heartbeat, packet->source_id); */

	ret = send_oz_packet_with_body(
		&heartbeat,
		body_buffer,
		sizeof(body_buffer),
		packet->source_id
	);
	link_stats.heartbeats_sent++;
	if (ret) {
		link_stats.heartbeats_tx_failed++;
		LOG_ERR("HEARTBEAT transmission failed: %d", ret);
	}

	print_link_stats();
}


/*
 * HEARTBEAT: confirms the outstanding probe. A heartbeat whose
 * sequence does not match answers an older probe and is
 * ignored, so a late reply cannot cancel a newer miss.
 */
static void handle_heartbeat(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	struct heartbeat_body body;

	/*
	 * Until parent selection + JOIN land, heartbeats only matter
	 * while PROBING. Later, REGISTERED nodes may also track them.
	 */
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

	if ((!probe_outstanding) ||
	    (packet->sequence != probe_outstanding_sequence)) {
		link_stats.heartbeats_stale++;
		LOG_WRN(
			"Stale/unsolicited HEARTBEAT seq=%u from 0x%04X ignored",
			(unsigned int)packet->sequence,
			(unsigned int)packet->source_id
		);
		return;
	}

	probe_outstanding = false;
	probe_k_seconds = PROBE_K_INITIAL_SECONDS;

	/*
	if (packet->value == IEEE802154_MAC_RSSI_DBM_UNDEFINED) {
		LOG_INF(
			"HEARTBEAT seq=%u from 0x%04X, probe RSSI unavailable",
			(unsigned int)packet->sequence,
			(unsigned int)packet->source_id
		);
	} else {
		LOG_INF(
			"HEARTBEAT seq=%u from 0x%04X, probe RSSI = %d dBm",
			(unsigned int)packet->sequence,
			(unsigned int)packet->source_id,
			(int)packet->value
		);
	}
	*/

	LOG_INF(
		"HEARTBEAT seq=%u from 0x%04X accepted",
		(unsigned int)packet->sequence,
		(unsigned int)packet->source_id
	);

	print_heartbeat_body("HEARTBEAT RX BODY", &body, packet->value);

	LOG_INF("K reset to %u s", probe_k_seconds);

	link_stats.heartbeats_accepted++;
	print_link_stats();
}


/*
 * Indexed by (message_type - MSG_TYPE_FIRST).
 * NULL entries are reserved for handlers not yet implemented.
 */
static const message_handler_t message_handlers[MSG_HANDLER_COUNT] = {
	[MSG_TYPE_PROBE          - MSG_TYPE_FIRST] = handle_probe,
	[MSG_TYPE_HEARTBEAT      - MSG_TYPE_FIRST] = handle_heartbeat,
	[MSG_TYPE_JOIN_REQUEST   - MSG_TYPE_FIRST] = NULL,
	[MSG_TYPE_JOIN_RESPONSE  - MSG_TYPE_FIRST] = NULL,
	[MSG_TYPE_JOIN_ACK       - MSG_TYPE_FIRST] = NULL,
	[MSG_TYPE_NETID_REQUEST  - MSG_TYPE_FIRST] = NULL,
	[MSG_TYPE_NETID_RESPONSE - MSG_TYPE_FIRST] = NULL,
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
		LOG_INF("Non-OZ packet ignored");
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
 * Joining node (transitional probe timer)
 *
 * Until the join FSM is implemented, the joining board still
 * drives PROBE / HEARTBEAT discovery. Probes use the unassigned
 * source address and a broadcast destination (assignment §7.1).
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
	k_ticks_t remaining;
	int ret;

	LOG_INF("Probe sender started (joining)");
	log_node_state("Joining loop");
	LOG_INF(
		"Local       = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)local_address
	);
	LOG_INF(
		"Destination = %u:%u (0x%04X broadcast)",
		(unsigned int)ADDRESS_NETWORK(BROADCAST_ADDRESS),
		(unsigned int)ADDRESS_NODE(BROADCAST_ADDRESS),
		(unsigned int)BROADCAST_ADDRESS
	);
	LOG_INF(
		"K = %u s, N = %u s, K max = %u s",
		PROBE_K_INITIAL_SECONDS,
		PROBE_N_INCREMENT_SECONDS,
		PROBE_K_MAX_SECONDS
	);

	k_timer_start(&probe_timer, K_SECONDS(probe_k_seconds), K_NO_WAIT);

	while (1) {

		/* k_timer_status_get() also clears the expiry count. */
		if (k_timer_status_get(&probe_timer) > 0) {
			probe_timer_expired();
			continue;
		}

		/* Wait for a frame, but no longer than K has left. */
		remaining = k_timer_remaining_ticks(&probe_timer);

		ret = k_msgq_get(&receive_queue, &frame, K_TICKS(remaining));
		if (ret == 0) {
			process_frame(&frame);
		}
	}
}

#endif


/* =========================================================
 * Root node (transitional probe listener / heartbeat reply)
 * ========================================================= */

#if NODE_ROLE == ROLE_ROOT

static void run_probe_receiver(void)
{
	struct received_frame frame;
	int ret;

	LOG_INF("Probe receiver started (root)");
	log_node_state("Root loop");
	LOG_INF(
		"Local       = %u:%u (0x%04X)",
		(unsigned int)ADDRESS_NETWORK(local_address),
		(unsigned int)ADDRESS_NODE(local_address),
		(unsigned int)local_address
	);
	LOG_INF("Waiting for PROBE packets...");

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
 * Main
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
	 * Run configured role
	 * ===================================================== */

#if NODE_ROLE == ROLE_JOINING

	run_probe_sender();

#elif NODE_ROLE == ROLE_ROOT

	LOG_INF("Root will listen for probes (REGISTERED)");

	run_probe_receiver();

#endif


	return 0;
}
