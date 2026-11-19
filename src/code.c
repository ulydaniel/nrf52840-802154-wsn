/*
 * OzturkLab nRF52840
 *
 * IEEE 802.15.4 fixed-channel point-to-point experiment
 *
 * One source file supports:
 *
 *      ROLE_SENDER
 *      ROLE_RECEIVER
 *
 * Topology:
 *
 *      Sender                         Receiver
 *      0x0001   ------------------->  0x0003
 *
 * Probe / heartbeat exchange:
 *
 *      0x0001   ------- PROBE ----->  0x0003
 *      0x0001   <---- HEARTBEAT ----  0x0003
 *
 *      Sender: every K seconds timer K expires and a PROBE
 *      is sent. If the previous PROBE received no HEARTBEAT
 *      by the time the timer expires again, K is incremented
 *      by N. A valid HEARTBEAT resets K to its initial value.
 *
 *      HEARTBEAT carries: probe RSSI, GUID, battery level
 *      and a heartbeat sequence number (see Message bodies).
 *
 *      Both roles decode received packets and dispatch them
 *      through a 7-entry function-pointer table indexed by
 *      message type. Only PROBE and HEARTBEAT are
 *      implemented; the JOIN and NET ID entries are reserved.
 *
 * Channel : 15
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

#define ROLE_SENDER       1
#define ROLE_RECEIVER     2


/*
 * Change ONLY this line for each board:
 *
 * ROLE_SENDER
 * ROLE_RECEIVER
 */

#define NODE_ROLE         ROLE_RECEIVER


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
#define IEEE802154_TX_POWER_DBM   0


/* =========================================================
 * Short addresses
 * ========================================================= */

#define SENDER_ADDR              0x0001
#define RECEIVER_ADDR            0x0003


/* =========================================================
 * Application protocol
 * ========================================================= */

#define PROTOCOL_VERSION         1
#define MESSAGE_TYPE_DATA        1

#define TX_PERIOD_MS             1000

#define RX_QUEUE_DEPTH           4
#define RX_WAIT_TIMEOUT_SECONDS  5

/*
 * Upper bound on net_context_sendto() waiting for a network
 * buffer. Sends also run in the RX thread (PROBE -> HEARTBEAT
 * reply), and K_FOREVER there would stop receive_queue from
 * being drained.
 */
#define TX_SEND_TIMEOUT_MS       100

enum node_state {
    STATE_IDLE,
    STATE_WAITING_FOR_HEARTBEAT,
    STATE_SENDING_JOIN_REQUEST,
    STATE_WAITING_FOR_NETID,
    STATE_JOINED
};

/* =========================================================
 * Thread configuration
 * ========================================================= */

#define RX_STACK_SIZE 2048
#define TX_STACK_SIZE 2048

#define RX_PRIORITY 5
#define TX_PRIORITY 5

/*
 * Static and app_-prefixed: the network stack's net_tc.c
 * already defines global rx_stack / tx_stack symbols.
 */
static K_THREAD_STACK_DEFINE(app_rx_stack, RX_STACK_SIZE);
static K_THREAD_STACK_DEFINE(app_tx_stack, TX_STACK_SIZE);

static struct k_thread app_rx_thread;
static struct k_thread app_tx_thread;

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
 * Role-specific local address
 * ========================================================= */

#if NODE_ROLE == ROLE_SENDER

#define LOCAL_SHORT_ADDR     SENDER_ADDR

#elif NODE_ROLE == ROLE_RECEIVER

#define LOCAL_SHORT_ADDR     RECEIVER_ADDR

#else

#error "Invalid NODE_ROLE"

#endif


/* =========================================================
 * Globals
 * ========================================================= */

static struct net_if *iface;

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
 * Probe state. Only the sender originates probes. Written by
 * the TX thread (run_probe_sender) and the RX thread
 * (handle_heartbeat), so every access holds probe_state_mutex.
 */
static unsigned int probe_k_seconds = PROBE_K_INITIAL_SECONDS;
static uint16_t probe_outstanding_sequence;
static bool probe_outstanding;

/* Shared synchronization objects */
K_SEM_DEFINE(heartbeat_sem, 0, 1); 
K_MUTEX_DEFINE(probe_state_mutex);



/*
 * Heartbeat state: this node's GUID, the battery ADC, and the
 * count of heartbeats this node has sent.
 */
static uint8_t device_guid[GUID_SIZE];

static const struct device *const battery_adc =
	DEVICE_DT_GET(DT_NODELABEL(adc));

static bool battery_adc_ready;

static uint16_t heartbeat_tx_sequence = 1;


/* =========================================================
 * Role name
 * ========================================================= */

static const char *role_name(void)
{
#if NODE_ROLE == ROLE_SENDER

	return "SENDER";

#elif NODE_ROLE == ROLE_RECEIVER

	return "RECEIVER";

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
		"Source        = 0x%04X",
		source
	);

	LOG_INF(
		"Destination   = 0x%04X",
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

    /* Short address */
    value = LOCAL_SHORT_ADDR;

    ret = net_mgmt(
        NET_REQUEST_IEEE802154_SET_SHORT_ADDR,
        iface,
        &value,
        sizeof(value)
    );

    if (ret) {
        LOG_ERR("SET_SHORT_ADDR failed: %d", ret);
        return ret;
    }

    LOG_INF("Short Addr  = 0x%04X", LOCAL_SHORT_ADDR);

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
			K_MSEC(TX_SEND_TIMEOUT_MS),
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
		K_MSEC(TX_SEND_TIMEOUT_MS),
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
	packet->source_id = LOCAL_SHORT_ADDR;
	packet->destination_id = destination_id;
	packet->value = value;
	packet->sequence = sequence;
}


/* =========================================================
 * Message handlers
 *
 * Every handler has the same signature so it can be stored
 * in message_handlers[]. Handlers run in the RX thread.
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

	LOG_INF(
		"PROBE seq=%u from 0x%04X (sender K = %d s)",
		(unsigned int)packet->sequence,
		(unsigned int)packet->source_id,
		(int)packet->value
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
	if (ret) {
		LOG_ERR("HEARTBEAT transmission failed: %d", ret);
	}
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
     * The HEARTBEAT body starts after the 12-byte OZ header.
     */
    if (!deserialize_heartbeat_body(
            &frame->data[OZ_PACKET_SIZE],
            frame->length - OZ_PACKET_SIZE,
            &body)) {

        LOG_WRN(
            "HEARTBEAT seq=%u from 0x%04X ignored: "
            "frame %d bytes, expected %d",
            (unsigned int)packet->sequence,
            (unsigned int)packet->source_id,
            (int)frame->length,
            HEARTBEAT_FRAME_SIZE
        );

        return;
    }

    /*
     * Protect the shared probe state.
     */
    k_mutex_lock(&probe_state_mutex, K_FOREVER);

    /*
     * Accept the HEARTBEAT only if:
     *
     * 1. A probe is currently outstanding.
     * 2. The HEARTBEAT sequence matches that probe.
     */
    if ((!probe_outstanding) || (packet->sequence != probe_outstanding_sequence)) {

        k_mutex_unlock(&probe_state_mutex);

        LOG_WRN(
            "Stale/unsolicited HEARTBEAT seq=%u "
            "from 0x%04X ignored",
            (unsigned int)packet->sequence,
            (unsigned int)packet->source_id
        );

        return;
    }

    /*
     * The correct HEARTBEAT arrived.
     */
    probe_outstanding = false;
    probe_k_seconds = PROBE_K_INITIAL_SECONDS;

    /*
     * Give inside the lock: if the give happened after the
     * unlock, the TX thread could open the next probe (and
     * reset the semaphore) first, and this give would then
     * satisfy the new probe's wait. k_sem_give() never blocks.
     */
    k_sem_give(&heartbeat_sem);

    k_mutex_unlock(&probe_state_mutex);

    LOG_INF(
        "HEARTBEAT seq=%u from 0x%04X accepted",
        (unsigned int)packet->sequence,
        (unsigned int)packet->source_id
    );

    print_heartbeat_body(
        "HEARTBEAT RX BODY",
        &body,
        packet->value
    );

    LOG_INF(
        "K reset to %u s",
        PROBE_K_INITIAL_SECONDS
    );
}


static __maybe_unused void handle_heartbeatold(
	const struct oz_packet *packet,
	const struct received_frame *frame)
{
	struct heartbeat_body body;

	/* ARG_UNUSED(frame); */

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

	if ((!probe_outstanding) ||
	    (packet->sequence != probe_outstanding_sequence)) {
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

	if (packet.destination_id != LOCAL_SHORT_ADDR) {
		LOG_INF(
			"Packet ignored: destination=0x%04X, expected=0x%04X",
			(unsigned int)packet.destination_id,
			LOCAL_SHORT_ADDR
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
 * Sender role (probe timer)
 * ========================================================= */


static void run_probe_sender(void)
{
    struct oz_packet probe;
    int ret;

    LOG_INF("Probe sender started");
    LOG_INF("Destination = 0x%04X", RECEIVER_ADDR);
    LOG_INF(
        "K = %u s, N = %u s, K max = %u s",
        PROBE_K_INITIAL_SECONDS,
        PROBE_N_INCREMENT_SECONDS,
        PROBE_K_MAX_SECONDS
    );

    while (1) {
        build_packet(
            &probe,
            MSG_TYPE_PROBE,
            RECEIVER_ADDR,
            (int16_t)probe_k_seconds,
            tx_sequence++
        );

        /*
         * Mark the probe as outstanding before sending.
         *
         * A HEARTBEAT for the previous probe can arrive just
         * after its k_sem_take() timed out and give the
         * semaphore. Reset it under the same lock that opens
         * the new probe so that stale give cannot satisfy the
         * wait below.
         */
        k_mutex_lock(&probe_state_mutex, K_FOREVER);

        probe_outstanding = true;
        probe_outstanding_sequence = probe.sequence;
        k_sem_reset(&heartbeat_sem);

        k_mutex_unlock(&probe_state_mutex);

        ret = send_oz_packet(
            &probe,
            RECEIVER_ADDR
        );

        if (ret != 0) {
            LOG_ERR("PROBE transmission failed: %d", ret);
        }

        /*
         * Sleep until the matching HEARTBEAT arrives
         * or until K seconds pass.
         */
        ret = k_sem_take(
            &heartbeat_sem,
            K_SECONDS(probe_k_seconds)
        ); // Wait for the heartbeat semaphore with a timeout of K seconds

        if (ret == 0) {
            /*
             * Heartbeat arrived.
             */
            k_mutex_lock(&probe_state_mutex, K_FOREVER);

            probe_outstanding = false; // Clear the outstanding flag
            probe_k_seconds = PROBE_K_INITIAL_SECONDS;

            k_mutex_unlock(&probe_state_mutex);

            LOG_INF("HEARTBEAT received");
        } else {
            /*
             * Timeout: no heartbeat arrived.
             */
            k_mutex_lock(&probe_state_mutex, K_FOREVER);

            probe_outstanding = false;
            probe_k_seconds += PROBE_N_INCREMENT_SECONDS;

            if (probe_k_seconds > PROBE_K_MAX_SECONDS) {
                probe_k_seconds = PROBE_K_MAX_SECONDS;
            }

            k_mutex_unlock(&probe_state_mutex);

            LOG_WRN("HEARTBEAT timeout");
        }
    }
}





/* =========================================================
 * Receiver role (probe listener)
 * ========================================================= */



static void run_probe_receiver(void)
{
	struct received_frame frame;
	int ret;

	LOG_INF("Probe receiver started");
	LOG_INF("Waiting for PROBE packets...");


	while (1) {
        // Get a received frame from the message queue, waiting up to RX_WAIT_TIMEOUT_SECONDS.
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


static void rx_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("RX thread started");

    run_probe_receiver();
}

static void tx_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("TX thread started");

    run_probe_sender();
}



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
		"Short Addr : 0x%04X",
		LOCAL_SHORT_ADDR
	);

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
	 * Both roles send and receive, so both use the single
	 * packet-aware context (RX callback + TX).
	 */
	ret = create_receive_context();
	if (ret) {
		LOG_ERR("Radio context setup failed: %d", ret);
		return 0;
	}

    k_thread_create(
        &app_rx_thread,
        app_rx_stack,
        K_THREAD_STACK_SIZEOF(app_rx_stack),
        rx_thread_entry,
        NULL,
        NULL,
        NULL,
        RX_PRIORITY,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &app_tx_thread,
        app_tx_stack,
        K_THREAD_STACK_SIZEOF(app_tx_stack),
        tx_thread_entry,
        NULL,
        NULL,
        NULL,
        TX_PRIORITY,
        0,
        K_NO_WAIT
    );




	return 0;
}