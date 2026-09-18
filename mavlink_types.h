#ifndef MAVLINK_TYPES_H_
#define MAVLINK_TYPES_H_

// Visual Studio versions before 2013 don't conform to C99.
#if (defined _MSC_VER) && (_MSC_VER < 1800)
#include <stdint.h>
#else
#include <inttypes.h>
#endif

// Macro to define packed structures
#ifdef __GNUC__
  #define MAVPACKED( __Declaration__ ) __Declaration__ __attribute__((packed))
#elif MAVLINK_C2000
  #define MAVPACKED( __Declaration__ ) __Declaration__
#else
  #define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#ifndef MAVLINK_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#endif

/* ============================================================
 * MAVLINK2 PATCH: wire-format markers and header-length macros
 * ------------------------------------------------------------
 * MAVLink1 and MAVLink2 packets are told apart by their first
 * ("magic"/STX) byte, and have different core header lengths.
 * We keep both defined so the parser/packer can support either
 * on the wire, selected per-message via msg->magic.
 * ============================================================ */
#define MAVLINK_STX_MAVLINK1 0xFE ///< Marker to protocol MAVLink 1.0: sync byte, start of a new packet.
#define MAVLINK_STX 0xFD          ///< Marker to protocol MAVLink 2.0: sync byte, start of a new packet.

// MAVLink 1.0 core header: len, seq, sysid, compid, msgid(1 byte)  = 5 bytes
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5
// MAVLink 2.0 core header: len, incompat_flags, compat_flags, seq, sysid, compid, msgid(3 bytes) = 9 bytes
#define MAVLINK_CORE_HEADER_LEN 9

// "Num header bytes" = magic byte + core header. Sized to the larger (v2) case so
// buffers are always big enough for either version.
#define MAVLINK_NUM_HEADER_BYTES_MAVLINK1 (MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1) ///< 6
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1)                  ///< 10

#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)
#define MAVLINK_NUM_NON_PAYLOAD_BYTES_MAVLINK1 (MAVLINK_NUM_HEADER_BYTES_MAVLINK1 + MAVLINK_NUM_CHECKSUM_BYTES)

#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) ///< Maximum packet length

// MAVLink2 incompatibility/compatibility flag bits (only SIGNED is defined by the spec
// today; we don't set it since this patch does not implement packet signing).
#define MAVLINK_IFLAG_SIGNED 0x01

#define MAVLINK_MSG_ID_EXTENDED_MESSAGE 255
#define MAVLINK_EXTENDED_HEADER_LEN 14

#if (defined _MSC_VER) || ((defined __APPLE__) && (defined __MACH__)) || (defined __linux__)
  /* full fledged 32bit++ OS */
  #define MAVLINK_MAX_EXTENDED_PACKET_LEN 65507
#else
  /* small microcontrollers */
  #define MAVLINK_MAX_EXTENDED_PACKET_LEN 2048
#endif

#define MAVLINK_MAX_EXTENDED_PAYLOAD_LEN (MAVLINK_MAX_EXTENDED_PACKET_LEN - MAVLINK_EXTENDED_HEADER_LEN - MAVLINK_NUM_NON_PAYLOAD_BYTES)


/**
 * Old-style 4 byte param union
 */
MAVPACKED(
typedef struct param_union {
	union {
		float param_float;
		int32_t param_int32;
		uint32_t param_uint32;
		int16_t param_int16;
		uint16_t param_uint16;
		int8_t param_int8;
		uint8_t param_uint8;
		uint8_t bytes[4];
	};
	uint8_t type;
}) mavlink_param_union_t;


/**
 * New-style 8 byte param union
 */
MAVPACKED(
typedef union {
    struct {
        uint8_t is_double:1;
        uint8_t mavlink_type:7;
        union {
            char c;
            uint8_t uint8;
            int8_t int8;
            uint16_t uint16;
            int16_t int16;
            uint32_t uint32;
            int32_t int32;
            float f;
            uint8_t align[7];
        };
    };
    uint8_t data[8];
}) mavlink_param_union_double_t;

/**
 * This structure is required to make the mavlink_send_xxx convenience functions
 * work, as it tells the library what the current system and component ID are.
 */
MAVPACKED(
typedef struct __mavlink_system {
    uint8_t sysid;   ///< Used by the MAVLink message_xx_send() convenience function
    uint8_t compid;  ///< Used by the MAVLink message_xx_send() convenience function
}) mavlink_system_t;

/* ============================================================
 * MAVLINK2 PATCH: message struct
 * ------------------------------------------------------------
 * Added incompat_flags / compat_flags (present on every MAVLink2
 * packet, unused/zero for MAVLink1). Widened msgid from uint8_t
 * to uint32_t: MAVLink2 message IDs are 24 bits on the wire.
 * Every message this gimbal actually sends/receives (HEARTBEAT,
 * GIMBAL_CONTROL, GOPRO_*, PARAM_*, COMMAND_LONG, etc.) has an ID
 * under 256, so existing generated mavlink_msg_*_pack() code that
 * does `msg->msgid = MAVLINK_MSG_ID_XXX;` continues to work
 * unmodified - the assignment just promotes to the wider type.
 * ============================================================ */
MAVPACKED(
typedef struct __mavlink_message {
	uint16_t checksum; ///< sent at end of packet
	uint8_t magic;   ///< protocol magic marker: MAVLINK_STX (v2) or MAVLINK_STX_MAVLINK1 (v1)
	uint8_t len;     ///< Length of payload
	uint8_t incompat_flags; ///< MAVLink2 incompatibility flags (0 for MAVLink1 packets)
	uint8_t compat_flags;   ///< MAVLink2 compatibility flags (0 for MAVLink1 packets)
	uint8_t seq;     ///< Sequence of packet
	uint8_t sysid;   ///< ID of message sender system/aircraft
	uint8_t compid;  ///< ID of the message sender component
	uint32_t msgid;  ///< ID of message in payload (up to 24 bits on the wire for MAVLink2)
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
}) mavlink_message_t;

MAVPACKED(
typedef struct __mavlink_extended_message {
       mavlink_message_t base_msg;
       int32_t extended_payload_len;   ///< Length of extended payload if any
       uint8_t extended_payload[MAVLINK_MAX_EXTENDED_PAYLOAD_LEN];
}) mavlink_extended_message_t;

typedef enum {
	MAVLINK_TYPE_CHAR     = 0,
	MAVLINK_TYPE_UINT8_T  = 1,
	MAVLINK_TYPE_INT8_T   = 2,
	MAVLINK_TYPE_UINT16_T = 3,
	MAVLINK_TYPE_INT16_T  = 4,
	MAVLINK_TYPE_UINT32_T = 5,
	MAVLINK_TYPE_INT32_T  = 6,
	MAVLINK_TYPE_UINT64_T = 7,
	MAVLINK_TYPE_INT64_T  = 8,
	MAVLINK_TYPE_FLOAT    = 9,
	MAVLINK_TYPE_DOUBLE   = 10
} mavlink_message_type_t;

#define MAVLINK_MAX_FIELDS 64

typedef struct __mavlink_field_info {
        const char *name;                 // name of this field
        const char *print_format;         // printing format hint, or NULL
        mavlink_message_type_t type;      // type of this field
        unsigned int array_length;        // if non-zero, field is an array
        unsigned int wire_offset;         // offset of each field in the payload
        unsigned int structure_offset;    // offset in a C structure
} mavlink_field_info_t;

// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
typedef struct __mavlink_message_info {
	const char *name;                                      // name of the message
	unsigned num_fields;                                   // how many fields in this message
	mavlink_field_info_t fields[MAVLINK_MAX_FIELDS];       // field information
} mavlink_message_info_t;

#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes
#define mavlink_ck_a(msg) *((msg)->len + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))
#define mavlink_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))

typedef enum {
    MAVLINK_COMM_0,
    MAVLINK_COMM_1,
    MAVLINK_COMM_2,
    MAVLINK_COMM_3
} mavlink_channel_t;

/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#ifndef MAVLINK_COMM_NUM_BUFFERS
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
# define MAVLINK_COMM_NUM_BUFFERS 16
#else
# define MAVLINK_COMM_NUM_BUFFERS 4
#endif
#endif

/* ============================================================
 * MAVLINK2 PATCH: expanded parse state machine
 * ------------------------------------------------------------
 * MAVLink1 and MAVLink2 headers differ in length and content
 * after the length byte, so the state machine branches based on
 * which magic byte started the packet (tracked via
 * MAVLINK_STATUS_FLAG_IN_MAVLINK1, below). MAVLink1 packets skip
 * the two flag-byte states and only consume one msgid byte;
 * MAVLink2 packets consume both flag bytes and three msgid bytes.
 * ============================================================ */
typedef enum {
    MAVLINK_PARSE_STATE_UNINIT=0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS, // MAVLink2 only
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,   // MAVLink2 only
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,         // MAVLink2 only (waiting for msgid byte 2)
    MAVLINK_PARSE_STATE_GOT_MSGID2,         // MAVLink2 only (waiting for msgid byte 3)
    MAVLINK_PARSE_STATE_GOT_MSGID,          // msgid complete (v1: 1 byte, v2: 3 bytes) - now accumulating payload
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state_t; ///< The state machine for the comm parser

/* ============================================================
 * MAVLINK2 PATCH: per-channel version flags
 * ------------------------------------------------------------
 * OUT_MAVLINK1 - set => we TRANSMIT MAVLink1 on this channel.
 *                Cleared by default (i.e. we transmit MAVLink2
 *                by default), and also auto-cleared the first
 *                time we successfully receive a MAVLink2 packet
 *                from the other end (auto-upgrade), matching the
 *                behavior of the official MAVLink library.
 * IN_MAVLINK1   - scratch bit used only while a packet is being
 *                parsed, to remember which header shape to expect
 *                for the packet currently in progress.
 * ============================================================ */
#define MAVLINK_STATUS_FLAG_OUT_MAVLINK1 0x01
#define MAVLINK_STATUS_FLAG_IN_MAVLINK1  0x02

typedef struct __mavlink_status {
    uint8_t msg_received;               ///< Number of received messages
    uint8_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    mavlink_parse_state_t parse_state;  ///< Parsing state machine
    uint8_t packet_idx;                 ///< Index in current packet
    uint8_t current_rx_seq;             ///< Sequence number of last packet received
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t flags;                      ///< MAVLINK_STATUS_FLAG_* bits (MAVLINK2 PATCH)
} mavlink_status_t;

#define MAVLINK_BIG_ENDIAN 0
#define MAVLINK_LITTLE_ENDIAN 1

#endif /* MAVLINK_TYPES_H_ */
