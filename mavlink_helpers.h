#ifndef  _MAVLINK_HELPERS_H_
#define  _MAVLINK_HELPERS_H_

#include "string.h"
#include "checksum.h"
#include "mavlink_types.h"
#include "mavlink_conversions.h"
#include "protocol_c2000.h"

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
	// No m_mavlink_status array defined in function,
	// has to be defined externally
#else
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_status[chan];
}
#endif

/*
 * Internal function to give access to the channel buffer for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_BUFFER
MAVLINK_HELPER mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	
#ifdef MAVLINK_EXTERNAL_RX_BUFFER
	// No m_mavlink_buffer array defined in function,
	// has to be defined externally
#else
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_buffer[chan];
}
#endif

/**
 * @brief Reset the status of a channel.
 *
 * MAVLINK2 PATCH: also resets the version-negotiation flags to the default
 * (transmit MAVLink2; auto-detect on receive), so a channel reset doesn't
 * leave a stale "stuck in MAVLink1 output" state from a previous session.
 */
MAVLINK_HELPER void mavlink_reset_channel_status(uint8_t chan)
{
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	status->parse_state = MAVLINK_PARSE_STATE_IDLE;
	status->flags = 0; // 0 = transmit MAVLink2 by default; see MAVLINK_STATUS_FLAG_* in mavlink_types.h
}

/* ============================================================
 * MAVLINK2 PATCH: mavlink_finalize_message_chan
 * ------------------------------------------------------------
 * Rewritten to:
 *  1) Decide MAVLink1 vs MAVLink2 output from this channel's
 *     status->flags (MAVLINK_STATUS_FLAG_OUT_MAVLINK1), instead
 *     of always producing MAVLink1.
 *  2) Compute the CRC by explicitly accumulating each header
 *     field's value, rather than casting the mavlink_message_t
 *     struct to a byte pointer and walking its raw memory. The
 *     old pointer-cast approach assumed the C struct's in-memory
 *     byte layout exactly matched the wire layout; that assumption
 *     breaks now that msgid is a wider field and the v1/v2 headers
 *     differ in length, and it was already fragile on C2000 (word
 *     addressing) even before this patch. Explicit accumulation
 *     works uniformly for both protocol versions and both the
 *     normal and C2000 builds.
 * ============================================================ */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						      uint8_t chan, uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						      uint8_t chan, uint8_t length)
#endif
{
	mavlink_status_t *chan_status = mavlink_get_channel_status(chan);
	uint8_t use_mavlink1 = (chan_status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) ? 1 : 0;
	uint16_t crc;

	msg->magic = use_mavlink1 ? MAVLINK_STX_MAVLINK1 : MAVLINK_STX;
	msg->len = length;
	msg->incompat_flags = 0; // no packet signing support in this build
	msg->compat_flags = 0;
	msg->sysid = system_id;
	msg->compid = component_id;
	// One sequence number per component
	msg->seq = chan_status->current_tx_seq;
	chan_status->current_tx_seq = chan_status->current_tx_seq + 1;

#if !MAVLINK_C2000
	crc_init(&crc);
	crc_accumulate(msg->len, &crc);
	if (!use_mavlink1) {
		crc_accumulate(msg->incompat_flags, &crc);
		crc_accumulate(msg->compat_flags, &crc);
	}
	crc_accumulate(msg->seq, &crc);
	crc_accumulate(msg->sysid, &crc);
	crc_accumulate(msg->compid, &crc);
	crc_accumulate((uint8_t)(msg->msgid & 0xFF), &crc);
	if (!use_mavlink1) {
		crc_accumulate((uint8_t)((msg->msgid >> 8) & 0xFF), &crc);
		crc_accumulate((uint8_t)((msg->msgid >> 16) & 0xFF), &crc);
	}
	crc_accumulate_buffer(&crc, _MAV_PAYLOAD(msg), msg->len);
#else
	crc_init_c2000(&crc);
	crc_accumulate_c2000(msg->len, &crc);
	if (!use_mavlink1) {
		crc_accumulate_c2000(msg->incompat_flags, &crc);
		crc_accumulate_c2000(msg->compat_flags, &crc);
	}
	crc_accumulate_c2000(msg->seq, &crc);
	crc_accumulate_c2000(msg->sysid, &crc);
	crc_accumulate_c2000(msg->compid, &crc);
	crc_accumulate_c2000((uint8_t)(msg->msgid & 0xFF), &crc);
	if (!use_mavlink1) {
		crc_accumulate_c2000((uint8_t)((msg->msgid >> 8) & 0xFF), &crc);
		crc_accumulate_c2000((uint8_t)((msg->msgid >> 16) & 0xFF), &crc);
	}
	crc_accumulate_msg_payload_c2000(&crc, &(msg->payload64[0]), msg->len);
#endif

#if MAVLINK_CRC_EXTRA
#if !MAVLINK_C2000
	crc_accumulate(crc_extra, &crc);
#else
	crc_accumulate_c2000(crc_extra, &crc);
#endif
#endif
	msg->checksum = crc;

#if !MAVLINK_C2000
	mavlink_ck_a(msg) = (uint8_t)(msg->checksum & 0xFF);
	mavlink_ck_b(msg) = (uint8_t)(msg->checksum >> 8);
#else
	// Insert the checksum into the message payload for C2000 memory alignment
	mav_put_uint8_t_c2000(&(msg->payload64[0]), msg->len, ((msg->checksum >> 8) & 0x00FF));
	mav_put_uint8_t_c2000(&(msg->payload64[0]), msg->len + 1, (msg->checksum & 0x00FF));
#endif

	return length + (use_mavlink1 ? MAVLINK_NUM_NON_PAYLOAD_BYTES_MAVLINK1 : MAVLINK_NUM_NON_PAYLOAD_BYTES);
}


/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						 uint8_t length, uint8_t crc_extra)
{
	return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, length, crc_extra);
}
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						 uint8_t length)
{
	return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, length);
}
#endif

/* ============================================================
 * MAVLINK2 PATCH: mavlink_msg_to_send_buffer
 * ------------------------------------------------------------
 * Rewritten to serialize the header field-by-field into the
 * output buffer instead of memcpy-ing the mavlink_message_t
 * struct's raw memory. That old approach assumed the struct's
 * in-memory layout exactly matched the wire byte order with no
 * padding - which no longer holds now that msgid is a wider
 * field and v1/v2 headers differ in length. This version also
 * replaces the old C2000-only manual byte loop, since the same
 * explicit approach is now used (and needed) for both builds.
 * ============================================================ */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg)
{
	uint8_t use_mavlink1 = (msg->magic == MAVLINK_STX_MAVLINK1) ? 1 : 0;
	uint16_t i = 0;
	uint16_t header_len;

	buffer[i++] = msg->magic;
	buffer[i++] = msg->len;
	if (!use_mavlink1) {
		buffer[i++] = msg->incompat_flags;
		buffer[i++] = msg->compat_flags;
	}
	buffer[i++] = msg->seq;
	buffer[i++] = msg->sysid;
	buffer[i++] = msg->compid;
	buffer[i++] = (uint8_t)(msg->msgid & 0xFF);
	if (!use_mavlink1) {
		buffer[i++] = (uint8_t)((msg->msgid >> 8) & 0xFF);
		buffer[i++] = (uint8_t)((msg->msgid >> 16) & 0xFF);
	}
	header_len = i;

#if !MAVLINK_C2000
	memcpy(&buffer[header_len], _MAV_PAYLOAD(msg), (uint16_t)msg->len);
#else
	{
		int payload_bytes_packed = 0;
		while (payload_bytes_packed < msg->len) {
			buffer[header_len + payload_bytes_packed] =
				mav_get_uint8_t_c2000((void*)(&(msg->payload64[0])), payload_bytes_packed);
			payload_bytes_packed++;
		}
	}
#endif

	buffer[header_len + msg->len]     = (uint8_t)(msg->checksum & 0xFF);
	buffer[header_len + msg->len + 1] = (uint8_t)(msg->checksum >> 8);

	return header_len + (uint16_t)msg->len + MAVLINK_NUM_CHECKSUM_BYTES;
}

union __mavlink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
	crc_init(&msg->checksum);
}

MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
#if !MAVLINK_C2000
	crc_accumulate(c, &msg->checksum);
#else
    crc_accumulate_c2000(c, &msg->checksum);
#endif
}

/* ============================================================
 * MAVLINK2 PATCH: mavlink_parse_char
 * ------------------------------------------------------------
 * The state machine now branches on which magic/STX byte started
 * the current packet (tracked via status->flags &
 * MAVLINK_STATUS_FLAG_IN_MAVLINK1, set the moment MAVLINK_STX or
 * MAVLINK_STX_MAVLINK1 is seen in IDLE) so it can correctly parse
 * either a 5-byte (v1) or 9-byte (v2) core header before falling
 * through to the identical payload/CRC handling both versions
 * share. On successfully receiving a MAVLink2 packet, this also
 * auto-clears MAVLINK_STATUS_FLAG_OUT_MAVLINK1 so future
 * transmissions upgrade to MAVLink2 automatically, matching the
 * behavior of the official MAVLink library.
 * ============================================================ */
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
        /*
	  default message crc function. You can override this per-system to
	  put this data in a different memory segment
	*/
#if MAVLINK_CRC_EXTRA
#ifndef MAVLINK_MESSAGE_CRC
	static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
#define MAVLINK_MESSAGE_CRC(msgid) mavlink_message_crcs[(uint8_t)(msgid)]
#endif
#endif

	mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan); ///< The currently decoded message
	mavlink_status_t* status = mavlink_get_channel_status(chan); ///< The current decode status
	int bufferIndex = 0;

	status->msg_received = 0;

	switch (status->parse_state)
	{
	case MAVLINK_PARSE_STATE_UNINIT:
	case MAVLINK_PARSE_STATE_IDLE:
		if (c == MAVLINK_STX || c == MAVLINK_STX_MAVLINK1)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
			if (c == MAVLINK_STX_MAVLINK1) {
				status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
			} else {
				status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
			}
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:
			if (status->msg_received 
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
				|| c > MAVLINK_MAX_PAYLOAD_LEN
#endif
				)
		{
			status->buffer_overrun++;
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
		}
		else
		{
			// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			rxmsg->len = c;
			status->packet_idx = 0;
			mavlink_update_checksum(rxmsg, c);
			status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:
		if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
		{
			// MAVLink1 header has no flag bytes - this byte is seq
			rxmsg->incompat_flags = 0;
			rxmsg->compat_flags = 0;
			rxmsg->seq = c;
			mavlink_update_checksum(rxmsg, c);
			status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
		}
		else
		{
			rxmsg->incompat_flags = c;
			mavlink_update_checksum(rxmsg, c);
			status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS: // MAVLink2 only
		rxmsg->compat_flags = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS: // MAVLink2 only
		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
		break;

	case MAVLINK_PARSE_STATE_GOT_SEQ:
		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:
		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:
		rxmsg->msgid = (uint32_t)c; // msgid byte 0 (LSB)
		mavlink_update_checksum(rxmsg, c);
		if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
		{
			// MAVLink1: msgid is this one byte only
			if (rxmsg->len == 0)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
			}
			else
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
			}
		}
		else
		{
			// MAVLink2: two more msgid bytes to come
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID1: // MAVLink2 only
		rxmsg->msgid |= ((uint32_t)c) << 8; // msgid byte 1
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID2: // MAVLink2 only
		rxmsg->msgid |= ((uint32_t)c) << 16; // msgid byte 2 (MSB)
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->len == 0)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		else
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID: // msgid complete (v1 or v2) - accumulating payload bytes
#if !MAVLINK_C2000
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
#else
	    mav_put_uint8_t_c2000(&(rxmsg->payload64[0]), status->packet_idx++, c);
#endif

		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
#if MAVLINK_CRC_EXTRA
		mavlink_update_checksum(rxmsg, MAVLINK_MESSAGE_CRC(rxmsg->msgid));
#endif
		if (c != (rxmsg->checksum & 0xFF)) {
			// Check first checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX || c == MAVLINK_STX_MAVLINK1)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				rxmsg->len = 0;
				rxmsg->magic = c;
				if (c == MAVLINK_STX_MAVLINK1) {
					status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
				} else {
					status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
				}
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
#if !MAVLINK_C2000
			_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char)c;
#else
			mav_put_uint8_t_c2000(&(rxmsg->payload64[0]), status->packet_idx, c);
#endif
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_CRC1:
		if (c != (rxmsg->checksum >> 8)) {
			// Check second checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX || c == MAVLINK_STX_MAVLINK1)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				rxmsg->len = 0;
				rxmsg->magic = c;
				if (c == MAVLINK_STX_MAVLINK1) {
					status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
				} else {
					status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
				}
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			// Successfully got message
			status->msg_received = 1;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
#if !MAVLINK_C2000
			_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx+1] = (char)c;
#else
			mav_put_uint8_t_c2000(&(rxmsg->payload64[0]), status->packet_idx + 1, c);
#endif
			memcpy(r_message, rxmsg, sizeof(mavlink_message_t));

			// MAVLINK2 PATCH: auto-upgrade our own transmissions to MAVLink2 the
			// first time we successfully receive a MAVLink2 packet on this channel.
			if (!(rxmsg->magic == MAVLINK_STX_MAVLINK1)) {
				status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
			}
		}
		break;
	}

	bufferIndex++;
	// If a message has been sucessfully decoded, check index
	if (status->msg_received == 1)
	{
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
		// Count this packet as received
		status->packet_rx_success_count++;
	}

	r_mavlink_status->current_rx_seq = status->current_rx_seq+1;
	r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
	r_mavlink_status->packet_rx_drop_count = status->parse_error;
	status->parse_error = 0;
	return status->msg_received;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 */
MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
	uint16_t bits_remain = bits;
	// Transform number into network order
	int32_t v;
	uint8_t i_bit_index, i_byte_index, curr_bits_n;
#if MAVLINK_NEED_BYTE_SWAP
	union {
		int32_t i;
		uint8_t b[4];
	} bin, bout;
	bin.i = b;
	bout.b[0] = bin.b[3];
	bout.b[1] = bin.b[2];
	bout.b[2] = bin.b[1];
	bout.b[3] = bin.b[0];
	v = bout.i;
#else
	v = b;
#endif

	i_bit_index = bit_index;
	i_byte_index = packet_index;
	if (bit_index > 0)
	{
		i_byte_index--;
	}

	while (bits_remain > 0)
	{
		if (bits_remain <= (uint8_t)(8 - i_bit_index))
		{
			curr_bits_n = (uint8_t)bits_remain;
		}
		else
		{
			curr_bits_n = (8 - i_bit_index);
		}
		
		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);
		
		i_bit_index += curr_bits_n;

		bits_remain -= curr_bits_n;
		if (bits_remain > 0)
		{
			i_byte_index++;
			i_bit_index = 0;
		}
	}
	
	*r_bit_index = i_bit_index;
	if (i_bit_index != 7) i_byte_index++;
	return i_byte_index - packet_index;
}

#endif /* _MAVLINK_HELPERS_H_ */
