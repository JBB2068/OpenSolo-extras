# Solo Gimbal: MAVLink1 -> MAVLink2 patch

## Files changed
- `Headers/MAVLink/mavlink_types.h` — replace with the patched version
- `Headers/MAVLink/mavlink_helpers.h` — replace with the patched version
- `Source/mavlink_interface/mavlink_gimbal_interface.c` — **no changes needed**
- `Headers/MAVLink/protocol_c2000.h` — **no changes needed**
- `Headers/MAVLink/checksum.h` — **no changes needed** (assumed to define
  `crc_init`, `crc_accumulate`, `crc_accumulate_buffer`, `crc_calculate` in
  the normal way — I have not seen this file's actual contents, see caveats)

## What changed and why
1. **`mavlink_message_t`** gained `incompat_flags` / `compat_flags` fields
   (present on every MAVLink2 packet) and its `msgid` field widened from
   `uint8_t` to `uint32_t` (MAVLink2 message IDs are 24 bits on the wire).
2. **Header length macros** now exist for both versions
   (`MAVLINK_CORE_HEADER_LEN` = 9 for v2, `MAVLINK_CORE_HEADER_MAVLINK1_LEN`
   = 5 for v1), instead of only the v1 length.
3. **New STX marker** `MAVLINK_STX` (0xFD) added alongside the existing
   `MAVLINK_STX_MAVLINK1` (0xFE, was just called `MAVLINK_STX` before).
4. **Parse state machine** (`mavlink_parse_char`) now branches on which STX
   byte started the packet, to correctly walk either the 5-byte or 9-byte
   header before falling into shared payload/CRC handling.
5. **`mavlink_finalize_message_chan`** now decides which version to send
   based on a per-channel flag, and computes the CRC by explicitly
   accumulating each header byte's *value* rather than casting the message
   struct to a byte pointer and walking its raw memory. That old approach
   assumed the C struct's in-memory layout exactly mirrored the wire byte
   order — already a fragile assumption on C2000 (word-addressed memory),
   and one that would silently break now that the header shape varies by
   version and `msgid` is wider.
6. **`mavlink_msg_to_send_buffer`** rewritten the same way — explicit
   field-by-field serialization instead of a struct memcpy.
7. **Version negotiation**: a channel starts out (`mavlink_reset_channel_status`)
   set to transmit MAVLink2. The first time it successfully receives a
   MAVLink2 packet it also auto-confirms MAVLink2 out (redundant with the
   default, but matches upstream MAVLink library behavior and protects
   against a future default change). It will only transmit MAVLink1 if you
   explicitly set the `MAVLINK_STATUS_FLAG_OUT_MAVLINK1` bit in
   `mavlink_get_channel_status(MAVLINK_COMM_0)->flags` somewhere — nothing
   in the current code does this, so MAVLink2 is what will actually ship.

## Deliberately NOT implemented
- **Packet signing.** MAVLink2 signing is optional. ArduCopter with
  `SERIAL4_PROTOCOL = 2` works fine unsigned. Adding HMAC-SHA256 signing,
  key storage, and timestamp/replay tracking on this DSP is a materially
  bigger and riskier job for very little benefit on a short wired link
  inside one aircraft. If you later want signing, that's a separate,
  scoped piece of work.
- **Payload truncation.** Real MAVLink2 senders trim trailing zero bytes
  from the payload to save bandwidth. Skipped for simplicity — it's a
  bandwidth optimization only, not a correctness requirement. Receivers
  handle untrimmed v2 payloads fine.

## Things I could NOT verify (I don't have these files)
- **`checksum.h`** — I assumed it defines `crc_init`, `crc_accumulate`,
  `crc_accumulate_buffer` with the standard MAVLink X.25 CRC signatures,
  matching how the original `mavlink_helpers.h` already called them.
  If this file has anything version-specific in it, it needs review too.
- **The generated per-message files** (e.g. whatever defines
  `mavlink_msg_heartbeat_pack`, `mavlink_msg_gimbal_control_decode`, etc. —
  presumably one file per message under `Headers/MAVLink/ardupilotmega/` and
  `.../common/`). These should keep working unmodified as long as they only
  ever set `msg->msgid = MAVLINK_MSG_ID_XXX` (a plain assignment, which
  still works with the widened type) and never touch header bytes directly.
  **Please spot-check one or two of these files** for anything that reads
  `msg->msgid` as if it were still a `uint8_t`, or that does raw pointer
  arithmetic on `mavlink_message_t` — if you find any, paste them and I'll
  check.
- **`MAVLINK_USE_CONVENIENCE_FUNCTIONS`** — the original `mavlink_helpers.h`
  had an optional block (`_mavlink_send_uart`, `_mav_finalize_message_chan_send`,
  `_mavlink_resend_uart`) guarded by this macro. `mavlink_gimbal_interface.c`
  doesn't use it (it calls `mavlink_msg_to_send_buffer` + `uart_send_data`
  directly), so I dropped that block rather than porting it blind. **If
  anything else in the codebase defines `MAVLINK_USE_CONVENIENCE_FUNCTIONS`
  or calls those functions, tell me and I'll port that block too** — right
  now, if it's compiled in, it will fail to build (missing functions), which
  is a safe, loud failure rather than a silent wire-format bug.
- **`mavlink_conversions.h`** — not reviewed; nothing in this patch should
  interact with it, but it wasn't shown to me.

## Before flashing to real hardware
1. **Build it.** This alone will surface anything above I couldn't check
   (undefined references, type-mismatch warnings). Do not skip straight to
   flashing on a build that "looks right."
2. **Bench-test on the actual gimbal hardware before flight**, per the
   `OpenSolo-extras` README's own admission that this firmware has never
   been confirmed to run on real hardware — that risk exists independent
   of this patch and applies to the baseline MAVLink1 firmware too.
3. **Test with `SERIAL4_PROTOCOL` still set to 1 (MAVLink1) first.** This
   patch is backward compatible — the gimbal should behave identically to
   before when talking to a MAVLink1 sender. Confirm heartbeat, GoPro
   control, and gimbal angle/rate commands all still work exactly as
   before, on the ground, before changing anything on the flight
   controller side.
4. **Then set `SERIAL4_PROTOCOL` to 2 (MAVLink2)** and re-run the same
   checks. Confirm the gimbal starts sending MAVLink2 (a packet sniffer /
   logic analyzer on the UART, or ArduCopter-side MAVLink inspection, should
   show 0xFD-framed packets from the gimbal instead of 0xFE).
5. Only fly once both directions check out on the bench.
