#pragma once

#include "checksum.h"
#include "mavlink_conversions.h"
#include "mavlink_types.h"
#include "string.h"
#include <stdio.h>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS                                                                                                                                                                 \
    {                                                                                                                                                                                        \
        {0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 43, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0},                       \
            {7, 119, 32, 32, 0, 0, 0}, {8, 117, 36, 36, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0},                 \
            {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 52, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0},           \
            {29, 115, 14, 16, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0},           \
            {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3},                \
            {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 2, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 5, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0},                         \
            {47, 153, 3, 4, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {54, 15, 27, 27, 3, 24, 25},             \
            {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0},        \
            {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 18, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0},            \
            {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {80, 14, 4, 4, 3, 2, 3}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 51, 3, 36, 37},         \
            {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0},         \
            {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0},          \
            {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0},    \
            {108, 32, 84, 84, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 16, 0, 0, 0}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0},       \
            {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11},        \
            {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 57, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0},         \
            {126, 220, 79, 81, 3, 79, 80}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0},   \
            {132, 85, 14, 39, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 16, 0, 0, 0},           \
            {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 16, 0, 0, 0}, \
            {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 54, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0},    \
            {192, 36, 44, 54, 0, 0, 0}, {225, 208, 65, 65, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0},    \
            {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0},        \
            {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0},        \
            {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 54, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0},            \
            {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 235, 0, 0, 0}, {260, 146, 5, 13, 0, 0, 0}, {261, 179, 27, 61, 0, 0, 0}, {262, 12, 18, 22, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0},  \
            {264, 49, 28, 28, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 213, 0, 0, 0},    \
            {270, 59, 19, 19, 0, 0, 0}, {271, 22, 52, 52, 0, 0, 0}, {275, 126, 31, 31, 0, 0, 0}, {276, 18, 49, 49, 0, 0, 0}, {280, 70, 33, 33, 0, 0, 0}, {281, 48, 13, 13, 0, 0, 0},         \
            {282, 123, 35, 35, 3, 32, 33}, {283, 74, 144, 144, 0, 0, 0}, {284, 99, 32, 32, 3, 30, 31}, {285, 137, 40, 40, 3, 38, 39}, {286, 210, 53, 53, 3, 50, 51},                         \
            {287, 1, 23, 23, 3, 20, 21}, {288, 20, 23, 23, 3, 20, 21}, {290, 251, 46, 46, 0, 0, 0}, {291, 10, 57, 57, 0, 0, 0}, {299, 19, 96, 98, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0},     \
            {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0},     \
            {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 232, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0},                          \
            {333, 231, 109, 109, 0, 0, 0}, {334, 72, 10, 10, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {336, 245, 84, 84, 0, 0, 0}, {339, 199, 5, 5, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0},      \
            {350, 232, 20, 252, 0, 0, 0}, {360, 11, 25, 25, 0, 0, 0}, {370, 75, 87, 109, 0, 0, 0}, {373, 117, 42, 42, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {380, 232, 20, 20, 0, 0, 0},  \
            {385, 147, 133, 133, 3, 2, 3}, {386, 132, 16, 16, 3, 4, 5}, {387, 4, 72, 72, 3, 4, 5}, {388, 8, 37, 37, 3, 32, 33}, {390, 156, 238, 238, 0, 0, 0}, {395, 0, 212, 212, 0, 0, 0},  \
            {397, 182, 108, 108, 0, 0, 0}, {400, 110, 254, 254, 3, 4, 5}, {401, 183, 6, 6, 3, 4, 5}, {410, 160, 53, 53, 0, 0, 0}, {411, 106, 3, 3, 0, 0, 0}, {412, 33, 6, 6, 3, 4, 5},       \
            {413, 77, 7, 7, 3, 4, 5}, {9000, 113, 137, 137, 0, 0, 0}, {9005, 117, 34, 34, 0, 0, 0}, {12900, 114, 44, 44, 3, 0, 1}, {12901, 254, 59, 59, 3, 30, 31},                          \
            {12902, 140, 53, 53, 3, 4, 5}, {12903, 249, 46, 46, 3, 0, 1}, {12904, 77, 54, 54, 3, 28, 29}, {12905, 49, 43, 43, 3, 0, 1}, {12915, 94, 249, 249, 3, 0, 1},                      \
            {12920, 20, 5, 5, 0, 0, 0},                                                                                                                                                      \
        {                                                                                                                                                                                    \
            40601, 48, 35, 35, 0, 0, 0                                                                                                                                                       \
        }                                                                                                                                                                                    \
    }
#endif

#include "mavlink_sha256.h"

/*
 * Internal function to give access to the channel status for each channel
 */
#ifndef MAVLINK_GET_CHANNEL_STATUS
static inline mavlink_status_t *mavlink_get_channel_status(uint8_t chan)
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
static inline mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan)
{

#ifdef MAVLINK_EXTERNAL_RX_BUFFER
    // No m_mavlink_buffer array defined in function,
    // has to be defined externally
#else
    static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
    return &m_mavlink_buffer[chan];
}
#endif // MAVLINK_GET_CHANNEL_BUFFER

/* Enable this option to check the length of each message.
    This allows invalid messages to be caught much sooner. Use if the transmission
    medium is prone to missing (or extra) characters (e.g. a radio that fades in
    and out). Only use if the channel will only contain messages types listed in
    the headers.
*/
// #define MAVLINK_CHECK_MESSAGE_LENGTH

/**
 * @brief Reset the status of a channel.
 */
static inline void mavlink_reset_channel_status(uint8_t chan)
{
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
}

/**
 * @brief create a signature block for a packet
 */
static inline uint8_t mavlink_sign_packet(mavlink_signing_t *signing, uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN], const uint8_t *header, uint8_t header_len, const uint8_t *packet,
                                          uint8_t packet_len, const uint8_t crc[2])
{
    mavlink_sha256_ctx ctx;
    union {
        uint64_t t64;
        uint8_t t8[8];
    } tstamp;
    if (signing == NULL || !(signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING))
    {
        return 0;
    }
    signature[0] = signing->link_id;
    tstamp.t64 = signing->timestamp;
    memcpy(&signature[1], tstamp.t8, 6);
    signing->timestamp++;

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
    mavlink_sha256_update(&ctx, header, header_len);
    mavlink_sha256_update(&ctx, packet, packet_len);
    mavlink_sha256_update(&ctx, crc, 2);
    mavlink_sha256_update(&ctx, signature, 7);
    mavlink_sha256_final_48(&ctx, &signature[7]);

    return MAVLINK_SIGNATURE_BLOCK_LEN;
}

/**
 * @brief Trim payload of any trailing zero-populated bytes (MAVLink 2 only).
 *
 * @param payload Serialised payload buffer.
 * @param length Length of full-width payload buffer.
 * @return Length of payload after zero-filled bytes are trimmed.
 */
static inline uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
    while (length > 1 && payload[length - 1] == 0)
    {
        length--;
    }
    return length;
}

/**
 * @brief check a signature block for a packet
 */
static inline bool mavlink_signature_check(mavlink_signing_t *signing, mavlink_signing_streams_t *signing_streams, const mavlink_message_t *msg)
{
    if (signing == NULL)
    {
        return true;
    }
    const uint8_t *p = (const uint8_t *) &msg->magic;
    const uint8_t *psig = msg->signature;
    const uint8_t *incoming_signature = psig + 7;
    mavlink_sha256_ctx ctx;
    uint8_t signature[6];
    uint16_t i;

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
    mavlink_sha256_update(&ctx, p, MAVLINK_NUM_HEADER_BYTES);
    mavlink_sha256_update(&ctx, _MAV_PAYLOAD(msg), msg->len);
    mavlink_sha256_update(&ctx, msg->ck, 2);
    mavlink_sha256_update(&ctx, psig, 1 + 6);
    mavlink_sha256_final_48(&ctx, signature);
    if (memcmp(signature, incoming_signature, 6) != 0)
    {
        return false;
    }

    // now check timestamp
    union tstamp {
        uint64_t t64;
        uint8_t t8[8];
    } tstamp;
    uint8_t link_id = psig[0];
    tstamp.t64 = 0;
    memcpy(tstamp.t8, psig + 1, 6);

    if (signing_streams == NULL)
    {
        return false;
    }

    // find stream
    for (i = 0; i < signing_streams->num_signing_streams; i++)
    {
        if (msg->sysid == signing_streams->stream[i].sysid && msg->compid == signing_streams->stream[i].compid && link_id == signing_streams->stream[i].link_id)
        {
            break;
        }
    }
    if (i == signing_streams->num_signing_streams)
    {
        if (signing_streams->num_signing_streams >= MAVLINK_MAX_SIGNING_STREAMS)
        {
            // over max number of streams
            return false;
        }
        // new stream. Only accept if timestamp is not more than 1 minute old
        if (tstamp.t64 + 6000 * 1000UL < signing->timestamp)
        {
            return false;
        }
        // add new stream
        signing_streams->stream[i].sysid = msg->sysid;
        signing_streams->stream[i].compid = msg->compid;
        signing_streams->stream[i].link_id = link_id;
        signing_streams->num_signing_streams++;
    }
    else
    {
        union tstamp last_tstamp;
        last_tstamp.t64 = 0;
        memcpy(last_tstamp.t8, signing_streams->stream[i].timestamp_bytes, 6);
        if (tstamp.t64 <= last_tstamp.t64)
        {
            // repeating old timestamp
            return false;
        }
    }

    // remember last timestamp
    memcpy(signing_streams->stream[i].timestamp_bytes, psig + 1, 6);

    // our next timestamp must be at least this timestamp
    if (tstamp.t64 > signing->timestamp)
    {
        signing->timestamp = tstamp.t64;
    }
    return true;
}

/**
 * @brief Finalize a MAVLink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length
 */
static inline uint16_t mavlink_finalize_message_buffer(mavlink_message_t *msg, uint8_t system_id, uint8_t component_id, mavlink_status_t *status, uint8_t min_length, uint8_t length,
                                                       uint8_t crc_extra)
{
    bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
    bool signing = (!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
    uint8_t signature_len = signing ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
    uint8_t header_len = MAVLINK_CORE_HEADER_LEN + 1;
    uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
    if (mavlink1)
    {
        msg->magic = MAVLINK_STX_MAVLINK1;
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
    }
    else
    {
        msg->magic = MAVLINK_STX;
    }
    msg->len = mavlink1 ? min_length : _mav_trim_payload(_MAV_PAYLOAD(msg), length);
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->incompat_flags = 0;
    if (signing)
    {
        msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;
    }
    msg->compat_flags = 0;
    msg->seq = status->current_tx_seq;
    status->current_tx_seq = status->current_tx_seq + 1;

    // form the header as a byte array for the crc
    buf[0] = msg->magic;
    buf[1] = msg->len;
    if (mavlink1)
    {
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = msg->msgid & 0xFF;
    }
    else
    {
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = msg->msgid & 0xFF;
        buf[8] = (msg->msgid >> 8) & 0xFF;
        buf[9] = (msg->msgid >> 16) & 0xFF;
    }

    uint16_t checksum = crc_calculate(&buf[1], header_len - 1);
    crc_accumulate_buffer(&checksum, _MAV_PAYLOAD(msg), msg->len);
    crc_accumulate(crc_extra, &checksum);
    mavlink_ck_a(msg) = (uint8_t) (checksum & 0xFF);
    mavlink_ck_b(msg) = (uint8_t) (checksum >> 8);

    msg->checksum = checksum;

    if (signing)
    {
        mavlink_sign_packet(status->signing, msg->signature, (const uint8_t *) buf, header_len, (const uint8_t *) _MAV_PAYLOAD(msg), msg->len,
                            (const uint8_t *) _MAV_PAYLOAD(msg) + (uint16_t) msg->len);
    }

    return msg->len + header_len + 2 + signature_len;
}

static inline uint16_t mavlink_finalize_message_chan(mavlink_message_t *msg, uint8_t system_id, uint8_t component_id, uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    return mavlink_finalize_message_buffer(msg, system_id, component_id, status, min_length, length, crc_extra);
}

/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
static inline uint16_t mavlink_finalize_message(mavlink_message_t *msg, uint8_t system_id, uint8_t component_id, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, min_length, length, crc_extra);
}

static inline void _mav_parse_error(mavlink_status_t *status)
{
    status->parse_error++;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
static inline void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
static inline void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid, const char *packet, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    uint16_t checksum;
    uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
    uint8_t ck[2];
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    uint8_t header_len = MAVLINK_CORE_HEADER_LEN;
    uint8_t signature_len = 0;
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
    bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
    bool signing = (!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);

    if (mavlink1)
    {
        length = min_length;
        if (msgid > 255)
        {
            // can't send 16 bit messages
            _mav_parse_error(status);
            return;
        }
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
        buf[0] = MAVLINK_STX_MAVLINK1;
        buf[1] = length;
        buf[2] = status->current_tx_seq;
        buf[3] = mavlink_system.sysid;
        buf[4] = mavlink_system.compid;
        buf[5] = msgid & 0xFF;
    }
    else
    {
        uint8_t incompat_flags = 0;
        if (signing)
        {
            incompat_flags |= MAVLINK_IFLAG_SIGNED;
        }
        length = _mav_trim_payload(packet, length);
        buf[0] = MAVLINK_STX;
        buf[1] = length;
        buf[2] = incompat_flags;
        buf[3] = 0; // compat_flags
        buf[4] = status->current_tx_seq;
        buf[5] = mavlink_system.sysid;
        buf[6] = mavlink_system.compid;
        buf[7] = msgid & 0xFF;
        buf[8] = (msgid >> 8) & 0xFF;
        buf[9] = (msgid >> 16) & 0xFF;
    }
    status->current_tx_seq++;
    checksum = crc_calculate((const uint8_t *) &buf[1], header_len);
    crc_accumulate_buffer(&checksum, packet, length);
    crc_accumulate(crc_extra, &checksum);
    ck[0] = (uint8_t) (checksum & 0xFF);
    ck[1] = (uint8_t) (checksum >> 8);

    if (signing)
    {
        // possibly add a signature
        signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len + 1, (const uint8_t *) packet, length, ck);
    }

    MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t) length + (uint16_t) signature_len);
    _mavlink_send_uart(chan, (const char *) buf, header_len + 1);
    _mavlink_send_uart(chan, packet, length);
    _mavlink_send_uart(chan, (const char *) ck, 2);
    if (signature_len != 0)
    {
        _mavlink_send_uart(chan, (const char *) signature, signature_len);
    }
    MAVLINK_END_UART_SEND(chan, header_len + 3 + (uint16_t) length + (uint16_t) signature_len);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 * If the message is signed then the original signature is also sent
 */
static inline void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
    uint8_t ck[2];

    ck[0] = (uint8_t) (msg->checksum & 0xFF);
    ck[1] = (uint8_t) (msg->checksum >> 8);
    // XXX use the right sequence here

    uint8_t header_len;
    uint8_t signature_len;

    if (msg->magic == MAVLINK_STX_MAVLINK1)
    {
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
        signature_len = 0;
        MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
        // we can't send the structure directly as it has extra mavlink2 elements in it
        uint8_t buf[MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = msg->msgid & 0xFF;
        _mavlink_send_uart(chan, (const char *) buf, header_len);
    }
    else
    {
        header_len = MAVLINK_CORE_HEADER_LEN + 1;
        signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED) ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
        MAVLINK_START_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
        uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
        buf[0] = msg->magic;
        buf[1] = msg->len;
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = msg->msgid & 0xFF;
        buf[8] = (msg->msgid >> 8) & 0xFF;
        buf[9] = (msg->msgid >> 16) & 0xFF;
        _mavlink_send_uart(chan, (const char *) buf, header_len);
    }
    _mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
    _mavlink_send_uart(chan, (const char *) ck, 2);
    if (signature_len != 0)
    {
        _mavlink_send_uart(chan, (const char *) msg->signature, MAVLINK_SIGNATURE_BLOCK_LEN);
    }
    MAVLINK_END_UART_SEND(chan, header_len + msg->len + 2 + signature_len);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a message to send it over a serial byte stream
 */
static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
    uint8_t signature_len, header_len;
    uint8_t *ck;
    uint8_t length = msg->len;

    if (msg->magic == MAVLINK_STX_MAVLINK1)
    {
        signature_len = 0;
        header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
        buf[0] = msg->magic;
        buf[1] = length;
        buf[2] = msg->seq;
        buf[3] = msg->sysid;
        buf[4] = msg->compid;
        buf[5] = msg->msgid & 0xFF;
        memcpy(&buf[6], _MAV_PAYLOAD(msg), msg->len);
        ck = buf + header_len + 1 + (uint16_t) msg->len;
    }
    else
    {
        length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
        header_len = MAVLINK_CORE_HEADER_LEN;
        buf[0] = msg->magic;
        buf[1] = length;
        buf[2] = msg->incompat_flags;
        buf[3] = msg->compat_flags;
        buf[4] = msg->seq;
        buf[5] = msg->sysid;
        buf[6] = msg->compid;
        buf[7] = msg->msgid & 0xFF;
        buf[8] = (msg->msgid >> 8) & 0xFF;
        buf[9] = (msg->msgid >> 16) & 0xFF;
        memcpy(&buf[10], _MAV_PAYLOAD(msg), length);
        ck = buf + header_len + 1 + (uint16_t) length;
        signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED) ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
    }
    ck[0] = (uint8_t) (msg->checksum & 0xFF);
    ck[1] = (uint8_t) (msg->checksum >> 8);
    if (signature_len > 0)
    {
        memcpy(&ck[2], msg->signature, signature_len);
    }

    return header_len + 1 + 2 + (uint16_t) length + (uint16_t) signature_len;
}

union __mavlink_bitfield {
    uint8_t uint8;
    int8_t int8;
    uint16_t uint16;
    int16_t int16;
    uint32_t uint32;
    int32_t int32;
};

static inline void mavlink_start_checksum(mavlink_message_t *msg)
{
    uint16_t crcTmp = 0;
    crc_init(&crcTmp);
    msg->checksum = crcTmp;
}

static inline void mavlink_update_checksum(mavlink_message_t *msg, uint8_t c)
{
    uint16_t checksum = msg->checksum;
    crc_accumulate(c, &checksum);
    msg->checksum = checksum;
}

/*
  return the crc_entry value for a msgid
*/
#ifndef MAVLINK_GET_MSG_ENTRY
static inline const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid)
{
    static const mavlink_msg_entry_t mavlink_message_crcs[] = MAVLINK_MESSAGE_CRCS;
    /*
  use a bisection search to find the right entry. A perfect hash may be better
  Note that this assumes the table is sorted by msgid
*/
    uint32_t low = 0, high = sizeof(mavlink_message_crcs) / sizeof(mavlink_message_crcs[0]) - 1;
    while (low < high)
    {
        uint32_t mid = (low + 1 + high) / 2;
        if (msgid < mavlink_message_crcs[mid].msgid)
        {
            high = mid - 1;
            continue;
        }
        if (msgid > mavlink_message_crcs[mid].msgid)
        {
            low = mid;
            continue;
        }
        low = mid;
        break;
    }
    if (mavlink_message_crcs[low].msgid != msgid)
    {
        // msgid is not in the table
        return NULL;
    }
    return &mavlink_message_crcs[low];
}
#endif // MAVLINK_GET_MSG_ENTRY

/*
  return the crc_extra value for a message
*/
static inline uint8_t mavlink_get_crc_extra(const mavlink_message_t *msg)
{
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
    return e ? e->crc_extra : 0;
}

/*
  return the min message length
*/
#define MAVLINK_HAVE_MIN_MESSAGE_LENGTH
static inline uint8_t mavlink_min_message_length(const mavlink_message_t *msg)
{
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
    return e ? e->min_msg_len : 0;
}

/*
  return the max message length (including extensions)
*/
#define MAVLINK_HAVE_MAX_MESSAGE_LENGTH
static inline uint8_t mavlink_max_message_length(const mavlink_message_t *msg)
{
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg->msgid);
    return e ? e->max_msg_len : 0;
}

/**
 * This is a variant of mavlink_frame_char() but with caller supplied
 * parsing buffers. It is useful when you want to create a MAVLink
 * parser in a library that doesn't use any global variables
 *
 * @param rxmsg    parsing message buffer
 * @param status   parsing status buffer
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 */
static inline uint8_t mavlink_frame_char_buffer(mavlink_message_t *rxmsg, mavlink_status_t *status, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status)
{
    int bufferIndex = 0;

    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

    switch (status->parse_state)
    {
    case MAVLINK_PARSE_STATE_UNINIT:
    case MAVLINK_PARSE_STATE_IDLE:
        if (c == MAVLINK_STX)
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            rxmsg->len = 0;
            rxmsg->magic = c;
            status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
            mavlink_start_checksum(rxmsg);
        }
        else if (c == MAVLINK_STX_MAVLINK1)
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            rxmsg->len = 0;
            rxmsg->magic = c;
            status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
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
            _mav_parse_error(status);
            status->msg_received = 0;
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        }
        else
        {
            // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
            rxmsg->len = c;
            status->packet_idx = 0;
            mavlink_update_checksum(rxmsg, c);
            if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
            {
                rxmsg->incompat_flags = 0;
                rxmsg->compat_flags = 0;
                status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
            }
            else
            {
                status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
            }
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_LENGTH:
        rxmsg->incompat_flags = c;
        if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0)
        {
            // message includes an incompatible feature flag
            _mav_parse_error(status);
            status->msg_received = 0;
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            break;
        }
        mavlink_update_checksum(rxmsg, c);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
        break;

    case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
        rxmsg->compat_flags = c;
        mavlink_update_checksum(rxmsg, c);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
        break;

    case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
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
        rxmsg->msgid = c;
        mavlink_update_checksum(rxmsg, c);
        if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
        {
            if (rxmsg->len > 0)
            {
                status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
            }
            else
            {
                status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
            if (rxmsg->len < mavlink_min_message_length(rxmsg) || rxmsg->len > mavlink_max_message_length(rxmsg))
            {
                _mav_parse_error(status);
                status->parse_state = MAVLINK_PARSE_STATE_IDLE;
                break;
            }
#endif
        }
        else
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID1:
        rxmsg->msgid |= c << 8;
        mavlink_update_checksum(rxmsg, c);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID2:
        rxmsg->msgid |= ((uint32_t) c) << 16;
        mavlink_update_checksum(rxmsg, c);
        if (rxmsg->len > 0)
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
        }
        else
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
        if (rxmsg->len < mavlink_min_message_length(rxmsg) || rxmsg->len > mavlink_max_message_length(rxmsg))
        {
            _mav_parse_error(status);
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            break;
        }
#endif
        break;

    case MAVLINK_PARSE_STATE_GOT_MSGID3:
        _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char) c;
        mavlink_update_checksum(rxmsg, c);
        if (status->packet_idx == rxmsg->len)
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
        }
        break;

    case MAVLINK_PARSE_STATE_GOT_PAYLOAD: {
        const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);
        uint8_t crc_extra = e ? e->crc_extra : 0;
        mavlink_update_checksum(rxmsg, crc_extra);
        if (c != (rxmsg->checksum & 0xFF))
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
        }
        else
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
        }
        rxmsg->ck[0] = c;

        // zero-fill the packet to cope with short incoming packets
        if (e && status->packet_idx < e->max_msg_len)
        {
            memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0, e->max_msg_len - status->packet_idx);
        }
        break;
    }

    case MAVLINK_PARSE_STATE_GOT_CRC1:
    case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
        if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8))
        {
            // got a bad CRC message
            status->msg_received = MAVLINK_FRAMING_BAD_CRC;
        }
        else
        {
            // Successfully got message
            status->msg_received = MAVLINK_FRAMING_OK;
        }
        rxmsg->ck[1] = c;

        if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED)
        {
            status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
            status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;

            // If the CRC is already wrong, don't overwrite msg_received,
            // otherwise we can end up with garbage flagged as valid.
            if (status->msg_received != MAVLINK_FRAMING_BAD_CRC)
            {
                status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
            }
        }
        else
        {
            if (status->signing && (status->signing->accept_unsigned_callback == NULL || !status->signing->accept_unsigned_callback(status, rxmsg->msgid)))
            {

                // If the CRC is already wrong, don't overwrite msg_received.
                if (status->msg_received != MAVLINK_FRAMING_BAD_CRC)
                {
                    status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
                }
            }
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            if (r_message != NULL)
            {
                memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
            }
        }
        break;
    case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
        rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN - status->signature_wait] = c;
        status->signature_wait--;
        if (status->signature_wait == 0)
        {
            // we have the whole signature, check it is OK
            bool sig_ok = mavlink_signature_check(status->signing, status->signing_streams, rxmsg);
            if (!sig_ok && (status->signing->accept_unsigned_callback && status->signing->accept_unsigned_callback(status, rxmsg->msgid)))
            {
                // accepted via application level override
                sig_ok = true;
            }
            if (sig_ok)
            {
                status->msg_received = MAVLINK_FRAMING_OK;
            }
            else
            {
                status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
            }
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
            if (r_message != NULL)
            {
                memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
            }
        }
        break;
    }

    bufferIndex++;
    // If a message has been successfully decoded, check index
    if (status->msg_received == MAVLINK_FRAMING_OK)
    {
        // while(status->current_seq != rxmsg->seq)
        //{
        //	status->packet_rx_drop_count++;
        //                status->current_seq++;
        // }
        status->current_rx_seq = rxmsg->seq;
        // Initial condition: If no packet has been received so far, drop count is undefined
        if (status->packet_rx_success_count == 0)
            status->packet_rx_drop_count = 0;
        // Count this packet as received
        status->packet_rx_success_count++;
    }

    if (r_message != NULL)
    {
        r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg
    }
    if (r_mavlink_status != NULL)
    {
        r_mavlink_status->parse_state = status->parse_state;
        r_mavlink_status->packet_idx = status->packet_idx;
        r_mavlink_status->current_rx_seq = status->current_rx_seq + 1;
        r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
        r_mavlink_status->packet_rx_drop_count = status->parse_error;
        r_mavlink_status->flags = status->flags;
    }
    status->parse_error = 0;

    if (status->msg_received == MAVLINK_FRAMING_BAD_CRC)
    {
        /*
          the CRC came out wrong. We now need to overwrite the
          msg CRC with the one on the wire so that if the
          caller decides to forward the message anyway that
          mavlink_msg_to_send_buffer() won't overwrite the
          checksum
         */
        if (r_message != NULL)
        {
            r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1] << 8);
        }
    }

    return status->msg_received;
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0, 1 or
 * 2 (MAVLINK_FRAMING_INCOMPLETE, MAVLINK_FRAMING_OK or MAVLINK_FRAMING_BAD_CRC)
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_frame_char(chan, byte, &msg, &status) != MAVLINK_FRAMING_INCOMPLETE)
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
static inline uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status)
{
    return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan), mavlink_get_channel_status(chan), c, r_message, r_mavlink_status);
}

/**
 * Set the protocol version
 */
static inline void mavlink_set_proto_version(uint8_t chan, unsigned int version)
{
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (version > 1)
    {
        status->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }
    else
    {
        status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
}

/**
 * Get the protocol version
 *
 * @return 1 for v1, 2 for v2
 */
static inline unsigned int mavlink_get_proto_version(uint8_t chan)
{
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) > 0)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. This function will return 0 or 1.
 *
 * Messages are parsed into an internal buffer (one for each channel). When a complete
 * message is received it is copies into *r_message and the channel's status is
 * copied into *r_mavlink_status.
 *
 * @param chan     ID of the channel to be parsed.
 *                 A channel is not a physical message channel like a serial port, but a logical partition of
 *                 the communication streams. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to parse
 *
 * @param r_message NULL if no message could be decoded, otherwise the message data
 * @param r_mavlink_status if a message was decoded, this is filled with the channel's stats
 * @return 0 if no message could be decoded or bad CRC, 1 on good message and CRC
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <mavlink.h>
 *
 * mavlink_status_t status;
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg, &status))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
static inline uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);
    if (msg_received == MAVLINK_FRAMING_BAD_CRC || msg_received == MAVLINK_FRAMING_BAD_SIGNATURE)
    {
        // we got a bad CRC. Treat as a parse failure
        mavlink_message_t *rxmsg = mavlink_get_channel_buffer(chan);
        mavlink_status_t *status = mavlink_get_channel_status(chan);

        _mav_parse_error(status);
        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;

        if (c == MAVLINK_STX)
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            rxmsg->len = 0;
            mavlink_start_checksum(rxmsg);
        }

        return 0;
    }

    return msg_received;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 */
static inline uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t *r_bit_index, uint8_t *buffer)
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

    // buffer in
    // 01100000 01000000 00000000 11110001
    // buffer out
    // 11110001 00000000 01000000 01100000

    // Existing partly filled byte (four free slots)
    // 0111xxxx

    // Mask n free bits
    // 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
    // = ((uint32_t)(1 << n)) - 1; // = 2^n - 1

    // Shift n bits into the right position
    // out = in >> n;

    // Mask and shift bytes
    i_bit_index = bit_index;
    i_byte_index = packet_index;
    if (bit_index > 0)
    {
        // If bits were available at start, they were available
        // in the byte before the current index
        i_byte_index--;
    }

    // While bits have not been packed yet
    while (bits_remain > 0)
    {
        // Bits still have to be packed
        // there can be more than 8 bits, so
        // we might have to pack them into more than one byte

        // First pack everything we can into the current 'open' byte
        // curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
        // FIXME
        if (bits_remain <= (uint8_t) (8 - i_bit_index))
        {
            // Enough space
            curr_bits_n = (uint8_t) bits_remain;
        }
        else
        {
            curr_bits_n = (8 - i_bit_index);
        }

        // Pack these n bits into the current byte
        // Mask out whatever was at that position with ones (xxx11111)
        buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
        // Put content to this position, by masking out the non-used part
        buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);

        // Increment the bit index
        i_bit_index += curr_bits_n;

        // Now proceed to the next byte, if necessary
        bits_remain -= curr_bits_n;
        if (bits_remain > 0)
        {
            // Offer another 8 bits / one byte
            i_byte_index++;
            i_bit_index = 0;
        }
    }

    *r_bit_index = i_bit_index;
    // If a partly filled byte is present, mark this as consumed
    if (i_bit_index != 7)
        i_byte_index++;
    return i_byte_index - packet_index;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

// To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time

/*

#include "mavlink_types.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        uart0_transmit(ch);
    }
    if (chan == MAVLINK_COMM_1)
    {
        uart1_transmit(ch);
    }
}
 */

static inline void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
    /* this is the more efficient approach, if the platform
       defines it */
    MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *) buf, len);
#else
    /* fallback to one byte at a time */
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        comm_send_ch(chan, (uint8_t) buf[i]);
    }
#endif
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS
