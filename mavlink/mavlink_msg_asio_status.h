#pragma once

#include "protocol.h"

// MESSAGE ASIO_STATUS PACKING

#define MAVLINK_MSG_ID_ASIO_STATUS 40601


typedef struct __mavlink_asio_status_t {
 uint32_t error; /*<  */
 float pitch_offset; /*< [deg] Pitch offset*/
 float roll_offset; /*< [deg] Roll offset*/
 float reserved_2; /*<  */
 uint32_t version; /*<  asio software version*/
 int16_t temperature; /*< [cdegC] Internal temperature of the system, set to -1 if invalid*/
 uint8_t nav_state; /*<  */
 uint8_t nav_output; /*<  */
 uint8_t gps_jamming_state; /*<  */
 uint8_t active_sensor; /*<  */
 uint8_t active_rec; /*<  */
 uint8_t active_db_rec; /*<  */
 uint8_t active_nav; /*<  Current NAV_MODE*/
 uint8_t active_slam; /*<  Current SLAM_MODE*/
 uint8_t battery; /*< [%] Set to 255 for unknown/irrelevant*/
 uint8_t sys_state; /*<  Current system state*/
 uint8_t reserved; /*<  */
 uint8_t map_warning; /*<  */
 uint8_t calibration_state; /*<  */
} mavlink_asio_status_t;

#define MAVLINK_MSG_ID_ASIO_STATUS_LEN 35
#define MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN 35
#define MAVLINK_MSG_ID_40601_LEN 35
#define MAVLINK_MSG_ID_40601_MIN_LEN 35

#define MAVLINK_MSG_ID_ASIO_STATUS_CRC 48
#define MAVLINK_MSG_ID_40601_CRC 48



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASIO_STATUS { \
    40601, \
    "ASIO_STATUS", \
    19, \
    {  { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_asio_status_t, nav_state) }, \
         { "nav_output", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_asio_status_t, nav_output) }, \
         { "gps_jamming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_asio_status_t, gps_jamming_state) }, \
         { "active_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_asio_status_t, active_sensor) }, \
         { "active_rec", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_asio_status_t, active_rec) }, \
         { "active_db_rec", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_asio_status_t, active_db_rec) }, \
         { "active_nav", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_asio_status_t, active_nav) }, \
         { "active_slam", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_asio_status_t, active_slam) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_asio_status_t, battery) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_asio_status_t, temperature) }, \
         { "sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_asio_status_t, sys_state) }, \
         { "error", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_asio_status_t, error) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asio_status_t, reserved) }, \
         { "map_warning", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_asio_status_t, map_warning) }, \
         { "pitch_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_asio_status_t, pitch_offset) }, \
         { "roll_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_asio_status_t, roll_offset) }, \
         { "calibration_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_asio_status_t, calibration_state) }, \
         { "reserved_2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_asio_status_t, reserved_2) }, \
         { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_asio_status_t, version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASIO_STATUS { \
    "ASIO_STATUS", \
    19, \
    {  { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_asio_status_t, nav_state) }, \
         { "nav_output", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_asio_status_t, nav_output) }, \
         { "gps_jamming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_asio_status_t, gps_jamming_state) }, \
         { "active_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_asio_status_t, active_sensor) }, \
         { "active_rec", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_asio_status_t, active_rec) }, \
         { "active_db_rec", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_asio_status_t, active_db_rec) }, \
         { "active_nav", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_asio_status_t, active_nav) }, \
         { "active_slam", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_asio_status_t, active_slam) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_asio_status_t, battery) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_asio_status_t, temperature) }, \
         { "sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_asio_status_t, sys_state) }, \
         { "error", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_asio_status_t, error) }, \
         { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_asio_status_t, reserved) }, \
         { "map_warning", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_asio_status_t, map_warning) }, \
         { "pitch_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_asio_status_t, pitch_offset) }, \
         { "roll_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_asio_status_t, roll_offset) }, \
         { "calibration_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_asio_status_t, calibration_state) }, \
         { "reserved_2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_asio_status_t, reserved_2) }, \
         { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_asio_status_t, version) }, \
         } \
}
#endif

/**
 * @brief Pack a asio_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nav_state
 * @param nav_output
 * @param gps_jamming_state
 * @param active_sensor
 * @param active_rec
 * @param active_db_rec
 * @param active_nav  Current NAV_MODE
 * @param active_slam  Current SLAM_MODE
 * @param battery [%] Set to 255 for unknown/irrelevant
 * @param temperature [cdegC] Internal temperature of the system, set to -1 if invalid
 * @param sys_state  Current system state
 * @param error
 * @param reserved
 * @param map_warning
 * @param pitch_offset [deg] Pitch offset
 * @param roll_offset [deg] Roll offset
 * @param calibration_state
 * @param reserved_2
 * @param version  asio software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asio_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t nav_state, uint8_t nav_output, uint8_t gps_jamming_state, uint8_t active_sensor, uint8_t active_rec, uint8_t active_db_rec, uint8_t active_nav, uint8_t active_slam, uint8_t battery, int16_t temperature, uint8_t sys_state, uint32_t error, uint8_t reserved, uint8_t map_warning, float pitch_offset, float roll_offset, uint8_t calibration_state, float reserved_2, uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASIO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error);
    _mav_put_float(buf, 4, pitch_offset);
    _mav_put_float(buf, 8, roll_offset);
    _mav_put_float(buf, 12, reserved_2);
    _mav_put_uint32_t(buf, 16, version);
    _mav_put_int16_t(buf, 20, temperature);
    _mav_put_uint8_t(buf, 22, nav_state);
    _mav_put_uint8_t(buf, 23, nav_output);
    _mav_put_uint8_t(buf, 24, gps_jamming_state);
    _mav_put_uint8_t(buf, 25, active_sensor);
    _mav_put_uint8_t(buf, 26, active_rec);
    _mav_put_uint8_t(buf, 27, active_db_rec);
    _mav_put_uint8_t(buf, 28, active_nav);
    _mav_put_uint8_t(buf, 29, active_slam);
    _mav_put_uint8_t(buf, 30, battery);
    _mav_put_uint8_t(buf, 31, sys_state);
    _mav_put_uint8_t(buf, 32, reserved);
    _mav_put_uint8_t(buf, 33, map_warning);
    _mav_put_uint8_t(buf, 34, calibration_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASIO_STATUS_LEN);
#else
    mavlink_asio_status_t packet;
    packet.error = error;
    packet.pitch_offset = pitch_offset;
    packet.roll_offset = roll_offset;
    packet.reserved_2 = reserved_2;
    packet.version = version;
    packet.temperature = temperature;
    packet.nav_state = nav_state;
    packet.nav_output = nav_output;
    packet.gps_jamming_state = gps_jamming_state;
    packet.active_sensor = active_sensor;
    packet.active_rec = active_rec;
    packet.active_db_rec = active_db_rec;
    packet.active_nav = active_nav;
    packet.active_slam = active_slam;
    packet.battery = battery;
    packet.sys_state = sys_state;
    packet.reserved = reserved;
    packet.map_warning = map_warning;
    packet.calibration_state = calibration_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASIO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASIO_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
}

/**
 * @brief Pack a asio_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_state
 * @param nav_output
 * @param gps_jamming_state
 * @param active_sensor
 * @param active_rec
 * @param active_db_rec
 * @param active_nav  Current NAV_MODE
 * @param active_slam  Current SLAM_MODE
 * @param battery [%] Set to 255 for unknown/irrelevant
 * @param temperature [cdegC] Internal temperature of the system, set to -1 if invalid
 * @param sys_state  Current system state
 * @param error
 * @param reserved
 * @param map_warning
 * @param pitch_offset [deg] Pitch offset
 * @param roll_offset [deg] Roll offset
 * @param calibration_state
 * @param reserved_2
 * @param version  asio software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_asio_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t nav_state,uint8_t nav_output,uint8_t gps_jamming_state,uint8_t active_sensor,uint8_t active_rec,uint8_t active_db_rec,uint8_t active_nav,uint8_t active_slam,uint8_t battery,int16_t temperature,uint8_t sys_state,uint32_t error,uint8_t reserved,uint8_t map_warning,float pitch_offset,float roll_offset,uint8_t calibration_state,float reserved_2,uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASIO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error);
    _mav_put_float(buf, 4, pitch_offset);
    _mav_put_float(buf, 8, roll_offset);
    _mav_put_float(buf, 12, reserved_2);
    _mav_put_uint32_t(buf, 16, version);
    _mav_put_int16_t(buf, 20, temperature);
    _mav_put_uint8_t(buf, 22, nav_state);
    _mav_put_uint8_t(buf, 23, nav_output);
    _mav_put_uint8_t(buf, 24, gps_jamming_state);
    _mav_put_uint8_t(buf, 25, active_sensor);
    _mav_put_uint8_t(buf, 26, active_rec);
    _mav_put_uint8_t(buf, 27, active_db_rec);
    _mav_put_uint8_t(buf, 28, active_nav);
    _mav_put_uint8_t(buf, 29, active_slam);
    _mav_put_uint8_t(buf, 30, battery);
    _mav_put_uint8_t(buf, 31, sys_state);
    _mav_put_uint8_t(buf, 32, reserved);
    _mav_put_uint8_t(buf, 33, map_warning);
    _mav_put_uint8_t(buf, 34, calibration_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASIO_STATUS_LEN);
#else
    mavlink_asio_status_t packet;
    packet.error = error;
    packet.pitch_offset = pitch_offset;
    packet.roll_offset = roll_offset;
    packet.reserved_2 = reserved_2;
    packet.version = version;
    packet.temperature = temperature;
    packet.nav_state = nav_state;
    packet.nav_output = nav_output;
    packet.gps_jamming_state = gps_jamming_state;
    packet.active_sensor = active_sensor;
    packet.active_rec = active_rec;
    packet.active_db_rec = active_db_rec;
    packet.active_nav = active_nav;
    packet.active_slam = active_slam;
    packet.battery = battery;
    packet.sys_state = sys_state;
    packet.reserved = reserved;
    packet.map_warning = map_warning;
    packet.calibration_state = calibration_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASIO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASIO_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
}

/**
 * @brief Encode a asio_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param asio_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asio_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_asio_status_t* asio_status)
{
    return mavlink_msg_asio_status_pack(system_id, component_id, msg, asio_status->nav_state, asio_status->nav_output, asio_status->gps_jamming_state, asio_status->active_sensor, asio_status->active_rec, asio_status->active_db_rec, asio_status->active_nav, asio_status->active_slam, asio_status->battery, asio_status->temperature, asio_status->sys_state, asio_status->error, asio_status->reserved, asio_status->map_warning, asio_status->pitch_offset, asio_status->roll_offset, asio_status->calibration_state, asio_status->reserved_2, asio_status->version);
}

/**
 * @brief Encode a asio_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param asio_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_asio_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_asio_status_t* asio_status)
{
    return mavlink_msg_asio_status_pack_chan(system_id, component_id, chan, msg, asio_status->nav_state, asio_status->nav_output, asio_status->gps_jamming_state, asio_status->active_sensor, asio_status->active_rec, asio_status->active_db_rec, asio_status->active_nav, asio_status->active_slam, asio_status->battery, asio_status->temperature, asio_status->sys_state, asio_status->error, asio_status->reserved, asio_status->map_warning, asio_status->pitch_offset, asio_status->roll_offset, asio_status->calibration_state, asio_status->reserved_2, asio_status->version);
}

/**
 * @brief Send a asio_status message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_state
 * @param nav_output
 * @param gps_jamming_state
 * @param active_sensor
 * @param active_rec
 * @param active_db_rec
 * @param active_nav  Current NAV_MODE
 * @param active_slam  Current SLAM_MODE
 * @param battery [%] Set to 255 for unknown/irrelevant
 * @param temperature [cdegC] Internal temperature of the system, set to -1 if invalid
 * @param sys_state  Current system state
 * @param error
 * @param reserved
 * @param map_warning
 * @param pitch_offset [deg] Pitch offset
 * @param roll_offset [deg] Roll offset
 * @param calibration_state
 * @param reserved_2
 * @param version  asio software version
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_asio_status_send(mavlink_channel_t chan, uint8_t nav_state, uint8_t nav_output, uint8_t gps_jamming_state, uint8_t active_sensor, uint8_t active_rec, uint8_t active_db_rec, uint8_t active_nav, uint8_t active_slam, uint8_t battery, int16_t temperature, uint8_t sys_state, uint32_t error, uint8_t reserved, uint8_t map_warning, float pitch_offset, float roll_offset, uint8_t calibration_state, float reserved_2, uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASIO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, error);
    _mav_put_float(buf, 4, pitch_offset);
    _mav_put_float(buf, 8, roll_offset);
    _mav_put_float(buf, 12, reserved_2);
    _mav_put_uint32_t(buf, 16, version);
    _mav_put_int16_t(buf, 20, temperature);
    _mav_put_uint8_t(buf, 22, nav_state);
    _mav_put_uint8_t(buf, 23, nav_output);
    _mav_put_uint8_t(buf, 24, gps_jamming_state);
    _mav_put_uint8_t(buf, 25, active_sensor);
    _mav_put_uint8_t(buf, 26, active_rec);
    _mav_put_uint8_t(buf, 27, active_db_rec);
    _mav_put_uint8_t(buf, 28, active_nav);
    _mav_put_uint8_t(buf, 29, active_slam);
    _mav_put_uint8_t(buf, 30, battery);
    _mav_put_uint8_t(buf, 31, sys_state);
    _mav_put_uint8_t(buf, 32, reserved);
    _mav_put_uint8_t(buf, 33, map_warning);
    _mav_put_uint8_t(buf, 34, calibration_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASIO_STATUS, buf, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
#else
    mavlink_asio_status_t packet;
    packet.error = error;
    packet.pitch_offset = pitch_offset;
    packet.roll_offset = roll_offset;
    packet.reserved_2 = reserved_2;
    packet.version = version;
    packet.temperature = temperature;
    packet.nav_state = nav_state;
    packet.nav_output = nav_output;
    packet.gps_jamming_state = gps_jamming_state;
    packet.active_sensor = active_sensor;
    packet.active_rec = active_rec;
    packet.active_db_rec = active_db_rec;
    packet.active_nav = active_nav;
    packet.active_slam = active_slam;
    packet.battery = battery;
    packet.sys_state = sys_state;
    packet.reserved = reserved;
    packet.map_warning = map_warning;
    packet.calibration_state = calibration_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASIO_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
#endif
}

/**
 * @brief Send a asio_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_asio_status_send_struct(mavlink_channel_t chan, const mavlink_asio_status_t* asio_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_asio_status_send(chan, asio_status->nav_state, asio_status->nav_output, asio_status->gps_jamming_state, asio_status->active_sensor, asio_status->active_rec, asio_status->active_db_rec, asio_status->active_nav, asio_status->active_slam, asio_status->battery, asio_status->temperature, asio_status->sys_state, asio_status->error, asio_status->reserved, asio_status->map_warning, asio_status->pitch_offset, asio_status->roll_offset, asio_status->calibration_state, asio_status->reserved_2, asio_status->version);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASIO_STATUS, (const char *)asio_status, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASIO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_asio_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t nav_state, uint8_t nav_output, uint8_t gps_jamming_state, uint8_t active_sensor, uint8_t active_rec, uint8_t active_db_rec, uint8_t active_nav, uint8_t active_slam, uint8_t battery, int16_t temperature, uint8_t sys_state, uint32_t error, uint8_t reserved, uint8_t map_warning, float pitch_offset, float roll_offset, uint8_t calibration_state, float reserved_2, uint32_t version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, error);
    _mav_put_float(buf, 4, pitch_offset);
    _mav_put_float(buf, 8, roll_offset);
    _mav_put_float(buf, 12, reserved_2);
    _mav_put_uint32_t(buf, 16, version);
    _mav_put_int16_t(buf, 20, temperature);
    _mav_put_uint8_t(buf, 22, nav_state);
    _mav_put_uint8_t(buf, 23, nav_output);
    _mav_put_uint8_t(buf, 24, gps_jamming_state);
    _mav_put_uint8_t(buf, 25, active_sensor);
    _mav_put_uint8_t(buf, 26, active_rec);
    _mav_put_uint8_t(buf, 27, active_db_rec);
    _mav_put_uint8_t(buf, 28, active_nav);
    _mav_put_uint8_t(buf, 29, active_slam);
    _mav_put_uint8_t(buf, 30, battery);
    _mav_put_uint8_t(buf, 31, sys_state);
    _mav_put_uint8_t(buf, 32, reserved);
    _mav_put_uint8_t(buf, 33, map_warning);
    _mav_put_uint8_t(buf, 34, calibration_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASIO_STATUS, buf, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
#else
    mavlink_asio_status_t *packet = (mavlink_asio_status_t *)msgbuf;
    packet->error = error;
    packet->pitch_offset = pitch_offset;
    packet->roll_offset = roll_offset;
    packet->reserved_2 = reserved_2;
    packet->version = version;
    packet->temperature = temperature;
    packet->nav_state = nav_state;
    packet->nav_output = nav_output;
    packet->gps_jamming_state = gps_jamming_state;
    packet->active_sensor = active_sensor;
    packet->active_rec = active_rec;
    packet->active_db_rec = active_db_rec;
    packet->active_nav = active_nav;
    packet->active_slam = active_slam;
    packet->battery = battery;
    packet->sys_state = sys_state;
    packet->reserved = reserved;
    packet->map_warning = map_warning;
    packet->calibration_state = calibration_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASIO_STATUS, (const char *)packet, MAVLINK_MSG_ID_ASIO_STATUS_MIN_LEN, MAVLINK_MSG_ID_ASIO_STATUS_LEN, MAVLINK_MSG_ID_ASIO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ASIO_STATUS UNPACKING


/**
 * @brief Get field nav_state from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_nav_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field nav_output from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_nav_output(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field gps_jamming_state from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_gps_jamming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field active_sensor from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_active_sensor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field active_rec from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_active_rec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field active_db_rec from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_active_db_rec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field active_nav from asio_status message
 *
 * @return  Current NAV_MODE
 */
static inline uint8_t mavlink_msg_asio_status_get_active_nav(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field active_slam from asio_status message
 *
 * @return  Current SLAM_MODE
 */
static inline uint8_t mavlink_msg_asio_status_get_active_slam(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field battery from asio_status message
 *
 * @return [%] Set to 255 for unknown/irrelevant
 */
static inline uint8_t mavlink_msg_asio_status_get_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field temperature from asio_status message
 *
 * @return [cdegC] Internal temperature of the system, set to -1 if invalid
 */
static inline int16_t mavlink_msg_asio_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field sys_state from asio_status message
 *
 * @return  Current system state
 */
static inline uint8_t mavlink_msg_asio_status_get_sys_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field error from asio_status message
 *
 * @return
 */
static inline uint32_t mavlink_msg_asio_status_get_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field reserved from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_reserved(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field map_warning from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_map_warning(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field pitch_offset from asio_status message
 *
 * @return [deg] Pitch offset
 */
static inline float mavlink_msg_asio_status_get_pitch_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll_offset from asio_status message
 *
 * @return [deg] Roll offset
 */
static inline float mavlink_msg_asio_status_get_roll_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field calibration_state from asio_status message
 *
 * @return
 */
static inline uint8_t mavlink_msg_asio_status_get_calibration_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field reserved_2 from asio_status message
 *
 * @return
 */
static inline float mavlink_msg_asio_status_get_reserved_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field version from asio_status message
 *
 * @return  asio software version
 */
static inline uint32_t mavlink_msg_asio_status_get_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a asio_status message into a struct
 *
 * @param msg The message to decode
 * @param asio_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_asio_status_decode(const mavlink_message_t* msg, mavlink_asio_status_t* asio_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    asio_status->error = mavlink_msg_asio_status_get_error(msg);
    asio_status->pitch_offset = mavlink_msg_asio_status_get_pitch_offset(msg);
    asio_status->roll_offset = mavlink_msg_asio_status_get_roll_offset(msg);
    asio_status->reserved_2 = mavlink_msg_asio_status_get_reserved_2(msg);
    asio_status->version = mavlink_msg_asio_status_get_version(msg);
    asio_status->temperature = mavlink_msg_asio_status_get_temperature(msg);
    asio_status->nav_state = mavlink_msg_asio_status_get_nav_state(msg);
    asio_status->nav_output = mavlink_msg_asio_status_get_nav_output(msg);
    asio_status->gps_jamming_state = mavlink_msg_asio_status_get_gps_jamming_state(msg);
    asio_status->active_sensor = mavlink_msg_asio_status_get_active_sensor(msg);
    asio_status->active_rec = mavlink_msg_asio_status_get_active_rec(msg);
    asio_status->active_db_rec = mavlink_msg_asio_status_get_active_db_rec(msg);
    asio_status->active_nav = mavlink_msg_asio_status_get_active_nav(msg);
    asio_status->active_slam = mavlink_msg_asio_status_get_active_slam(msg);
    asio_status->battery = mavlink_msg_asio_status_get_battery(msg);
    asio_status->sys_state = mavlink_msg_asio_status_get_sys_state(msg);
    asio_status->reserved = mavlink_msg_asio_status_get_reserved(msg);
    asio_status->map_warning = mavlink_msg_asio_status_get_map_warning(msg);
    asio_status->calibration_state = mavlink_msg_asio_status_get_calibration_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASIO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ASIO_STATUS_LEN;
        memset(asio_status, 0, MAVLINK_MSG_ID_ASIO_STATUS_LEN);
    memcpy(asio_status, _MAV_PAYLOAD(msg), len);
#endif
}
