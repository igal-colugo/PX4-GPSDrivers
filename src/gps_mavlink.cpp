/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <ctime>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps_mavlink.h"
#include "rtcm.h"

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define ASH_UNUSED(x) (void) x;
#define HEXDIGIT_CHAR(d) ((char) ((d) + (((d) < 0xA) ? '0' : 'A' - 0xA)))

// #define ASH_DEBUG(...)		{GPS_WARN(__VA_ARGS__);}
#define ASH_DEBUG(...)                                                                                                                                                                       \
    { /*GPS_WARN(__VA_ARGS__);*/                                                                                                                                                             \
    }

px4::atomic_bool GPSDriverMavlink::_is_initialized{false};
// px4::atomic_bool GPSDriverMavlink::_is_can_update_gps_raw_int{false};

GPSDriverMavlink::GPSDriverMavlink(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position, satellite_info_s *satellite_info, float heading_offset)
    : GPSBaseStationSupport(callback, callback_user), _heading_offset(heading_offset), _gps_position(gps_position), _satellite_info(satellite_info)
{
    _is_initialized.store(false);
    get_parameters();
    // hrt_call_every(&_engagecall, (500 * 1000), (1000 * 1000), &GPSDriverMavlink::engage, this);
    // int _task = px4_task_spawn_cmd("update_gps_device", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2048, (px4_main_t) (&GPSDriverMavlink::update_device), nullptr);
    // if (_task < 0)
    // {
    //     return -1;
    // }
}

GPSDriverMavlink::~GPSDriverMavlink()
{
}

void GPSDriverMavlink::get_parameters()
{
    param_t parameter_handle = param_find("GPS_2_LAT");
    param_get(parameter_handle, &initialized_lattitude);

    parameter_handle = param_find("GPS_2_LON");
    param_get(parameter_handle, &initialized_longitude);

    parameter_handle = param_find("GPS_2_INIT_TM");
    param_get(parameter_handle, &initialized_time);
    if (initialized_time > 0)
    {
        start_timer_init_location = hrt_absolute_time();
    }
}

void GPSDriverMavlink::send_mavlink_packet(uint32_t msgid, const char *packet, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    uint16_t checksum;
    uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
    uint8_t ck[2];
    mavlink_status_t *status = new mavlink_status_t();
    uint8_t header_len = MAVLINK_CORE_HEADER_LEN;
    uint8_t signature_len = 0;
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
    bool mavlink1 = 0; // use mavlink 2
    bool signing = 0;  // don't use signing

    uint8_t sending_buffer[150] = {};

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
        buf[3] = 1; // mavlink_system.sysid;
        buf[4] = 0; // mavlink_system.compid;
        buf[5] = msgid & 0xFF;
    }
    else
    {
        uint8_t incompat_flags = 0;
        if (signing)
        {
            incompat_flags |= MAVLINK_IFLAG_SIGNED;
        }
        // length = _mav_trim_payload(packet, length);
        buf[0] = MAVLINK_STX;
        buf[1] = length;
        buf[2] = incompat_flags;
        buf[3] = 0; // compat_flags
        buf[4] = status->current_tx_seq;
        buf[5] = 1; // mavlink_system.sysid;
        buf[6] = 1; // mavlink_system.compid;
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

    for (int i = 0; i < sizeof(buf); i++)
    {
        sending_buffer[i] = buf[i];
    }

    int offset = sizeof(buf);
    for (int i = 0; i < length; i++)
    {
        sending_buffer[i + offset] = (uint8_t) * (packet + i);
    }

    sending_buffer[offset + length] = ck[0];
    sending_buffer[offset + length + 1] = ck[1];

    if (signing)
    {
        // possibly add a signature
        signature_len = mavlink_sign_packet(status->signing, signature, buf, header_len + 1, (const uint8_t *) packet, length, ck);
    }

    // MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t) length + (uint16_t) signature_len);
    // _mavlink_send_uart(chan, (const char *) buf, header_len + 1);
    // _mavlink_send_uart(chan, packet, length);
    // _mavlink_send_uart(chan, (const char *) ck, 2);
    // if (signature_len != 0)
    // {
    //     _mavlink_send_uart(chan, (const char *) signature, signature_len);
    // }
    // MAVLINK_END_UART_SEND(chan, header_len + 3 + (uint16_t) length + (uint16_t) signature_len);

    //@todo Vlad implement creating and sending uart buffer
    // uint8_t test_sending_buffer[56] = {0xFD, 0x2C, 0x00, 0x00, 0xB0, 0x01, 0x01, 0x18, 0x00, 0x00,   0x11, 0x3D, 0x54, 0x40, 0x00, 0x00, 0x00, 0x00,   0xC0,
    //                                    0x5A, 0x03, 0x13,   0xCF, 0xF7, 0xB4, 0x14,  alt 0x8E, 0xB6, 0x00, 0x00, eph  0x4F, 0x00,  epv 0x7F, 0x00,  vel 0x08, 0x00, cog 0x00, 0x00,
    //                                    fix type 0x03, sat 0x0A, alt el 0x99, 0xF9, 0x00, 0x00, hacc 0x9A, 0x0D, 0x00, 0x00, vacc 0x62, 0x13, 0x00, 0x00,  0xC4, 0x02,     0xEB, 0xF3};
    write((void *) sending_buffer, offset + length + 2); // 2-2 bytes of checksum
    // write((void *) test_sending_buffer, 56); // 2-2 bytes of checksum
}

void GPSDriverMavlink::send_mavlink_message(mavlink_message_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    write((void *) msg, sizeof(mavlink_message_t));
}

void GPSDriverMavlink::send_mavlink_attitude_message(mavlink_attitude_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    send_mavlink_packet(MAVLINK_MSG_ID_ATTITUDE, (const char *) msg, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

void GPSDriverMavlink::send_mavlink_gps_raw_int_message(mavlink_gps_raw_int_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    // msg->satellites_visible = 11;
    // msg->lat = 31900347;
    // msg->lon = 34740437;
    // msg->eph = 1100;
    // msg->yaw = 25955;
    send_mavlink_packet(MAVLINK_MSG_ID_GPS_RAW_INT, (const char *) msg, MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN, MAVLINK_MSG_ID_GPS_RAW_INT_LEN, MAVLINK_MSG_ID_GPS_RAW_INT_CRC);
}

void GPSDriverMavlink::send_mavlink_global_position_int_message(mavlink_global_position_int_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    send_mavlink_packet(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *) msg, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN,
                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
}

void GPSDriverMavlink::send_mavlink_extended_sys_state_message(mavlink_extended_sys_state_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    send_mavlink_packet(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, (const char *) msg, MAVLINK_MSG_ID_EXTENDED_SYS_STATE_MIN_LEN, MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN,
                        MAVLINK_MSG_ID_EXTENDED_SYS_STATE_CRC);
}

void GPSDriverMavlink::send_mavlink_command_long_message(mavlink_command_long_t *msg)
{
    //@todo Vlad implement it for communicate with OBOX
    send_mavlink_packet(MAVLINK_MSG_ID_COMMAND_LONG, (const char *) msg, MAVLINK_MSG_ID_COMMAND_LONG_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_LEN, MAVLINK_MSG_ID_COMMAND_LONG_CRC);
}

//@note handle message
bool GPSDriverMavlink::handle_message(mavlink_message_t *msg)
{
    bool return_value = false;

    switch (msg->msgid)
    {

    case MAVLINK_MSG_ID_HIL_GPS:
        handle_message_hil_gps(msg);
        return_value = true;
        break;

    default:
        break;
    }

    return return_value;
}

void GPSDriverMavlink::handle_message_hil_gps(mavlink_message_t *msg)
{
    mavlink_hil_gps_t hil_gps;
    mavlink_msg_hil_gps_decode(msg, &hil_gps);

    sensor_gps_s gps{};

    device::Device::DeviceId device_id{};
    device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
    device_id.devid_s.address = msg->sysid;
    device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
    _gps_position->device_id = device_id.devid;

    _gps_position->lat = hil_gps.lat;
    _gps_position->lon = hil_gps.lon;
    _gps_position->alt = hil_gps.alt;
    _gps_position->alt_ellipsoid = hil_gps.alt;

    _gps_position->s_variance_m_s = 0.25f;
    _gps_position->c_variance_rad = 0.5f;
    _gps_position->fix_type = hil_gps.fix_type;

    _gps_position->eph = (float) hil_gps.eph * 1e-2f; // cm -> m
    _gps_position->epv = (float) hil_gps.epv * 1e-2f; // cm -> m

    _gps_position->hdop = _gps_position->eph;
    _gps_position->vdop = _gps_position->epv;

    _gps_position->noise_per_ms = 0;
    _gps_position->automatic_gain_control = 0;
    _gps_position->jamming_indicator = 0;
    _gps_position->jamming_state = 0;

    _gps_position->vel_m_s = (float) (hil_gps.vel) / 100.0f;  // cm/s -> m/s
    _gps_position->vel_n_m_s = (float) (hil_gps.vn) / 100.0f; // cm/s -> m/s
    _gps_position->vel_e_m_s = (float) (hil_gps.ve) / 100.0f; // cm/s -> m/s
    _gps_position->vel_d_m_s = (float) (hil_gps.vd) / 100.0f; // cm/s -> m/s
    _gps_position->cog_rad = 0;                               //((hil_gps.cog == 65535) ? (float) NAN : matrix::wrap_2pi(math::radians(hil_gps.cog * 1e-2f))); // cdeg -> rad
    _gps_position->vel_ned_valid = true;

    _gps_position->timestamp_time_relative = 0;
    _gps_position->time_utc_usec = hil_gps.time_usec;

    _gps_position->satellites_used = hil_gps.satellites_visible;

    _gps_position->heading = NAN;
    _gps_position->heading_offset = NAN;

    _gps_position->timestamp = hrt_absolute_time();

    is_hil_data_recieved = true;
}

void GPSDriverMavlink::receiveWait(unsigned timeout_min)
{
    gps_abstime time_started = gps_absolute_time();

    while (gps_absolute_time() < time_started + timeout_min * 1000)
    {
        receive(timeout_min);
    }
}

int GPSDriverMavlink::receive(unsigned timeout)
{
    uint8_t buf[100];
    mavlink_message_t msg{};
    mavlink_status_t _status{};

    int handled = 0;

    update_device_frequently();

    /* timeout additional to poll */
    uint64_t time_started = gps_absolute_time();

    while (true)
    {
        /* then poll or read for new data */
        int ret = read(buf, sizeof(buf), timeout * 2);

        if (ret < 0)
        {
            /* something went wrong when polling */
            return -1;
        }
        else if (ret == 0)
        {
            /* Timeout while polling or just nothing read if reading, let's
             * stay here, and use timeout below. */

            // return -1;
        }
        else if (ret > 0)
        {
            for (ssize_t i = 0; i < ret; i++)
            {
                if (mavlink_parse_char(0, buf[i], &msg, &_status))
                {
                    /* handle generic messages and commands */
                    handled = (handle_message(&msg)) ? 1 : 0;
                }
            }

            if (handled > 0)
            {
                return handled;
            }
        }

        /* in case we get crap from GPS or time out */
        if (time_started + timeout * 1000 * 2 < gps_absolute_time())
        {
            return -1;
        }
    }
}

int GPSDriverMavlink::writeAckedCommand(const void *buf, int buf_length, unsigned timeout)
{
    if (write(buf, buf_length) != buf_length)
    {
        return -1;
    }

    return waitForReply(NMEACommand::Acked, timeout);
}

int GPSDriverMavlink::waitForReply(NMEACommand command, const unsigned timeout)
{
    gps_abstime time_started = gps_absolute_time();

    ASH_DEBUG("waiting for reply for command %i", (int) command);

    _command_state = NMEACommandState::waiting;
    _waiting_for_command = command;

    while (_command_state == NMEACommandState::waiting && gps_absolute_time() < time_started + timeout * 1000)
    {
        receive(timeout);
    }

    return _command_state == NMEACommandState::received ? 0 : -1;
}

int GPSDriverMavlink::configure(unsigned &baudrate, const GPSConfig &config)
{
    static bool _is_configured = false;

    _output_mode = config.output_mode;
    _configure_done = false;

    setBaudrate(baudrate);

    _configure_done = true;

    _is_initialized.store(true);

    return 0;
}

int GPSDriverMavlink::update_device(int argc, char *argv[])
{
    static hrt_abstime last_rate_gps_raw_int_update = hrt_absolute_time();

    //@todo Vlad implement update Obox with different frequences

    while (true)
    {
        if (_is_initialized.load())
        {
            //     if (hrt_absolute_time() - last_rate_gps_raw_int_update > RATE_GPS_RAW_INT_PERIOD)
            //     {
            //         _is_can_update_gps_raw_int.store(true);

            //         last_rate_gps_raw_int_update = hrt_absolute_time();
            //     }
            mavlink_gps_raw_int_t mavlink_gps_raw_int_msg = {};
            send_mavlink_gps_raw_int_message(&mavlink_gps_raw_int_msg);
        }

        px4_usleep(1000000);
    }

    PX4_INFO("exiting");
}

void GPSDriverMavlink::update_device_frequently()
{
    static hrt_abstime last_rate_mavlink_attitude_msg_update = hrt_absolute_time();
    static hrt_abstime last_rate_mavlink_gps_raw_int_msg_update = hrt_absolute_time();
    static hrt_abstime last_rate_mavlink_global_position_int_msg_update = hrt_absolute_time();

    sensor_gps_s main_gps_data;
    vehicle_attitude_s actual_attitude;

    timer_init_location = hrt_absolute_time();
    if (initialized_time > 0 && (timer_init_location - start_timer_init_location) > (initialized_time * 1000 * 1000) && !is_hil_data_recieved)
    {
        mavlink_mavlink_command_long_msg.command = 40604;
        mavlink_mavlink_command_long_msg.param5 = initialized_lattitude;
        mavlink_mavlink_command_long_msg.param6 = initialized_longitude;

        send_mavlink_command_long_message(&mavlink_mavlink_command_long_msg);

        start_timer_init_location = hrt_absolute_time();
    }

    if (_sensor_gps_sub.updated())
    {
        if (_sensor_gps_sub.copy(&main_gps_data))
        {
            // MESSAGE GLOBAL_POSITION_INT PACKING
            // =============================
            // uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
            // int32_t lat;           /*< [degE7] Latitude, expressed*/
            // int32_t lon;           /*< [degE7] Longitude, expressed*/
            // int32_t alt;           /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
            // int32_t relative_alt;  /*< [mm] Altitude above ground*/
            // int16_t vx;            /*< [cm/s] Ground X Speed (Latitude, positive north)*/
            // int16_t vy;            /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
            // int16_t vz;            /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
            // uint16_t hdg;          /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

            // MESSAGE GPS_RAW_INT_INT PACKING
            // =============================
            // int32_t lat;                /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
            // int32_t lon;                /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
            // int32_t alt;                /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
            // uint16_t eph;               /*<  GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
            // uint16_t epv;               /*<  GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
            // uint16_t vel;               /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
            // uint16_t cog;               /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
            // uint8_t fix_type;           /*<  GPS fix type.*/
            // uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to UINT8_MAX*/
            // int32_t alt_ellipsoid;      /*< [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.*/
            // uint32_t h_acc;             /*< [mm] Position uncertainty.*/
            // uint32_t v_acc;             /*< [mm] Altitude uncertainty.*/
            // uint32_t vel_acc;           /*< [mm] Speed uncertainty.*/
            // uint32_t hdg_acc;           /*< [degE5] Heading / track uncertainty*/
            // uint16_t yaw;

            // sensors_gps uorb message
            // =============================
            // uint64_t timestamp;
            // uint64_t time_utc_usec;
            // uint32_t device_id;
            // int32_t lat;
            // int32_t lon;
            // int32_t alt;
            // int32_t alt_ellipsoid;
            // float s_variance_m_s;
            // float c_variance_rad;
            // float eph;
            // float epv;
            // float hdop;
            // float vdop;
            // int32_t noise_per_ms;
            // int32_t jamming_indicator;
            // float vel_m_s;
            // float vel_n_m_s;
            // float vel_e_m_s;
            // float vel_d_m_s;
            // float cog_rad;
            // int32_t timestamp_time_relative;
            // float heading;
            // float heading_offset;
            // float heading_accuracy;
            // uint16_t automatic_gain_control;
            // uint8_t fix_type;
            // uint8_t jamming_state;
            // bool vel_ned_valid;
            // uint8_t satellites_used;
            // uint8_t _padding0[2]; // required for logger

            mavlink_gps_raw_int_msg.lat = main_gps_data.lat;
            mavlink_gps_raw_int_msg.lon = main_gps_data.lon;
            mavlink_gps_raw_int_msg.alt = main_gps_data.alt;
            mavlink_gps_raw_int_msg.eph = 79;                    //(uint16_t) (main_gps_data.eph * 100.0); //*100 check it
            mavlink_gps_raw_int_msg.epv = 127;                   // (uint16_t) (main_gps_data.epv * 100.0); //*100 check it
            mavlink_gps_raw_int_msg.vel = main_gps_data.vel_m_s; // check it
            mavlink_gps_raw_int_msg.cog = main_gps_data.cog_rad;
            mavlink_gps_raw_int_msg.fix_type = main_gps_data.fix_type;
            mavlink_gps_raw_int_msg.satellites_visible = 11; // main_gps_data.satellites_used;
            mavlink_gps_raw_int_msg.alt_ellipsoid = main_gps_data.alt_ellipsoid;
            mavlink_gps_raw_int_msg.h_acc = 0;
            mavlink_gps_raw_int_msg.v_acc = 0;
            mavlink_gps_raw_int_msg.vel_acc = 0;
            mavlink_gps_raw_int_msg.hdg_acc = 0;
            mavlink_gps_raw_int_msg.yaw = 708; // 25955; //(uint16_t) main_gps_data.heading;

            mavlink_global_position_int_msg.lat = main_gps_data.lat;
            mavlink_global_position_int_msg.lon = main_gps_data.lon;
            mavlink_global_position_int_msg.alt = main_gps_data.alt;
        }
    }
    if (_vehicle_attitude_sub.updated())
    {
        if (_vehicle_attitude_sub.copy(&actual_attitude))
        {
            // MESSAGE ATTITUDE PACKING
            // =============================
            // uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
            // float roll;            /*< [rad] Roll angle (-pi..+pi)*/
            // float pitch;           /*< [rad] Pitch angle (-pi..+pi)*/
            // float yaw;             /*< [rad] Yaw angle (-pi..+pi)*/
            // float rollspeed;       /*< [rad/s] Roll angular speed*/
            // float pitchspeed;      /*< [rad/s] Pitch angular speed*/
            // float yawspeed;        /*< [rad/s] Yaw angular speed*/

            // vehicle_attitude uorb message
            //  =============================
            // uint64 timestamp #time since system start(microseconds)
            // uint64 timestamp_sample #the timestamp of the raw data(microseconds)
            // float32[4] q #Quaternion rotation from the FRD body frame to the NED earth frame float32[4] delta_q_reset #Amount by which quaternion has changed during last reset uint8
            // quat_reset_counter #Quaternion reset counter

            q.w = actual_attitude.q[0];
            q.x = actual_attitude.q[1];
            q.y = actual_attitude.q[2];
            q.z = actual_attitude.q[3];

            angles = convert_to_euler_angles(q);

            mavlink_attitude_msg.roll = angles.roll;
            mavlink_attitude_msg.pitch = angles.pitch;
            mavlink_attitude_msg.yaw = angles.yaw;
        }
    }

    hrt_abstime actual_rate_update = hrt_absolute_time();
    hrt_abstime attitude_update_time = actual_rate_update - last_rate_mavlink_attitude_msg_update;
    if (attitude_update_time > RATE_GPS_RAW_INT_PERIOD)
    {
        send_mavlink_attitude_message(&mavlink_attitude_msg);
        last_rate_mavlink_attitude_msg_update = hrt_absolute_time();
    }
    actual_rate_update = hrt_absolute_time();
    hrt_abstime gps_raw_int_update_time = actual_rate_update - last_rate_mavlink_gps_raw_int_msg_update;
    if (gps_raw_int_update_time > RATE_GPS_RAW_INT_PERIOD)
    {
        send_mavlink_gps_raw_int_message(&mavlink_gps_raw_int_msg);
        last_rate_mavlink_gps_raw_int_msg_update = hrt_absolute_time();
    }
    actual_rate_update = hrt_absolute_time();
    hrt_abstime global_position_int_update_time = actual_rate_update - last_rate_mavlink_global_position_int_msg_update;
    if (global_position_int_update_time > RATE_GPS_RAW_INT_PERIOD)
    {
        send_mavlink_global_position_int_message(&mavlink_global_position_int_msg);
        last_rate_mavlink_global_position_int_msg_update = hrt_absolute_time();
    }
}

void GPSDriverMavlink::engage(void *arg)
{
    // GPSDriverMavlink *mavlink_driver = static_cast<GPSDriverMavlink *>(arg);
    // mavlink_gps_raw_int_t mavlink_gps_raw_int_msg = {};
    // mavlink_driver->send_mavlink_gps_raw_int_message(&mavlink_gps_raw_int_msg);
}

void GPSDriverMavlink::disengage(void *arg)
{
    // GPSDriverMavlink *trig = static_cast<GPSDriverMavlink *>(arg);
    // trig->_camera_interface->trigger(false);
}

#pragma region Helper function

GPSDriverMavlink::Quaternion GPSDriverMavlink::convert_to_quaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
GPSDriverMavlink::EulerAngles GPSDriverMavlink::convert_to_euler_angles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

#pragma endregion
