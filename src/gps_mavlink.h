/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
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

/** @file ASHTECH protocol definitions */

#pragma once

#include "../../definitions.h"
#include "../mavlink/mavlink_helpers.h"
#include "../mavlink/mavlink_msg_asio_status.h"
#include "../mavlink/mavlink_msg_command_ack.h"
#include "../mavlink/mavlink_msg_hil_gps.h"
#include "../mavlink/mavlink_types.h"
#include "base_station.h"
#include "gps_helper.h"
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <px4_platform_common/atomic.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/arial_obox_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <math.h>

#define MAVLINK_MSG_ID_HEARTBIT 0
#define MAVLINK_MSG_ID_HIL_GPS 113
#define MAV_CMD_ASIO_SET_SENSOR 40601
#define MAV_CMD_ASIO_SET_REC 40602
#define MAV_CMD_ASIO_SET_NAV_MODE 40603
#define MAV_CMD_ASIO_SET_INIT_LOC 40604
#define MAV_CMD_ASIO_RESET 40605
#define MAV_CMD_ASIO_CAL 40606
#define RATE_EXTENDED_SYS_STATE_PERIOD 1000000 // 1 Hz
#define RATE_GPS_RAW_INT_PERIOD 200000         // 5 Hz
#define RATE_ATTITUDE_PERIOD 90000             // 11 Hz

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

class GPSDriverMavlink : public GPSBaseStationSupport
{
  public:
    static hrt_abstime start_timer_init_location;

    struct Quaternion
    {
        double w;
        double x;
        double y;
        double z;
    };

    struct EulerAngles
    {
        double roll;
        double pitch;
        double yaw;
    };
    /**
     * @param heading_offset heading offset in radians [-pi, pi]. It is substracted from the measurement.
     */
    GPSDriverMavlink(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position, satellite_info_s *satellite_info, float heading_offset = 0.f);
    virtual ~GPSDriverMavlink();
    int configure(unsigned &baudrate, const GPSConfig &config) override;
    int receive(unsigned timeout) override;
    int update_device(int argc, char *argv[]);
    bool update_device_frequently();

  private:
    enum class NMEACommand
    {
        Acked,  // Command that returns a (N)Ack
        PRT,    // port config
        RID,    // board identification
        RECEIPT // board identification
    };

    enum class NMEACommandState
    {
        idle,
        waiting,
        nack,
        received
    };

    void get_parameters();
    bool handle_message(mavlink_message_t *msg);
    void handle_message_acknowledge(mavlink_message_t *msg);
    void handle_message_heartbit(mavlink_message_t *msg);
    void handle_message_hil_gps(mavlink_message_t *msg);
    void handle_message_asio_status(mavlink_message_t *msg);
    void send_mavlink_message(mavlink_message_t *msg);
    void send_mavlink_attitude_message(mavlink_attitude_t *msg);
    void send_mavlink_gps_raw_int_message(mavlink_gps_raw_int_t *msg);
    void send_mavlink_global_position_int_message(mavlink_global_position_int_t *msg);
    void send_mavlink_extended_sys_state_message(mavlink_extended_sys_state_t *msg);
    void send_mavlink_command_long_message(mavlink_command_long_t *msg);
    void send_mavlink_packet(uint32_t msgid, const char *packet, uint8_t min_length, uint8_t length, uint8_t crc_extra);

    void set_asio_auto_init_location();
    void set_asio_init_location(float lattitude, float longitude);
    void set_asio_init_location_2();
    void set_asio_sensor_type(uint8_t day_thermal);
    void set_asio_recording_type(uint8_t type);
    void set_asio_navigation_mode(uint8_t mode);
    void set_asio_calibration();

    /**
     * receive data for at least the specified amount of time
     */
    void receiveWait(unsigned timeout_min);

    /**
     * Write a command and wait for a (N)Ack
     * @return 0 on success, <0 otherwise
     */
    int writeAckedCommand(const void *buf, int buf_length, unsigned timeout);

    int waitForReply(NMEACommand command, const unsigned timeout);

    bool _configure_done{false};

    uint64_t _last_timestamp_time{0};

    float _heading_offset;

    sensor_gps_s *_gps_position{nullptr};
    satellite_info_s *_satellite_info{nullptr};
    arial_obox_status_s _report_arial_obox_status{}; ///< uORB topic for arial obox status

    NMEACommand _waiting_for_command;

    NMEACommandState _command_state{NMEACommandState::idle};

    OutputMode _output_mode{OutputMode::GPS};

    static px4::atomic_bool _is_initialized;
    // static px4::atomic_bool _is_can_update_gps_raw_int;
    uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _command_sub{ORB_ID(vehicle_command)};

    uORB::Publication<arial_obox_status_s> _arial_obox_status_pub{ORB_ID(arial_obox_status)};
    uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

    EulerAngles angles = EulerAngles();
    Quaternion q = Quaternion();

    mavlink_attitude_t mavlink_attitude_msg = {};
    mavlink_gps_raw_int_t mavlink_gps_raw_int_msg = {};
    mavlink_global_position_int_t mavlink_global_position_int_msg = {};
    mavlink_command_long_t mavlink_mavlink_command_long_msg = {};

    int32_t initialized_lattitude = 0;
    int32_t initialized_longitude = 0;
    int32_t initialized_time = -1;
    hrt_abstime timer_init_location = 0;
    bool is_hil_data_recieved = false;

    struct hrt_call _engagecall
    {
    };
    struct hrt_call _disengagecall
    {
    };

    /**
     * Fires trigger
     */
    static void engage(void *arg);

    /**
     * Resets trigger
     */
    static void disengage(void *arg);

    Quaternion convert_to_quaternion(double roll, double pitch, double yaw);
    EulerAngles convert_to_euler_angles(Quaternion q);
};
