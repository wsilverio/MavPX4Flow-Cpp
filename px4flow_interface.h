/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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

/**
 * @file px4flow_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef PX4FLOW_INTERFACE_H_
#define PX4FLOW_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <common/mavlink.h>

 #include <iostream>
 #include <vector>

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();

void* start_px4flow_interface_read_thread(void *args);
void* start_px4flow_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t data_transmission_handshake;
	uint64_t encapsulated_data;
	uint64_t optical_flow;
	uint64_t optical_flow_rad;
	uint64_t debug_vect;
	uint64_t named_value_float;
	uint64_t named_value_int;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		data_transmission_handshake = 0;
		encapsulated_data = 0;
		optical_flow = 0;
		optical_flow_rad = 0;
		debug_vect = 0;
		named_value_float = 0;
		named_value_int = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// Data Transmission Handshake
	mavlink_data_transmission_handshake_t data_transmission_handshake;

	// Encapsulated Data
	mavlink_encapsulated_data_t encapsulated_data;

	// Optical Flow
	mavlink_optical_flow_t optical_flow;

	// Optical Flow Rad
	mavlink_optical_flow_rad_t optical_flow_rad;

	// Debug Vect
	mavlink_debug_vect_t debug_vect;

	// Named Value Float
	mavlink_named_value_float_t named_value_float;

	// Named Value Int
	mavlink_named_value_int_t named_value_int;

	// System Parameters?

	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};

// Struct com os tipos comuns entre as mensagens

struct Common_Types{
    /*
    (0)Heartbeat | (1)Data_transmission_handshake | (2)Encapsulated_data | (3)Optical_flow
    (4)Optical_flow_rad | (5)Debug_vect | (6)Named_value_float | (7)Named_value_int
    */
    
    // Type: uint64_t
    // (0) ---
    // (1) ---
    // (2) ---
    // (3) time_usec
    // (4) time_usec
    // (5) time_usec
    // (6) ---
    // (7) ---
    std::vector<uint64_t> data_uint64_t;

	// Type: int32_t
    // (0) ---
    // (1) ---
    // (2) ---
    // (3) ---
    // (4) ---
    // (5) ---
    // (6) ---
    // (7) value
    std::vector<int32_t> data_int32_t;

	// Type: 
    // (0) custom_mode
    // (1) size
    // (2) ---
    // (3) ---
    // (4) integration_time_us, time_delta_distance_us
    // (5) ---
    // (6) time_boot_ms
    // (7) time_boot_ms
    std::vector<uint32_t> data_uint32_t;

	// Type: int16_t
    // (0) ---
    // (1) ---
    // (2) ---
    // (3) flow_x, flow_y
    // (4) temperature
    // (5) ---
    // (6) ---
    // (7) ---
    std::vector<int16_t> data_int16_t;

	// Type: uint16_t
    // (0) ---
    // (1) width, height, packets
    // (2) seqnr
    // (3) ---
    // (4) ---
    // (5) ---
    // (6) ---
    // (7) ---
    std::vector<uint16_t> data_uint16_t;

	// Type: uint8_t
    // (0) type, autopilot, base_mode, system_status, mavlink_version
    // (1) type, payload, jpg_quality
    // (2) data[253]
    // (3) sensor_id, quality
    // (4) sensor_id, quality
    // (5) ---
    // (6) ---
    // (7) ---
    std::vector<uint8_t> data_uint8_t;
    uint8_t data_array_uint8_t[253];

	// Type: float
    // (0) ---
    // (1) ---
    // (2) ---
    // (3) flow_comp_m_x, flow_comp_m_y, ground_distance
    // (4) integrated_x, integrated_y, integrated_xgyro, integrated_ygyro, integrated_zgyro, distance
    // (5) x, y, z
    // (6) value
    // (7) ---
    std::vector<float> data_float;

	// Type: char
    // (0) ---
    // (1) ---
    // (2) ---
    // (3) ---
    // (4) ---
    // (5) name[10]
    // (6) name[10]
    // (7) name[10]
    char data_char[10];
    // std::vector<char> data_char;
};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
class PX4Flow_Interface
{

public:

	PX4Flow_Interface();
	PX4Flow_Interface(Serial_Port *serial_port_, int msgID_, int msgFieldName_, Common_Types *Data_);
	~PX4Flow_Interface();

	char reading_status;
	char writing_status;
	char control_status;
    uint64_t write_count;

    int system_id;
	int px4flow_id;
	int companion_id;
	int msgUserID;
	int msgUserFieldName;

	std::vector<float> *data;
	Common_Types *Data;

	Mavlink_Messages current_messages;

	void read_messages();
	int  write_message(mavlink_message_t message);

	void enable_offboard_control();
	void disable_offboard_control();

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );


private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );
};



#endif // PX4FLOW_INTERFACE_H_


