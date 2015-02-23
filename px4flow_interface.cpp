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
 * @file px4flow_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "px4flow_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   PX4Flow Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
PX4Flow_Interface::
PX4Flow_Interface(Serial_Port *serial_port_, int msgID_, int msgFieldName_, std::vector<float> *data_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	px4flow_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = px4flow_id;

	serial_port = serial_port_; // serial port management object

	msgUserID = msgID_;
	msgUserFieldName = msgFieldName_;
	data = data_;
}

PX4Flow_Interface::
~PX4Flow_Interface()
{}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
read_messages()
{
	bool success;               // receive success flag
	Time_Stamps this_timestamps;

	while (not time_to_exit)
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			if(message.msgid != msgUserID) return;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT: // #0
				{
					// printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					
					enum Fields{
						type, autopilot, base_mode, custom_mode, system_status, mavlink_version
					};

					switch(msgUserFieldName){
						case type:
							data->push_back((float) current_messages.heartbeat.type);
							break;
					case autopilot:
							data->push_back((float) current_messages.heartbeat.autopilot);
							break;	
					case base_mode:
							data->push_back((float) current_messages.heartbeat.base_mode);
							break;
					case custom_mode:
							data->push_back((float) current_messages.heartbeat.custom_mode);
							break;
					case system_status:
							data->push_back((float) current_messages.heartbeat.system_status);
							break;
					case mavlink_version:
							data->push_back((float) current_messages.heartbeat.mavlink_version);
							break;
					default:
							// ########## SAIR DO PGM? ##########
							break;						
					}

					break;
				}

				case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: // #130
				{
					// printf("MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE\n");
					mavlink_msg_data_transmission_handshake_decode(&message, &(current_messages.data_transmission_handshake));
					current_messages.time_stamps.data_transmission_handshake = get_time_usec();
					this_timestamps.data_transmission_handshake = current_messages.time_stamps.data_transmission_handshake;

					enum Fields{
						type, size, width, height, packets, payload, jpg_quality
					};

					switch(msgUserFieldName){
						case type:
							data->push_back((float) current_messages.data_transmission_handshake.type);
							break;
						case size:
							data->push_back((float) current_messages.data_transmission_handshake.size);
							break;
						case width:
							data->push_back((float) current_messages.data_transmission_handshake.width);
							break;
						case height:
							data->push_back((float) current_messages.data_transmission_handshake.height);
							break;
						case packets:
							data->push_back((float) current_messages.data_transmission_handshake.packets);
							break;
						case payload:
							data->push_back((float) current_messages.data_transmission_handshake.payload);
							break;
						case jpg_quality:
							data->push_back((float) current_messages.data_transmission_handshake.jpg_quality);
							break;
						default:
							break;
					}

					break;
				}

				case MAVLINK_MSG_ID_ENCAPSULATED_DATA: // #131
				{
					// printf("MAVLINK_MSG_ID_ENCAPSULATED_DATA\n");
					mavlink_msg_encapsulated_data_decode(&message, &(current_messages.encapsulated_data));
					current_messages.time_stamps.encapsulated_data = get_time_usec();
					this_timestamps.encapsulated_data = current_messages.time_stamps.encapsulated_data;

					enum Fields{
						seqnr, data
					};

					switch(msgUserFieldName){
						case seqnr:
								// error: base operand of ‘->’ is not a pointer
								// data->push_back((float) current_messages.encapsulated_data.seqnr); 
							break;
						
						case data:
						{
							// implementar
							printf("\n----------------------------------------------\n%d: ", current_messages.encapsulated_data.seqnr);

							uint8_t *img = current_messages.encapsulated_data.data;

							for (int i = 0; i < 253; ++i){
								printf("%d ", *img);
								img++;
							}
						}break;

						default:
							break;

					}

					break;
				}

				case MAVLINK_MSG_ID_OPTICAL_FLOW: // #100
				{
					// printf("MAVLINK_MSG_ID_OPTICAL_FLOW\n");
					mavlink_msg_optical_flow_decode(&message, &(current_messages.optical_flow));
					current_messages.time_stamps.optical_flow = get_time_usec();
					this_timestamps.optical_flow = current_messages.time_stamps.optical_flow;
					// data->push_back(current_messages.optical_flow.flow_x);

					enum Fields{
						time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance
					};

					switch(msgUserFieldName){
						case time_usec:
							data->push_back((float) current_messages.optical_flow.time_usec);
							break;
						case sensor_id:
							data->push_back((float) current_messages.optical_flow.sensor_id);
							break;
						case flow_x:
							data->push_back((float) current_messages.optical_flow.flow_x);
							break;
						case flow_y:
							data->push_back((float) current_messages.optical_flow.flow_y);
							break;
						case flow_comp_m_x:
							data->push_back((float) current_messages.optical_flow.flow_comp_m_x);
							break;
						case flow_comp_m_y:
							data->push_back((float) current_messages.optical_flow.flow_comp_m_y);
							break;
						case quality:
							data->push_back((float) current_messages.optical_flow.quality);
							break;
						case ground_distance:
							data->push_back((float) current_messages.optical_flow.ground_distance);
							break;
						default:
							break;
					}

					break;
				}

				case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: // #106
				{
					// printf("MAVLINK_MSG_ID_OPTICAL_FLOW_RAD\n");
					mavlink_msg_optical_flow_rad_decode(&message, &(current_messages.optical_flow_rad));
					current_messages.time_stamps.optical_flow_rad = get_time_usec();
					this_timestamps.optical_flow_rad = current_messages.time_stamps.optical_flow_rad;

					enum Fields{
						time_usec, sensor_id, integration_time_us, integrated_x, integrated_y, integrated_xgyro,
						integrated_ygyro, integrated_zgyro, temperature, quality, time_delta_distance_us, distance
					};

					switch(msgUserFieldName){
						case time_usec:
							data->push_back((float) current_messages.optical_flow_rad.time_usec);
							break;
						case sensor_id:
							data->push_back((float) current_messages.optical_flow_rad.sensor_id);
							break;
						case integration_time_us:
							data->push_back((float) current_messages.optical_flow_rad.integration_time_us);
							break;
						case integrated_x:
							data->push_back((float) current_messages.optical_flow_rad.integrated_x);
							break;
						case integrated_y:
							data->push_back((float) current_messages.optical_flow_rad.integrated_y);
							break;
						case integrated_xgyro:
							data->push_back((float) current_messages.optical_flow_rad.integrated_xgyro);
							break;
						case integrated_ygyro:
							data->push_back((float) current_messages.optical_flow_rad.integrated_ygyro);
							break;
						case integrated_zgyro:
							data->push_back((float) current_messages.optical_flow_rad.integrated_zgyro);
							break;
						case temperature:
							data->push_back((float) current_messages.optical_flow_rad.temperature);
							break;
						case quality:
							data->push_back((float) current_messages.optical_flow_rad.quality);
							break;
						case time_delta_distance_us:
							data->push_back((float) current_messages.optical_flow_rad.time_delta_distance_us);
							break;
						case distance:
							data->push_back((float) current_messages.optical_flow_rad.distance);
							break;
						default:
							break;
					}

					break;
				}

				case MAVLINK_MSG_ID_DEBUG_VECT: // #250
				{
					// printf("MAVLINK_MSG_ID_DEBUG_VECT\n");
					mavlink_msg_debug_vect_decode(&message, &(current_messages.debug_vect));
					current_messages.time_stamps.debug_vect = get_time_usec();
					this_timestamps.debug_vect = current_messages.time_stamps.debug_vect;

					enum Fields{
						name, time_usec, x, y, z
					};

					switch(msgUserFieldName){
						case name:
							// data->push_back((float) current_messages.debug_vect.name); // ########## CHAR ##########
							break;
						case time_usec:
							data->push_back((float) current_messages.debug_vect.time_usec);
							break;
						case x:
							data->push_back((float) current_messages.debug_vect.x);
							break;
						case y:
							data->push_back((float) current_messages.debug_vect.y);
							break;
						case z:
							data->push_back((float) current_messages.debug_vect.z);
							break;
						default:
							break;
					}

					break;
				}

				case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: // #251
				{
					// printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOAT\n");
					mavlink_msg_named_value_float_decode(&message, &(current_messages.named_value_float));
					current_messages.time_stamps.named_value_float = get_time_usec();
					this_timestamps.named_value_float = current_messages.time_stamps.named_value_float;

					enum Fields{
						time_boot_ms, name, value
					};

					switch(msgUserFieldName){
						case time_boot_ms:
							data->push_back((float) current_messages.named_value_float.time_boot_ms);
							break;
						case name:
							// data->push_back((float) current_messages.named_value_float.name); // ########## CHAR ##########
							break;
						case value:
							data->push_back((float) current_messages.named_value_float.value);
							break;
						default:
							break;
					}

					break;
				}

				case MAVLINK_MSG_ID_NAMED_VALUE_INT: // #252
				{
					// printf("MAVLINK_MSG_ID_NAMED_VALUE_INT\n");
					mavlink_msg_named_value_int_decode(&message, &(current_messages.named_value_int));
					current_messages.time_stamps.named_value_int = get_time_usec();
					this_timestamps.named_value_int = current_messages.time_stamps.named_value_int;

					enum Fields{
						time_boot_ms, name, value
					};

					switch(msgUserFieldName){
						case time_boot_ms:
							data->push_back((float) current_messages.named_value_int.time_boot_ms);
							break;
						case name:
							// data->push_back((float) current_messages.named_value_int.name); // ########## CHAR ##########
							break;
						case value:
							data->push_back((float) current_messages.named_value_int.value);
							break;
						default:
							break;
					}

					break;
				}

				default:
				{
					printf("Warning, did not handle message id %i\n", message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// give the write thread time to use the port
		// if ( writing_status > false )
		// 	usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
PX4Flow_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}


// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
PX4Flow_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com;
	com.target_system    = system_id;
	com.target_component = px4flow_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( not serial_port->status == 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_px4flow_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not px4flow_id )
	{
		px4flow_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", px4flow_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// // Wait for initial position ned
	// while ( not ( current_messages.time_stamps.local_position_ned &&
	// 			  current_messages.time_stamps.attitude            )  )
	// {
	// 	if ( time_to_exit )
	// 		return;
	// 	usleep(500000);
	// }

	// // copy initial position ned
	// Mavlink_Messages local_data = current_messages;
	// initial_position.x        = local_data.local_position_ned.x;
	// initial_position.y        = local_data.local_position_ned.y;
	// initial_position.z        = local_data.local_position_ned.z;
	// initial_position.vx       = local_data.local_position_ned.vx;
	// initial_position.vy       = local_data.local_position_ned.vy;
	// initial_position.vz       = local_data.local_position_ned.vz;
	// initial_position.yaw      = local_data.attitude.yaw;
	// initial_position.yaw_rate = local_data.attitude.yawspeed;

	// printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	// printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	// printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	// result = pthread_create( &write_tid, NULL, &start_px4flow_interface_write_thread, this );
	// if ( result ) throw result;

	// wait for it to be started
	// while ( not writing_status )
	// 	usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	// printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
read_thread()
{
	reading_status = true;

	while ( not time_to_exit )
	{
		read_messages();
		// usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
write_thread(void)
{
	// signal startup
	// writing_status = 2;

	// // prepare an initial setpoint, just stay put
	// mavlink_set_position_target_local_ned_t sp;
	// sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
	// 			   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	// sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	// sp.vx       = 0.0;
	// sp.vy       = 0.0;
	// sp.vz       = 0.0;
	// sp.yaw_rate = 0.0;

	// // set position target
	// current_setpoint = sp;

	// // write a message and signal writing
	// write_setpoint();
	// writing_status = true;

	// // Pixhawk needs to see off-board commands at minimum 2Hz,
	// // otherwise it will go into fail safe
	// while ( not time_to_exit )
	// {
	// 	usleep(250000);   // Stream at 4Hz
	// 	write_setpoint();
	// }

	// // signal end
	// writing_status = false;

	return;

}

// End PX4Flow_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_px4flow_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	PX4Flow_Interface *px4flow_interface = (PX4Flow_Interface *)args;

	// run the object's read thread
	px4flow_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_px4flow_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	PX4Flow_Interface *px4flow_interface = (PX4Flow_Interface *)args;

	// run the object's read thread
	px4flow_interface->start_write_thread();

	// done!
	return NULL;
}



