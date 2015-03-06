/****************************************************************************
 *
 *        Disponível em https://github.com/wsilverio/MavPX4Flow-Cpp
 *
 *        Baseado em github.com/mavlink/c_uart_interface_example,
 *            dos autores MAVlink Development Team:
 *                Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *                Jaycee Lock,    <jaycee.lock@gmail.com>
 *                Lorenz Meier,   <lm@inf.ethz.ch>
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
#include <opencv2/opencv.hpp> // openCV
#include <pthread.h>

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
    uint64_t param_value;
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
        param_value = 0;
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

    // Param Value
    mavlink_param_value_t param_value;
    
    std::vector<mavlink_param_value_t> vector_param_value;

    // System Parameters?

    // Time Stamps
    Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }

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
    PX4Flow_Interface(Serial_Port *serial_port_, int msgID_, int msgFieldName_, std::vector<float> *data_, cv::Mat *img_);
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

    std::vector<uint8_t> imgVector;
    std::vector<float> *data;

    uint16_t packet;
    cv::Mat *img;
    pthread_mutex_t trava;

    Mavlink_Messages current_messages;

    void read_messages();
    int  write_message(mavlink_message_t message);

    void enable_offboard_control();
    void disable_offboard_control();

    void start();
    void stop();

    void start_read_thread();
    void start_write_thread(void);

    void param_request_list(void);
    void set_video_only(float val);

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


