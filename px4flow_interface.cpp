/****************************************************************************
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
PX4Flow_Interface(Serial_Port *serial_port_, int msgID_, int msgFieldName_, std::vector<float> *data_, cv::Mat *img_)
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
    img = img_;
    packet = 0;
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

            if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA and message.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE){
                // pass
            }else if(message.msgid != msgUserID)
                return;

            // Handle Message ID
            switch (message.msgid)
            {

                case MAVLINK_MSG_ID_HEARTBEAT: // #0
                {
                    // uint32_t custom_mode
                    // uint8_t type
                    // uint8_t autopilot
                    // uint8_t base_mode
                    // uint8_t system_status
                    // uint8_t mavlink_version

                    // printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                    current_messages.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                    
                    enum Fields{
                        type, autopilot, base_mode, custom_mode, system_status, mavlink_version
                    };

                    switch(msgUserFieldName)
                    {
                        case type:
                        {
                            data->push_back((float) current_messages.heartbeat.type);
                            printf("type: %u\n",    current_messages.heartbeat.type);
                        }break;

                        case autopilot:
                        {
                            data->push_back(    (float) current_messages.heartbeat.autopilot);
                            printf("autopilot: %u\n",   current_messages.heartbeat.autopilot);
                        }break; 

                        case base_mode:
                        {
                            data->push_back(    (float) current_messages.heartbeat.base_mode);
                            printf("base_mode: %u\n",   current_messages.heartbeat.base_mode);
                        }break;

                        case custom_mode:
                        {
                            data->push_back(    (float) current_messages.heartbeat.custom_mode);
                            printf("custom_mode: %u\n", current_messages.heartbeat.custom_mode);
                        }break;

                        case system_status:
                        {
                            data->push_back(        (float) current_messages.heartbeat.system_status);
                            printf("system_status: %u\n",   current_messages.heartbeat.system_status);
                        }break;

                        case mavlink_version:
                        {
                            data->push_back(        (float) current_messages.heartbeat.mavlink_version);
                            printf("link_version: %u\n",    current_messages.heartbeat.mavlink_version);
                        }break;

                        default: 
                            break;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: // #130
                {
                    // uint32_t size
                    // uint16_t width
                    // uint16_t height
                    // uint16_t packets
                    // uint8_t type
                    // uint8_t payload
                    // uint8_t jpg_quality

                    // printf("MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE\n");
                    mavlink_msg_data_transmission_handshake_decode(&message, &(current_messages.data_transmission_handshake));
                    current_messages.time_stamps.data_transmission_handshake = get_time_usec();
                    this_timestamps.data_transmission_handshake = current_messages.time_stamps.data_transmission_handshake;

                    break;

                    if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA){
                        std::cout 
                                << "type: " << (unsigned int) current_messages.data_transmission_handshake.type
                                << "\nsize: " << current_messages.data_transmission_handshake.size
                                << "\nwidth: " << current_messages.data_transmission_handshake.width
                                << "\nheight: " << current_messages.data_transmission_handshake.height
                                << "\npackets: " << current_messages.data_transmission_handshake.packets
                                << "\npayload: " << (unsigned int) current_messages.data_transmission_handshake.payload
                                << "\njpg_quality: " << (unsigned int) current_messages.data_transmission_handshake.jpg_quality << "\n\n";

                        // imgPointer = (uint8_t*) malloc(current_messages.data_transmission_handshake.size*sizeof(uint8_t) + 1);
                        // if(!imgPointer){
                        //     printf("Erro ao alocar memória.\n");
                        //     exit(EXIT_FAILURE);
                        // }

                    }else{
                        enum Fields{
                            type, size, width, height, packets, payload, jpg_quality
                        };
                                                            
                        switch(msgUserFieldName)
                        {
                            case type:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.type);
                                printf("type: %u\n",    current_messages.data_transmission_handshake.type);
                            }break;

                            case size:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.size);
                                printf("size: %u\n",    current_messages.data_transmission_handshake.size);
                            }break;

                            case width:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.width);
                                printf("width: %u\n",   current_messages.data_transmission_handshake.width);
                            }break;

                            case height:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.height);
                                printf("height: %u\n",  current_messages.data_transmission_handshake.height);
                            }break;

                            case packets:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.packets);
                                printf("packets: %u\n", current_messages.data_transmission_handshake.packets);
                            }break;

                            case payload:
                            {
                                data->push_back((float) current_messages.data_transmission_handshake.payload);
                                printf("payload: %u\n", current_messages.data_transmission_handshake.payload);
                            }break;

                            case jpg_quality:
                            {
                                data->push_back(    (float) current_messages.data_transmission_handshake.jpg_quality);
                                printf("jpg_quality: %u\n", current_messages.data_transmission_handshake.jpg_quality);
                            }break;

                            default:
                                break;
                        }
                    }
                    break;
                }

                case MAVLINK_MSG_ID_ENCAPSULATED_DATA: // #131
                {
                    // uint16_t seqnr
                    // uint8_t data[253]

                    // printf("MAVLINK_MSG_ID_ENCAPSULATED_DATA\n");
                    mavlink_msg_encapsulated_data_decode(&message, &(current_messages.encapsulated_data));
                    current_messages.time_stamps.encapsulated_data = get_time_usec();
                    this_timestamps.encapsulated_data = current_messages.time_stamps.encapsulated_data;

                    // Se os pacotes estiverem em sincronia
                    if (current_messages.encapsulated_data.seqnr == packet){
                        
                        // Atualiza o nº do pacote
                        packet++;
                        
                        // Copia os dados da imagem para o vetor
                        for (int i = 0; i < current_messages.data_transmission_handshake.payload; ++i){
                            imgVector.push_back(current_messages.encapsulated_data.data[i]);

                            // imgPointer[i] = current_messages.encapsulated_data.data[i]; // memcpy ?
                        }

                        // memcpy(&(imgPointer + current_messages.data_transmission_handshake.payload*current_messages.encapsulated_data.seqnr), &current_messages.encapsulated_data.data, current_messages.data_transmission_handshake.payload * sizeof(uint8_t));

                    }else{ // Reset
                        packet = 0;
                        imgVector.clear();
                        // free(imgPointer);
                    }

                    // if({0} && {1})
                    // {0}: verifica se o nº de pacotes já extrapolou o da imagem -> isto é comum nas imagens maiores que 64x64
                    // {1}: evita 'segmentation fault' ao acessar o vetor imgVector
                    if(packet >= (current_messages.data_transmission_handshake.packets - 1) && (imgVector.size() >= current_messages.data_transmission_handshake.size)){

                        std::cout
                                << "\nGerando imagem\n"
                                << "packet: [" << packet << "]\n"
                                << "packets: " << current_messages.data_transmission_handshake.packets << "\n"
                                << "img.height: " << current_messages.data_transmission_handshake.height << "\n"
                                << "img.width: " << current_messages.data_transmission_handshake.width << "\n"
                                << "img pixel count: " << current_messages.data_transmission_handshake.height*current_messages.data_transmission_handshake.width << "\n"
                                << "vector.size: " << imgVector.size() << "\n\n";

                        // Predefinição da imagem
                        *img = cv::Mat(
                                current_messages.data_transmission_handshake.height,
                                current_messages.data_transmission_handshake.width,
                                CV_8UC1);

                        // Formação da imagem: copia os dados do std::vector para a matriz cv::Mat
                        for (register unsigned int y = 0; y < img->rows; y++)
                            for (register unsigned int x = 0; x < img->cols; x++)
                                // Mapeia o vetor unidimensional na matriz da imagem
                                img->at<uchar>(y,x) = imgVector[x + y*img->cols];

                        // Reset
                        packet = 0;
                        imgVector.clear();
                        // free(imgPointer);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_OPTICAL_FLOW: // #100
                {
                    // uint64_t time_usec
                    // float flow_comp_m_x
                    // float flow_comp_m_y
                    // float ground_distance
                    // int16_t flow_x
                    // int16_t flow_y
                    // uint8_t sensor_id
                    // uint8_t quality

                    // printf("MAVLINK_MSG_ID_OPTICAL_FLOW\n");
                    mavlink_msg_optical_flow_decode(&message, &(current_messages.optical_flow));
                    current_messages.time_stamps.optical_flow = get_time_usec();
                    this_timestamps.optical_flow = current_messages.time_stamps.optical_flow;

                    enum Fields{
                        time_usec, sensor_id, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance
                    };

                    switch(msgUserFieldName)
                    {
                        case time_usec:
                        {
                            data->push_back(    (float) current_messages.optical_flow.time_usec); // (double) ?
                            printf("time_usec: %lu\n",  current_messages.optical_flow.time_usec);
                        }break;

                        case sensor_id:
                        {
                            data->push_back(    (float) current_messages.optical_flow.sensor_id);
                            printf("sensor_id: %u\n",   current_messages.optical_flow.sensor_id);
                        }break;

                        case flow_x:
                        {
                            data->push_back((float) current_messages.optical_flow.flow_x);
                            printf("flow_x: %d\n",  current_messages.optical_flow.flow_x);
                        }break;

                        case flow_y:
                        {
                            data->push_back((float) current_messages.optical_flow.flow_y);
                            printf("flow_y: %d\n",  current_messages.optical_flow.flow_y);
                        }break;

                        case flow_comp_m_x:
                        {
                            data->push_back(                current_messages.optical_flow.flow_comp_m_x);
                            printf("flow_comp_m_x: %f\n",   current_messages.optical_flow.flow_comp_m_x);
                        }break;

                        case flow_comp_m_y:
                        {
                            data->push_back(                current_messages.optical_flow.flow_comp_m_y);
                            printf("flow_comp_m_y: %f\n",   current_messages.optical_flow.flow_comp_m_y);
                        }break;

                        case quality:
                        {
                            data->push_back((float) current_messages.optical_flow.quality);
                            printf("quality: %u\n", current_messages.optical_flow.quality);
                        }break;

                        case ground_distance:
                        {
                            data->push_back(                current_messages.optical_flow.ground_distance);
                            printf("ground_distance: %f\n", current_messages.optical_flow.ground_distance);
                        }break;

                        default:
                            break;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: // #106
                {
                    // uint64_t time_usec
                    // uint32_t integration_time_us
                    // float integrated_x
                    // float integrated_y
                    // float integrated_xgyro
                    // float integrated_ygyro
                    // float integrated_zgyro
                    // uint32_t time_delta_distance_us
                    // float distance
                    // int16_t temperature
                    // uint8_t sensor_id
                    // uint8_t quality

                    // printf("MAVLINK_MSG_ID_OPTICAL_FLOW_RAD\n");
                    mavlink_msg_optical_flow_rad_decode(&message, &(current_messages.optical_flow_rad));
                    current_messages.time_stamps.optical_flow_rad = get_time_usec();
                    this_timestamps.optical_flow_rad = current_messages.time_stamps.optical_flow_rad;

                    enum Fields{
                        time_usec, sensor_id, integration_time_us, integrated_x, integrated_y, integrated_xgyro,
                        integrated_ygyro, integrated_zgyro, temperature, quality, time_delta_distance_us, distance
                    };

                    switch(msgUserFieldName)
                    {
                        case time_usec:
                        {
                            data->push_back(    (float) current_messages.optical_flow_rad.time_usec);
                            printf("time_usec: %lu\n",  current_messages.optical_flow_rad.time_usec);
                        }break;

                        case sensor_id:
                        {
                            data->push_back(    (float) current_messages.optical_flow_rad.sensor_id);
                            printf("sensor_id: %u\n",   current_messages.optical_flow_rad.sensor_id);
                        }break;

                        case integration_time_us:
                        {
                            data->push_back(            (float) current_messages.optical_flow_rad.integration_time_us);
                            printf("integration_time_us: %u\n", current_messages.optical_flow_rad.integration_time_us);
                        }break;

                        case integrated_x:
                        {
                            data->push_back(            current_messages.optical_flow_rad.integrated_x);
                            printf("integrated_x: %f\n",current_messages.optical_flow_rad.integrated_x);
                        }break;

                        case integrated_y:
                        {
                            data->push_back(            current_messages.optical_flow_rad.integrated_y);
                            printf("integrated_y: %f\n",current_messages.optical_flow_rad.integrated_y);
                        }break;

                        case integrated_xgyro:
                        {
                            data->push_back(                current_messages.optical_flow_rad.integrated_xgyro);
                            printf("integrated_xgyro: %f\n",current_messages.optical_flow_rad.integrated_xgyro);
                        }break;

                        case integrated_ygyro:
                        {
                            data->push_back(                current_messages.optical_flow_rad.integrated_ygyro);
                            printf("integrated_ygyro: %f\n",current_messages.optical_flow_rad.integrated_ygyro);
                        }break;

                        case integrated_zgyro:
                        {
                            data->push_back(                current_messages.optical_flow_rad.integrated_zgyro);
                            printf("integrated_zgyro: %f\n",current_messages.optical_flow_rad.integrated_zgyro);
                        }break;

                        case temperature:
                        {
                            data->push_back(    (float) current_messages.optical_flow_rad.temperature);
                            printf("temperature: %d\n", current_messages.optical_flow_rad.temperature);
                        }break;

                        case quality:
                        {
                            data->push_back((float) current_messages.optical_flow_rad.quality);
                            printf("quality: %u\n", current_messages.optical_flow_rad.quality);
                        }break;

                        case time_delta_distance_us:
                        {
                            data->push_back(                (float) current_messages.optical_flow_rad.time_delta_distance_us);
                            printf("time_delta_distance_us: %u\n",  current_messages.optical_flow_rad.time_delta_distance_us);
                        }break;

                        case distance:
                        {
                            data->push_back(        current_messages.optical_flow_rad.distance);
                            printf("distance: %f\n",current_messages.optical_flow_rad.distance);
                        }break;

                        default:
                            break;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_DEBUG_VECT: // #250
                {
                    // uint64_t time_usec
                    // float x
                    // float y
                    // float z
                    // char name[10]

                    // printf("MAVLINK_MSG_ID_DEBUG_VECT\n");
                    mavlink_msg_debug_vect_decode(&message, &(current_messages.debug_vect));
                    current_messages.time_stamps.debug_vect = get_time_usec();
                    this_timestamps.debug_vect = current_messages.time_stamps.debug_vect;

                    enum Fields{
                        name, time_usec, x, y, z
                    };

                    switch(msgUserFieldName)
                    {
                        case name:
                        {
                            printf("name: %s\n", current_messages.debug_vect.name);
                        }break;

                        case time_usec:
                        {
                            data->push_back(    (float) current_messages.debug_vect.time_usec);
                            printf("time_usec: %lu\n",  current_messages.debug_vect.time_usec);
                        }break;

                        case x:
                        {
                            data->push_back(    current_messages.debug_vect.x);
                            printf("x: %f\n",   current_messages.debug_vect.x);
                        }break;

                        case y:
                        {
                            data->push_back(    current_messages.debug_vect.y);
                            printf("y: %f\n",   current_messages.debug_vect.y);
                        }break;

                        case z:
                        {
                            data->push_back(    current_messages.debug_vect.z);
                            printf("z: %f\n",   current_messages.debug_vect.z);
                        }break;

                        default:
                            break;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: // #251
                {
                    // uint32_t time_boot_ms
                    // float value
                    // char name[10]

                    // printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOAT\n");
                    mavlink_msg_named_value_float_decode(&message, &(current_messages.named_value_float));
                    current_messages.time_stamps.named_value_float = get_time_usec();
                    this_timestamps.named_value_float = current_messages.time_stamps.named_value_float;

                    enum Fields{
                        time_boot_ms, name, value
                    };

                    switch(msgUserFieldName)
                    {
                        case time_boot_ms:
                        {
                            data->push_back((float) current_messages.named_value_float.time_boot_ms);
                            printf("time_boot_ms: %u\n", current_messages.named_value_float.time_boot_ms);
                        }break;

                        case name:
                        {
                            printf("name: %s\n", current_messages.named_value_float.name);
                        }break;

                        case value:
                        {
                            data->push_back(        current_messages.named_value_float.value);
                            printf("value: %f\n",   current_messages.named_value_float.value);
                        }break;

                        default:
                            break;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_NAMED_VALUE_INT: // #252
                {
                    // uint32_t time_boot_ms
                    // int32_t value
                    // char name[10]

                    // printf("MAVLINK_MSG_ID_NAMED_VALUE_INT\n");
                    mavlink_msg_named_value_int_decode(&message, &(current_messages.named_value_int));
                    current_messages.time_stamps.named_value_int = get_time_usec();
                    this_timestamps.named_value_int = current_messages.time_stamps.named_value_int;

                    enum Fields{
                        time_boot_ms, name, value
                    };

                    switch(msgUserFieldName)
                    {
                        case time_boot_ms:
                        {
                            data->push_back((float) current_messages.named_value_int.time_boot_ms);
                            printf("time_boot_ms: %u\n", current_messages.named_value_int.time_boot_ms);
                        }break;

                        case name:
                        {
                            printf("name: %s\n", current_messages.named_value_int.name);
                        }break;

                        case value:
                        {
                            data->push_back((float) current_messages.named_value_int.value);
                            printf("value: %d\n", current_messages.named_value_int.value);
                        }break;

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
        if ( writing_status > false )
         usleep(100); // look for components of batches at 10kHz

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
        // usleep(500000); // check at 2Hz
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
        printf("GOT AUTOPILOT COMPONENT ID: %i\n\n", px4flow_id);
    }

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    printf("START WRITE THREAD \n");

    result = pthread_create( &write_tid, NULL, &start_px4flow_interface_write_thread, this );
    if ( result ) throw result;

    // Configura o tipo da imagem
    if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA)
        this->set_video_only();

    // // wait for it to be started
    // while ( not writing_status )
    //     usleep(100000); // 10Hz

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
    //             MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
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
    //  usleep(250000);   // Stream at 4Hz
    //  write_setpoint();
    // }

    // // signal end
    // writing_status = false;

    return;

}

// ------------------------------------------------------------------------------
//   Set Video Only
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
set_video_only(void)
{
    // Configura o tipo da imagem
    // se msgUserFieldName == True: full size image
    // se msgUserFieldName == False: 64x64 px
        
    mavlink_message_t msg;
    mavlink_param_set_t param;

    param.param_value = msgUserFieldName; // True | False
    param.target_system = system_id;
    param.target_component = px4flow_id;
    // param.param_type;

    // https://github.com/PX4/Flow/blob/master/src/settings.c
    //      global_data.param_name[PARAM_VIDEO_ONLY]
    //          "VIDEO_ONLY"
    strcpy(param.param_id, "VIDEO_ONLY");
        
    mavlink_msg_param_set_encode(this->system_id, this->px4flow_id, &msg, &param);
    this->write_message(msg);
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