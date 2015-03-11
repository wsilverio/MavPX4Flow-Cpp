/****************************************************************************
 *
 *        Por
 *           Wendeurick Silverio <Twitter @obelonave>, <GitHub @wsilverio>
 *           Disponível em https://github.com/wsilverio/MavPX4Flow-Cpp
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
PX4Flow_Interface(Serial_Port *serial_port_, int msgID_, int msgFieldName_, bool state)
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
    
    packet = 0;

    debug = state;

    // Start mutex
    int result = pthread_mutex_init(&trava, NULL);
    if (result != 0){
        printf("\n mutex init failed\n");
        throw EXIT_FAILURE;
    }    
}

PX4Flow_Interface::
~PX4Flow_Interface()
{
    // destroy mutex
    pthread_mutex_destroy(&trava);
}

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

            if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA and message.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE){ // or message.msgid == MAVLINK_MSG_ID_PARAM_VALUE){
                // pass
            }else if(message.msgid != msgUserID)
                continue;

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

                    // if(debug) printf("MAVLINK_MSG_ID_HEARTBEAT\n");
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
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.type);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("type: %u\n", current_messages.heartbeat.type);
                        }break;

                        case autopilot:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.autopilot);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("autopilot: %u\n", current_messages.heartbeat.autopilot);
                        }break; 

                        case base_mode:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.base_mode);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("base_mode: %u\n", current_messages.heartbeat.base_mode);
                        }break;

                        case custom_mode:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.custom_mode);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("custom_mode: %u\n", current_messages.heartbeat.custom_mode);
                        }break;

                        case system_status:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.system_status);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("system_status: %u\n", current_messages.heartbeat.system_status);
                        }break;

                        case mavlink_version:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.heartbeat.mavlink_version);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("link_version: %u\n", current_messages.heartbeat.mavlink_version);
                        }break;

                        default: 
                            break;
                    }

                    break;
                }

                case MAVLINK_MSG_ID_PARAM_VALUE: // #22
                {
                    // float param_value
                    // uint16_t param_count
                    // uint16_t param_index
                    // char param_id[16]
                    // uint8_t param_type

                    // if(debug) printf("MAVLINK_PARAM_VALUE\n");
                    mavlink_msg_param_value_decode(&message, &(current_messages.param_value));
                    current_messages.time_stamps.param_value = get_time_usec();
                    this_timestamps.param_value = current_messages.time_stamps.param_value;

                    if(debug) printf( "param_value: %f\n"
                            "param_count: %u\n"
                            "param_index: %u\n"
                            "param_id: %s\n"
                            "param_type: %u\n\n",
                            current_messages.param_value.param_value,
                            current_messages.param_value.param_count,
                            current_messages.param_value.param_index,
                            current_messages.param_value.param_id,
                            current_messages.param_value.param_type);

                    if(current_messages.vector_param_value.size() < current_messages.param_value.param_count)
                        current_messages.vector_param_value.resize(current_messages.param_value.param_count);

                    current_messages.vector_param_value[current_messages.param_value.param_index] = current_messages.param_value;

                    // if(debug) printf( "param_value: %f\n"
                    //         "param_count: %u\n"
                    //         "param_index: %u\n"
                    //         "param_id: %s\n"
                    //         "param_type: %u\n\n",
                    //         current_messages.vector_param_value[current_messages.param_value.param_index].param_value,
                    //         current_messages.vector_param_value[current_messages.param_value.param_index].param_count,
                    //         current_messages.vector_param_value[current_messages.param_value.param_index].param_index,
                    //         current_messages.vector_param_value[current_messages.param_value.param_index].param_id,
                    //         current_messages.vector_param_value[current_messages.param_value.param_index].param_type);

                    pthread_mutex_lock(&trava);
                    data.push_back((float) current_messages.param_value.param_index);
                    pthread_mutex_unlock(&trava);

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

                    // if(debug) printf("MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE\n");
                    mavlink_msg_data_transmission_handshake_decode(&message, &(current_messages.data_transmission_handshake));
                    current_messages.time_stamps.data_transmission_handshake = get_time_usec();
                    this_timestamps.data_transmission_handshake = current_messages.time_stamps.data_transmission_handshake;

                    if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA && debug){
                        std::cout 
                                << "type: " << (unsigned int) current_messages.data_transmission_handshake.type
                                << "\nsize: " << current_messages.data_transmission_handshake.size
                                << "\nwidth: " << current_messages.data_transmission_handshake.width
                                << "\nheight: " << current_messages.data_transmission_handshake.height
                                << "\npackets: " << current_messages.data_transmission_handshake.packets
                                << "\npayload: " << (unsigned int) current_messages.data_transmission_handshake.payload
                                << "\njpg_quality: " << (unsigned int) current_messages.data_transmission_handshake.jpg_quality << "\n\n";

                    }else{
                        enum Fields{
                            type, size, width, height, packets, payload, jpg_quality
                        };

                        switch(msgUserFieldName)
                        {
                            case type:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.type);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("type: %u\n", current_messages.data_transmission_handshake.type);
                            }break;

                            case size:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.size);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("size: %u\n", current_messages.data_transmission_handshake.size);
                            }break;

                            case width:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.width);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("width: %u\n", current_messages.data_transmission_handshake.width);
                            }break;

                            case height:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.height);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("height: %u\n", current_messages.data_transmission_handshake.height);
                            }break;

                            case packets:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.packets);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("packets: %u\n", current_messages.data_transmission_handshake.packets);
                            }break;

                            case payload:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.payload);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("payload: %u\n", current_messages.data_transmission_handshake.payload);
                            }break;

                            case jpg_quality:
                            {
                                pthread_mutex_lock(&trava);
                                data.push_back((float) current_messages.data_transmission_handshake.jpg_quality);
                                pthread_mutex_unlock(&trava);
                                if(debug) printf("jpg_quality: %u\n", current_messages.data_transmission_handshake.jpg_quality);
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

                    // if(debug) printf("MAVLINK_MSG_ID_ENCAPSULATED_DATA\n");
                    mavlink_msg_encapsulated_data_decode(&message, &(current_messages.encapsulated_data));
                    current_messages.time_stamps.encapsulated_data = get_time_usec();
                    this_timestamps.encapsulated_data = current_messages.time_stamps.encapsulated_data;

                    // Se os pacotes estiverem em sincronia
                    if (current_messages.encapsulated_data.seqnr == packet){
                        // Atualiza o nº do pacote
                        packet++;
                        
                        // Copia os dados da imagem para o vetor
                        for (int i = 0; i < current_messages.data_transmission_handshake.payload; ++i)
                            imgVector.push_back(current_messages.encapsulated_data.data[i]);

                    }else{ // Reset
                        packet = 0;
                        imgVector.clear();
                    }

                    // if({0} && {1})
                    // {0}: verifica se o nº de pacotes já extrapolou o da imagem -> isto é comum nas imagens maiores que 64x64
                    // {1}: evita 'segmentation fault' ao acessar o vetor imgVector
                    if(packet >= (current_messages.data_transmission_handshake.packets - 1) && (imgVector.size() >= current_messages.data_transmission_handshake.size)){

                        if(debug)   std::cout
                                    << "\nGerando imagem\n"
                                    << "packet: [" << packet << "]\n"
                                    << "packets: " << current_messages.data_transmission_handshake.packets << "\n"
                                    << "img.height: " << current_messages.data_transmission_handshake.height << "\n"
                                    << "img.width: " << current_messages.data_transmission_handshake.width << "\n"
                                    << "img pixel count: " << current_messages.data_transmission_handshake.height*current_messages.data_transmission_handshake.width << "\n"
                                    << "vector.size: " << imgVector.size() << "\n\n";

                        // Predefinição da imagem buffer
                        cv::Mat imgTemp(current_messages.data_transmission_handshake.height,
                                        current_messages.data_transmission_handshake.width,
                                        CV_8UC1);
                        
                        // Formação da imagem: copia os dados do std::vector para a matriz cv::Mat
                        for (register unsigned int y = 0; y < imgTemp.rows; y++)
                            for (register unsigned int x = 0; x < imgTemp.cols; x++)
                                // Mapeia o vetor unidimensional na matriz da imagem
                                imgTemp.at<uchar>(y,x) = imgVector[x + y*imgTemp.cols];

                        if(msgUserFieldName){ // VIDEO_ONLY set
                            // cria uma cópia da imagem
                            pthread_mutex_lock(&trava);
                            // imgTemp.copyTo(img);
                            cv::resize(imgTemp, img, cv::Size(2*376, 2*240));
                            pthread_mutex_unlock(&trava);
                        }else{
                            // redimensiona a imagem: Size(64, 64) -> Size(480, 480), bilinear interpolation
                            pthread_mutex_lock(&trava);
                            cv::resize(imgTemp, img, cv::Size(480, 480));
                            pthread_mutex_unlock(&trava);
                        }

                        // Reset
                        packet = 0;
                        imgVector.clear();
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

                    // if(debug) printf("MAVLINK_MSG_ID_OPTICAL_FLOW\n");
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
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow.time_usec); // (double) ?
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_usec: %lu\n", current_messages.optical_flow.time_usec);
                        }break;

                        case sensor_id:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow.sensor_id);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("sensor_id: %u\n", current_messages.optical_flow.sensor_id);
                        }break;

                        case flow_x:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow.flow_x);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("flow_x: %d\n", current_messages.optical_flow.flow_x);
                        }break;

                        case flow_y:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow.flow_y);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("flow_y: %d\n", current_messages.optical_flow.flow_y);
                        }break;

                        case flow_comp_m_x:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow.flow_comp_m_x);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("flow_comp_m_x: %f\n", current_messages.optical_flow.flow_comp_m_x);
                        }break;

                        case flow_comp_m_y:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow.flow_comp_m_y);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("flow_comp_m_y: %f\n", current_messages.optical_flow.flow_comp_m_y);
                        }break;

                        case quality:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow.quality);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("quality: %u\n", current_messages.optical_flow.quality);
                        }break;

                        case ground_distance:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow.ground_distance);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("ground_distance: %f\n", current_messages.optical_flow.ground_distance);
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

                    // if(debug) printf("MAVLINK_MSG_ID_OPTICAL_FLOW_RAD\n");
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
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.time_usec);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_usec: %lu\n", current_messages.optical_flow_rad.time_usec);
                        }break;

                        case sensor_id:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.sensor_id);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("sensor_id: %u\n", current_messages.optical_flow_rad.sensor_id);
                        }break;

                        case integration_time_us:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.integration_time_us);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integration_time_us: %u\n", current_messages.optical_flow_rad.integration_time_us);
                        }break;

                        case integrated_x:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.integrated_x);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integrated_x: %f\n", current_messages.optical_flow_rad.integrated_x);
                        }break;

                        case integrated_y:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.integrated_y);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integrated_y: %f\n", current_messages.optical_flow_rad.integrated_y);
                        }break;

                        case integrated_xgyro:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.integrated_xgyro);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integrated_xgyro: %f\n", current_messages.optical_flow_rad.integrated_xgyro);
                        }break;

                        case integrated_ygyro:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.integrated_ygyro);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integrated_ygyro: %f\n", current_messages.optical_flow_rad.integrated_ygyro);
                        }break;

                        case integrated_zgyro:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.integrated_zgyro);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("integrated_zgyro: %f\n", current_messages.optical_flow_rad.integrated_zgyro);
                        }break;

                        case temperature:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.temperature);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("temperature: %d\n", current_messages.optical_flow_rad.temperature);
                        }break;

                        case quality:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.quality);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("quality: %u\n", current_messages.optical_flow_rad.quality);
                        }break;

                        case time_delta_distance_us:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.optical_flow_rad.time_delta_distance_us);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_delta_distance_us: %u\n", current_messages.optical_flow_rad.time_delta_distance_us);
                        }break;

                        case distance:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.optical_flow_rad.distance);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("distance: %f\n", current_messages.optical_flow_rad.distance);
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

                    // if(debug) printf("MAVLINK_MSG_ID_DEBUG_VECT\n");
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
                            if(debug) printf("name: %s\n", current_messages.debug_vect.name);
                        }break;

                        case time_usec:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.debug_vect.time_usec);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_usec: %lu\n", current_messages.debug_vect.time_usec);
                        }break;

                        case x:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.debug_vect.x);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("x: %f\n", current_messages.debug_vect.x);
                        }break;

                        case y:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.debug_vect.y);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("y: %f\n", current_messages.debug_vect.y);
                        }break;

                        case z:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.debug_vect.z);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("z: %f\n", current_messages.debug_vect.z);
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

                    // if(debug) printf("MAVLINK_MSG_ID_NAMED_VALUE_FLOAT\n");
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
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.named_value_float.time_boot_ms);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_boot_ms: %u\n", current_messages.named_value_float.time_boot_ms);
                        }break;

                        case name:
                        {
                            if(debug) printf("name: %s\n", current_messages.named_value_float.name);
                        }break;

                        case value:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back(current_messages.named_value_float.value);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("value: %f\n", current_messages.named_value_float.value);
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

                    // if(debug) printf("MAVLINK_MSG_ID_NAMED_VALUE_INT\n");
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
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.named_value_int.time_boot_ms);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("time_boot_ms: %u\n", current_messages.named_value_int.time_boot_ms);
                        }break;

                        case name:
                        {
                            if(debug) printf("name: %s\n", current_messages.named_value_int.name);
                        }break;

                        case value:
                        {
                            pthread_mutex_lock(&trava);
                            data.push_back((float) current_messages.named_value_int.value);
                            pthread_mutex_unlock(&trava);
                            if(debug) printf("value: %d\n", current_messages.named_value_int.value);
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


    if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA){
        // Configura o tipo da imagem
        this->set_video_only(msgUserFieldName);
    }
    else{
        // Desabilita VIDEO_ONLY
        this->set_video_only(0);
        
        if(msgUserID == MAVLINK_MSG_ID_PARAM_VALUE){
            // Solicita todos os parâmetros da PX4Flow
            this->param_request_list();
        }
    }

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
    // Caso esteja no modo VIDEO_ONLY
    if(msgUserID == MAVLINK_MSG_ID_ENCAPSULATED_DATA && msgUserFieldName){
        // Desabilita VIDEO_ONLY
        if (debug) printf("Desabilitando modo VIDEO_ONLY\n");
        this->set_video_only(0);
        usleep(500000); // 500ms
    }

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
//   Param Request List
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
param_request_list(void)
{
    mavlink_message_t msg;
    mavlink_param_request_list_t param_req_lst;

    param_req_lst.target_system = system_id;
    param_req_lst.target_component = px4flow_id;

    mavlink_msg_param_request_list_encode(this->system_id, this->px4flow_id, &msg, &param_req_lst);
    this->write_message(msg);
}

// ------------------------------------------------------------------------------
//   Set Video Only
// ------------------------------------------------------------------------------
void
PX4Flow_Interface::
set_video_only(float val)
{
    // Configura o tipo da imagem
    // se msgUserFieldName == True: full size image
    // se msgUserFieldName == False: 64x64 px
        
    mavlink_message_t msg;
    mavlink_param_set_t param;

    param.param_value = val;
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