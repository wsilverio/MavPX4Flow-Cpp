/*
    Baseado em https://github.com/mavlink/c_uart_interface_example
*/

#include <stdio.h>  // printf
#include <cstdlib>  // EXIT_FAILURE
#include <signal.h> // signal

#include <common/mavlink.h> // msgID
#include "serial_port.h" // Serial_Port
#include "px4flow_interface.h" // PX4Flow_Interface

#include <vector>
#include <iostream>
#include <string> // std::to_string

#include <opencv2/opencv.hpp>

// Global
Serial_Port *serial_port_quit; // função quit_handler()
PX4Flow_Interface *px4flow_interface_quit; // função quit_handler()

// ------------------------------------------------------------------------------
//  Parse Command Line
// ------------------------------------------------------------------------------
//  Compara as entradas da linha de comando com as opções do programa:
//      "-h" ou "--help": exibe a mensagem de ajuda
//      "-d": configura o dispositivo serial
//      "-b": configura a taxa de comunicação (bps)
//      "-id": configura o ID da mensagem Mavlink
void Argparse(int argc, char **argv, char *&uart_name, int &baudrate){

    const char *help_msg = "Uso:\n$ ./mavpx4flow.run -d <dispositivo> -b <baudrate>\n";

    if(argc < 2){
        printf("%s\n", help_msg);
        throw EXIT_FAILURE;
    }

    for (int i = 1; i < argc; i++) { // argv[0] is "mavpx4.run"

        // Help
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")){
            printf("%s\n", help_msg);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (!strcmp(argv[i], "-d")){
            if (argc > i + 1) {
                uart_name = argv[i + 1];
            } else {
                printf("%s\n", help_msg);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (!strcmp(argv[i], "-b")){
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);
            } else {
                printf("%s\n", help_msg);
                throw EXIT_FAILURE;
            }
        }
    }

    printf("Device: %s\nBaud rate: %d\n", uart_name, baudrate);
    return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// Esta função é chamada quando o usuário pressiona ^C (Ctrl-C)
void Quit_Handler(int sig){

    printf("\n\n### PEDIDO DE TÉRMINO DE EXECUÇÃO ###\n\n");

    try {
        px4flow_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    exit(EXIT_SUCCESS);
}

// ------------------------------------------------------------------------------
//   Map - https://processing.org/reference/map_.html
// ------------------------------------------------------------------------------
// Re-maps a number from one range to another.
float Map(float value, float start1, float stop1, float start2, float stop2){
    return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
}

// ------------------------------------------------------------------------------
//   Vetor - mínimo e máximo
// ------------------------------------------------------------------------------
// Armazena o maior e o menor elemento de um vetor
int MinMaxElement(std::vector<int> & v, int & min, int & max) {
    min = max = v[0];

    for (register unsigned int i = 1; i < v.size(); ++i) {
        if (v[i] < min) min = v[i];
        else if (v[i] > max) max = v[i];
    }
}

// ------------------------------------------------------------------------------
//   Vetor - mínimo e máximo
// ------------------------------------------------------------------------------
// Armazena o maior e o menor elemento de um vetor
float MinMaxElement(std::vector<float> & v, float & min, float & max) {
    min = max = v[0];

    for (register unsigned int i = 1; i < v.size(); ++i) {
        if (v[i] < min) min = v[i];
        else if (v[i] > max) max = v[i];
    }
}

int main(int argc, char **argv){

    // Padrão
    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 57600;

    // Tratamento da linha de comando
    Argparse(argc, argv, uart_name, baudrate);

    int msgID, msgFieldName, msgFieldType;

    std::string windowName;

    try
    {
        
        enum MSGID{
            HEARTBEAT,
            DATA_TRANSMISSION_HANDSHAKE,
            ENCAPSULATED_DATA,
            OPTICAL_FLOW,
            OPTICAL_FLOW_RAD,
            DEBUG_VECT,
            NAMED_VALUE_FLOAT,
            NAMED_VALUE_INT
        };

        enum MSGFIELDTYPE{
            VECTOR_UINT64_T,
            VECTOR_INT32_T,
            VECTOR_UINT32_T,
            VECTOR_INT16_T,
            VECTOR_UINT16_T,
            VECTOR_UINT8_T,
            VECTOR_FLOAT,
            ARRAY_UINT8_T,
            ARRAY_CHAR
        };

        #define TAM 8
        const char *fields[TAM] = 
        {
            "HEARTBEAT",
            "DATA_TRANSMISSION_HANDSHAKE",
            "ENCAPSULATED_DATA",
            "OPTICAL_FLOW",
            "OPTICAL_FLOW_RAD",
            "DEBUG_VECT",
            "NAMED_VALUE_FLOAT",
            "NAMED_VALUE_INT"
        };

        printf("\n### MavPX4Flow ###\n");
        printf("Escolha a mensagem Mavlink:\n\n");

        for (int i = 0; i < TAM; ++i){
            printf("{%d} %s\n", i, fields[i]);
        }

        printf("Mensagem: ");

        msgID = -1;
        scanf("%d", &msgID);

        switch(msgID){
            case HEARTBEAT:
            {
                msgID = MAVLINK_MSG_ID_HEARTBEAT;
                windowName = "HEARTBEAT: ";

                #define TAM 6
                const char *fields[TAM] = {"type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"};
                const int types[TAM] = {VECTOR_UINT8_T, VECTOR_UINT8_T, VECTOR_UINT8_T, VECTOR_UINT32_T, VECTOR_UINT8_T, VECTOR_UINT8_T};

                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case DATA_TRANSMISSION_HANDSHAKE:
            {
                msgID = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
                windowName = "DATA_TRANSMISSION_HANDSHAKE: ";

                #define TAM 7
                const char *fields[TAM] = {"type", "size", "width", "height", "packets", "payload", "jpg_quality"};
                const int types[TAM] = {VECTOR_UINT8_T, VECTOR_UINT32_T, VECTOR_UINT16_T, VECTOR_UINT16_T, VECTOR_UINT16_T, VECTOR_UINT8_T, VECTOR_UINT8_T};

                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case ENCAPSULATED_DATA:
            {
                msgID = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
                windowName = "ENCAPSULATED_DATA: ";

                #define TAM 2
                const char *fields[TAM] = {"seqnr", "data"};
                const int types[TAM] = {VECTOR_UINT16_T, ARRAY_UINT8_T};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");

                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case OPTICAL_FLOW:
            {
                msgID = MAVLINK_MSG_ID_OPTICAL_FLOW;
                windowName = "OPTICAL_FLOW: ";

                #define TAM 8
                const char *fields[TAM] = {"time_usec", "sensor_id", "flow_x", "flow_y", "flow_comp_m_x", "flow_comp_m_y", "quality", "ground_distance"};
                const int types[TAM] = {VECTOR_UINT64_T, VECTOR_UINT8_T, VECTOR_INT16_T, VECTOR_INT16_T, VECTOR_FLOAT, VECTOR_FLOAT, VECTOR_UINT8_T, VECTOR_FLOAT};

                printf("\nEscolha o Field Name da mensagem:\n\n");

                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case OPTICAL_FLOW_RAD:
            {
                msgID = MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
                windowName = "OPTICAL_FLOW_RAD: ";

                #define TAM 12
                const char *fields[TAM] = { "time_usec", "sensor_id", "integration_time_us", "integrated_x", "integrated_y", "integrated_xgyro",
                                            "integrated_ygyro", "integrated_zgyro", "temperature", "quality", "time_delta_distance_us", "distance"};
                const int types[TAM] = {VECTOR_UINT64_T, VECTOR_UINT8_T, VECTOR_UINT32_T, VECTOR_FLOAT, VECTOR_FLOAT, VECTOR_FLOAT,
                                        VECTOR_FLOAT, VECTOR_FLOAT, VECTOR_INT16_T, VECTOR_UINT8_T, VECTOR_UINT32_T, VECTOR_FLOAT};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");

                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case DEBUG_VECT:
            {
                msgID = MAVLINK_MSG_ID_DEBUG_VECT;
                windowName = "DEBUG_VECT: ";

                #define TAM 5
                const char *fields[TAM] = {"name", "time_usec", "x", "y", "z"};
                const int types[TAM] = {ARRAY_CHAR, VECTOR_UINT64_T, VECTOR_FLOAT};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");                
                
                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case NAMED_VALUE_FLOAT:
            {
                msgID = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
                windowName = "NAMED_VALUE_FLOAT: ";

                #define TAM 3
                const char *fields[TAM] = {"time_boot_ms", "name", "value"};
                const int types[TAM] = {VECTOR_UINT32_T, ARRAY_CHAR, VECTOR_FLOAT};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");  

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            case NAMED_VALUE_INT:
            {
                msgID = MAVLINK_MSG_ID_NAMED_VALUE_INT;
                windowName = "NAMED_VALUE_INT: ";

                #define TAM 3
                const char *fields[TAM] = {"time_boot_ms", "name", "value"};
                const int types[TAM] = {VECTOR_UINT32_T, ARRAY_CHAR, VECTOR_INT32_T};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");                 

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                msgFieldType = types[msgFieldName];
                windowName += fields[msgFieldName];

            }
            break;

            default:
            {
                printf("\n*** Erro: mensagem ainda não implementada ***\n\n");
                throw EXIT_FAILURE;
            }
            break; // ?
        }

        // std::vector<float> data;
        Common_Types Data;

        Serial_Port serial_port(uart_name, baudrate);
        serial_port_quit = &serial_port; // função Quit_Handler()

        PX4Flow_Interface px4flow(&serial_port, msgID, msgFieldName, &Data);
        px4flow_interface_quit = &px4flow; // função Quit_Handler()

        // Associa ^C à função Quit_Handler()
        signal(SIGINT, Quit_Handler);

        // Abre a porta e inicializa a comunicação serial
        serial_port.start();
        // Inicializa a comunicação Mavlink (thread)
        px4flow.start();

        cv::Mat imagemPlot;
        cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);

        switch(msgFieldType){

            case VECTOR_UINT64_T:
            {

            }break;

            case VECTOR_INT32_T:
            {

            }break;

            case VECTOR_UINT32_T:
            {

            }break;

            case VECTOR_INT16_T:
            {

            }break;

            case VECTOR_UINT16_T:
            {

            }break;

            case VECTOR_UINT8_T:
            {

            }break;

            case VECTOR_FLOAT:
            {

            }break;

            case ARRAY_UINT8_T:
            {
                
            }break;

            case ARRAY_CHAR:
            {

            }break;

            default:
            {

            }break;

        }

        // for(;;){

        //     #define WIDTH 1280
        //     #define HEIGHT 720

        //     imagemPlot = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(54,54,54));

        //     if (data.size() > WIDTH)
        //         data.erase(data.begin());

        //     float min, max;
        //     MinMaxElement(data, min, max);

        //     for(int i = 1; i < data.size(); i++)
        //         if (min == max) cv::line(imagemPlot, cv::Point(i-1, HEIGHT/2.0), cv::Point(i, HEIGHT/2.0 + 31*data[i]), cv::Scalar(191,172,35), 1.5, CV_AA, 0);
        //         else            cv::line(imagemPlot, cv::Point(i-1, Map(data[i-1], min, max, HEIGHT, 0)), cv::Point(i, HEIGHT/2.0 + 31*data[i]), cv::Scalar(191,172,35), 1.5, CV_AA, 0);

        //     cv::putText(imagemPlot, std::to_string(data.back()), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(68,225,242), 1, CV_AA);

        //     cv::imshow(windowName, imagemPlot);
            
        //     char c = (char) cvWaitKey(1);
        //     if(c == 27) Quit_Handler(c); // ESC key
        // }


    }

    catch (int error)
    {
        fprintf(stderr, "%s lançou uma exceção: %i \n", argv[0], error);
        return error;
    }

    return EXIT_SUCCESS;
}