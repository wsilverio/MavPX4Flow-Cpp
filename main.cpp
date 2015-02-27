/*
    Baseado em https://github.com/mavlink/c_uart_interface_example
*/

#include <stdio.h>  // printf
#include <cstdlib>  // EXIT_FAILURE
#include <signal.h> // signal

#include <common/mavlink.h>     // Protocolo Mavlink
#include "serial_port.h"        // Serial_Port
#include "px4flow_interface.h"  // PX4Flow_Interface

#include <iostream>     // std::cout
#include <vector>       // std::vector<>    
#include <string>       // std::to_string
#include <algorithm>    // std::min_element, std::max_element
#include <math.h>       // fabs

#include <opencv2/opencv.hpp>   // OpenCV
#include <X11/Xlib.h>           // DefaultScreenOfDisplay

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
void Argparse(int argc, char **argv, char *&uart_name, int &baudrate){

    const char *help_msg = "\nUso:\n$ ./mavpx4flow.run -d <dispositivo> -b <baudrate>\n";

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

    printf("\nDevice: %s\nBaud rate: %d\n\n", uart_name, baudrate);
    return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// Esta função é chamada quando o usuário pressiona ^C (Ctrl-C) sobre o terminal ou ESC sobre a janela
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
//   Map - http://openframeworks.cc/documentation/math/ofMath.html#!show_ofMap
// ------------------------------------------------------------------------------
// Re-maps a number from one range to another.
float Map(float value, float inputMin, float inputMax, float outputMin, float outputMax){
    if (fabs(inputMin - inputMax) < FLT_EPSILON)
        return outputMin;
    else
        return outputMin + (outputMax - outputMin) * ((value - inputMin) / (inputMax - inputMin));
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv){

    printf("\n### MavPX4Flow ###\n");

    // Padrão
    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 57600;

    // Tratamento da linha de comando
    Argparse(argc, argv, uart_name, baudrate);

    // Relacionadas a, por ex.:
    // OPTICAL_FLOW, flow_x
    int msgID, msgFieldName;

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

        #define TAM 8
        const char *names[TAM] = 
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

        printf("Escolha a mensagem Mavlink:\n\n");

        for (int i = 0; i < TAM; ++i){
            printf("{%d} %s\n", i, names[i]);
        }

        printf("Mensagem: ");

        msgID = -1;
        scanf("%d", &msgID);

        // Verifica o ID selecionado pelo usuário
        switch(msgID){
            case HEARTBEAT:
            {
                msgID = MAVLINK_MSG_ID_HEARTBEAT;
                windowName = "HEARTBEAT: ";

                #define TAM 6
                const char *fields[TAM] = {"type", "autopilot", "base_mode", "custom_mode", "system_status", "mavlink_version"};

                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                windowName += fields[msgFieldName];

            }
            break;

            case DATA_TRANSMISSION_HANDSHAKE:
            {
                msgID = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
                windowName = "DATA_TRANSMISSION_HANDSHAKE: ";

                #define TAM 7
                const char *fields[TAM] = {"type", "size", "width", "height", "packets", "payload", "jpg_quality"};

                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                windowName += fields[msgFieldName];

            }
            break;

            case ENCAPSULATED_DATA:
            {
                msgID = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
                windowName = "ENCAPSULATED_DATA: ";

                // #define TAM 2
                // const char *fields[TAM] = {"seqnr", "data"};
                
                // printf("\nEscolha o Field Name da mensagem:\n\n");

                // for (int i = 0; i < TAM; ++i){
                //     printf("{%d} %s\n", i, fields[i]);
                // }

                // printf("Field Name: ");

                // msgFieldName = -1;
                // scanf("%d", &msgFieldName);

                // if (msgFieldName < 0 || msgFieldName >= TAM){
                //     printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                //     throw EXIT_FAILURE;
                // }

                // windowName += fields[msgFieldName];
                windowName += "image";

            }
            break;

            case OPTICAL_FLOW:
            {
                msgID = MAVLINK_MSG_ID_OPTICAL_FLOW;
                windowName = "OPTICAL_FLOW: ";

                #define TAM 8
                const char *fields[TAM] = {"time_usec", "sensor_id", "flow_x", "flow_y", "flow_comp_m_x", "flow_comp_m_y", "quality", "ground_distance"};

                printf("\nEscolha o Field Name da mensagem:\n\n");

                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM || msgFieldName == 0){ // time_usec desabilitado
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

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
                
                printf("\nEscolha o Field Name da mensagem:\n\n");

                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }

                printf("Field Name: ");

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM || msgFieldName == 0){ // time_usec desabilitado
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                windowName += fields[msgFieldName];

            }
            break;

            case DEBUG_VECT:
            {
                msgID = MAVLINK_MSG_ID_DEBUG_VECT;
                windowName = "DEBUG_VECT: ";

                #define TAM 5
                const char *fields[TAM] = {"name", "time_usec", "x", "y", "z"};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");                
                
                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM || msgFieldName == 1){ // time_usec desabilitado
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                windowName += fields[msgFieldName];

            }
            break;

            case NAMED_VALUE_FLOAT:
            {
                msgID = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
                windowName = "NAMED_VALUE_FLOAT: ";

                #define TAM 3
                const char *fields[TAM] = {"time_boot_ms", "name", "value"};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");  

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

                windowName += fields[msgFieldName];

            }
            break;

            case NAMED_VALUE_INT:
            {
                msgID = MAVLINK_MSG_ID_NAMED_VALUE_INT;
                windowName = "NAMED_VALUE_INT: ";

                #define TAM 3
                const char *fields[TAM] = {"time_boot_ms", "name", "value"};
                
                printf("\nEscolha o Field Name da mensagem:\n\n");
                
                for (int i = 0; i < TAM; ++i){
                    printf("{%d} %s\n", i, fields[i]);
                }                

                printf("Field Name: ");                 

                msgFieldName = -1;
                scanf("%d", &msgFieldName);

                if (msgFieldName < 0 || msgFieldName >= TAM){
                    printf("\n*** Erro: Fiel Name desconhecido ou ainda não implementado ***\n\n");
                    throw EXIT_FAILURE;
                }

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

        // Vetor que contém o histórico de dados
        std::vector<float> data;
        // Buffer da imagem a ser plotada
        cv::Mat imagemPlot;
        
        // Parâmetros da imagem
        #define WIDTH 1280
        #define HEIGHT 720
        
        // Parâmetros do monitor do usuário
        Display *display = XOpenDisplay(NULL);
        Screen  *screen = DefaultScreenOfDisplay(display);
        const int screenWidth  = screen->width;
        const int screenHeight = screen->height;

        // Janela que conterá a imagem
        cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
        // Move a janela para o centro da tela
        cv::moveWindow(windowName, (screenWidth-WIDTH)/2, (screenHeight-HEIGHT)/2);

        // Gerenciador serial
        Serial_Port serial_port(uart_name, baudrate);
        // Controlador de saída. Ver função Quit_Handler()
        serial_port_quit = &serial_port;

        // Gerenciador Mavlink -> Imagem// Gerenciador Mavlink -> Datos
        PX4Flow_Interface px4flow(&serial_port, msgID, msgFieldName, &data, &imagemPlot);
        // Controlador de saída. Ver função Quit_Handler()
        px4flow_interface_quit = &px4flow;

        // Associa ^C (ctrl+c) à função Quit_Handler()
        signal(SIGINT, Quit_Handler);

        // Abre a porta e inicializa a comunicação serial
        serial_port.start();
        // Inicializa a comunicação Mavlink (thread)
        px4flow.start();

        // Main Loop
        for(;;){

            // Caso o usuário selecione para receber a imagem da sensor
            // o próprio gerenciador Mavlink a insere em imagemPlot
            if (msgID != MAVLINK_MSG_ID_ENCAPSULATED_DATA){                
    
                // Imagem vazia, com background
                imagemPlot = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(54,54,54));
    
                // Verifica se o buffer está vazio
                if(data.size() < 2){ // 2 porque o programa acessa data[i-1]
                    std::cout   << "\nAguardando o(s) parâmetro(s) solicitado(o) pelo usuário.\n"
                                << "Pressione ctrl+c para sair.\n";
                
                    // Fica em loop até o recebimento dos parâmetros (min 2)
                    while(data.size() < 2);
                }
    
                // Últimos WIDTH elementos do buffer
                if (data.size() > WIDTH)
                    data.erase(data.begin(), data.end() - WIDTH);
                    // data.erase(data.begin());
    
                // Armazena os extremos do buffer
                float min = *min_element(data.begin(), data.end());
                float max = *max_element(data.begin(), data.end());
    
                #define MARGEM 5
                // Varre o buffer e adiciona os pontos (escalados) na imagem
                for(register int i = 1; i < MIN(data.size(), WIDTH); i++){
                    if (fabs(max-min) < FLT_EPSILON) cv::line(imagemPlot, cv::Point(i-1, HEIGHT/2.0), cv::Point(i, HEIGHT/2.0), cv::Scalar(191,172,35), 1.5, CV_AA, 0);
                    else cv::line(imagemPlot, cv::Point(i-1, Map(data[i-1], min, max, HEIGHT, 0)), cv::Point(i, Map(data[i], min, max, HEIGHT-MARGEM, MARGEM)), cv::Scalar(191,172,35), 1.5, CV_AA, 0);
                }

                // Adiciona o último valor em modo texto na imagem
                cv::putText(imagemPlot, std::to_string(data.back()), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(68,225,242), 1, CV_AA);
            }

            // Imagem válida?
            while(not imagemPlot.rows && not imagemPlot.cols);

            // Exibe a imagem
            cv::imshow(windowName, imagemPlot);
            
            // Fecha o programa caso a tecla ESC seja pressionada sobre a janela
            char c = (char) cvWaitKey(1);
            if(c == 27) Quit_Handler(c); // ESC key
        }

    }

    catch (int error)
    {
        fprintf(stderr, "%s lançou uma exceção: %i \n", argv[0], error);
        return error;
    }

    return EXIT_SUCCESS;
}