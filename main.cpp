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
void argparse(int argc, char **argv, char *&uart_name, int &baudrate, int &msgID){

    const char *help_msg = "Uso:\n$ ./mavpx4flow.run -d <dispositivo> -b <baudrate> -id <msgid>\n";

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

        // Mavlink MSG ID
        if (!strcmp(argv[i], "-id")){
            if (argc > i + 1) {
                msgID = atoi(argv[i + 1]);
            } else {
                printf("%s\n", help_msg);
                throw EXIT_FAILURE;
            }
        }
    }

    printf("Device: %s\nBaud rate: %d\nMSG ID: #%d\n", uart_name, baudrate, msgID);
    return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// Esta função é chamada quando o usuário pressiona ^C (Ctrl-C)
void quit_handler(int sig){

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

int main(int argc, char **argv){

    // Padrão
    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 57600;
    int msgID = MAVLINK_MSG_ID_DEBUG_VECT;
    // int msgID = MAVLINK_MSG_ID_OPTICAL_FLOW;
    // int msgID = MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE;
    // int msgID = MAVLINK_MSG_ID_ENCAPSULATED_DATA;

    try
    {
        std::vector<float> data;

        // Tratamento da linha de comando
        argparse(argc, argv, uart_name, baudrate, msgID);

        Serial_Port serial_port(uart_name, baudrate);
        serial_port_quit = &serial_port; // função quit_handler()

        PX4Flow_Interface px4flow(&serial_port, msgID, &data);
        px4flow_interface_quit = &px4flow; // função quit_handler()

        // Associa ^C à função quit_handler()
        signal(SIGINT, quit_handler);

        // Abre a porta e inicializa a comunicação serial
        serial_port.start();
        // Inicializa a comunicação Mavlink (thread)
        px4flow.start();

        cv::Mat imagem;
        cv::namedWindow("eixo: z", CV_WINDOW_AUTOSIZE);

        for(;;){

            imagem = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(54,54,54));

            if (data.size() > 1280)
                data.erase(data.begin());
            

                for(int i = 1; i < data.size(); i++)
                    cv::line(imagem, cv::Point(i-1, 720/2.0 + 31*data[i-1]), cv::Point(i, 720/2.0 + 31*data[i]), cv::Scalar(191,172,35), 1.5, CV_AA, 0);

                cv::imshow("eixo: z", imagem);
                cv::waitKey(1);

        }


    }

    catch (int error)
    {
        fprintf(stderr, "%s lançou uma exceção: %i \n", argv[0], error);
        return error;
    }

    return EXIT_SUCCESS;
}