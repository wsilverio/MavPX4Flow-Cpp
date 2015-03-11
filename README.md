### MavPX4Flow-Cpp

**MavPX4Flow** - Comunicador do módulo [PX4Flow](https://pixhawk.org/modules/px4flow) sobre o protocolo [Mavlink](http://qgroundcontrol.org/mavlink/).

Clone e inicialização:
```
$ git clone https://github.com/wsilverio/MavPX4Flow-Cpp.git
$ cd MavPX4Flow-Cpp
$ git submodule update --init
```
Se desejar, altere o tamanho da tela, no arquivo [main.cpp](https://github.com/wsilverio/MavPX4Flow-Cpp/blob/master/main.cpp):
```cpp
// Parâmetros da imagem
#define WIDTH 1280
#define HEIGHT 720
```

Uso:
```
$ make
$ ./mavpx4flow.run -d <dispositivo> -b <baudrate> [--debug]
```
Exemplo:
```
$ ./mavpx4flow.run -d /dev/ttyACM0 -b 57600
```

Ou altere o gatilho **run-teste** do arquivo [makefile](https://github.com/wsilverio/MavPX4Flow-Cpp/blob/master/makefile) e execute:
```
$ make
$ make run-teste
```

Para sair, pressione **ESC** sobre a janela ou **ctrl+c** sobre o terminal.

**Atenção!** Este código é experimental.  
Testado com:
> [PX4FLOW v1.3 (Hardware)](https://pixhawk.org/modules/px4flow)  
> [OpenCV for Linux v2.4.10](http://opencv.org/)

Baseado em [mavlink/c_uart_interface_example](https://github.com/mavlink/c_uart_interface_example).
