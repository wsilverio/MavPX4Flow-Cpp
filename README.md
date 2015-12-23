### MavPX4Flow-Cpp

**MavPX4Flow** - Comunicador do módulo [PX4Flow](https://pixhawk.org/modules/px4flow) sobre o protocolo [Mavlink](http://qgroundcontrol.org/mavlink/).

Clone e inicialização:
```
$ git clone --recursive https://github.com/wsilverio/MavPX4Flow-Cpp.git
```
Se desejar, altere o tamanho da janela, no arquivo [main.cpp](https://github.com/wsilverio/MavPX4Flow-Cpp/blob/master/main.cpp#L511):
```cpp
// Parâmetros da imagem
WIDTH = 1200;
HEIGHT = 600;
```

O usuário deve fazer parte do grupo `dialout`
```
$ sudo usermod -a -G dialout <user> #necessário reiniciar a sessão
```

Uso:
```
$ make
$ ./mavpx4flow.run -d <dispositivo> -b <baudrate> [--debug]
```
Exemplo:
```
$ ./mavpx4flow.run -d /dev/ttyACM0 -b 115200
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
