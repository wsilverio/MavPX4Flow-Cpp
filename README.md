### MavPX4Flow-Cpp

**MavPX4Flow** - Comunicador do módulo [PX4Flow](https://pixhawk.org/modules/px4flow) sobre o protocolo [Mavlink](http://qgroundcontrol.org/mavlink/).

Uso:
```
$ make
$ ./mavpx4flow.run -d <dispositivo> -b <baudrate>
```
Exemplo:
```
$ ./mavpx4flow.run -d /dev/ttyACM0 -b 57600
```

Ou altere o gatilho **run-teste** do arquivo [makefile](.makefile) e execute:
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