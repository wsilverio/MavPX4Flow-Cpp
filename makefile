all: mavlink_control

mavlink_control: main.cpp
	g++ -I mavlink/include/mavlink/v1.0 serial_port.cpp px4flow_interface.cpp main.cpp -o mavpx4flow.run -lpthread -lX11 `pkg-config --cflags --libs opencv` -std=c++11

clean:
	 rm -rf *.o *.run

run-teste:
	./mavpx4flow.run -d /dev/ttyACM0 -b 115200

run-debug:
	./mavpx4flow.run -d /dev/ttyACM0 -b 115200 --debug