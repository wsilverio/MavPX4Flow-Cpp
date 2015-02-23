all: mavlink_control

mavlink_control: mavlink_control.cpp
#	g++ -I mavlink/include/mavlink/v1.0 mavlink_control.cpp serial_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread
	g++ -I mavlink/include/mavlink/v1.0 serial_port.cpp px4flow_interface.cpp main.cpp -o mavpx4flow.run -lpthread `pkg-config --cflags --libs opencv` -std=c++11

clean:
	 rm -rf *.o *.run

run-teste:
	./mavpx4flow.run -d /dev/ttyACM0 -b 57600