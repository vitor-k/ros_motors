#include <ros/ros.h>
#include <iostream>
#include <fcntl.h>
//#include "trekking_node/serial.h"

#define INIT_VMAX 255

#define abs(x) ((x) > 0 ? (x) : -(x))

//static int max_speed = INIT_VMAX;
//static int servo = 0, speed = 0, cur_speed = 80;

//static const int baudrate = 9600;
//static const uint32_t sleep_time = 10;

class Motor
{
	public:
		Motor();
		void spin();
	
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub;
	
		void init_serial();
		void drive_serial();

		void motors_callback();
	
		int max_speed = INIT_VMAX;
		int servo = 0, speed = 0, cur_speed = 80;
	
		int fd;
		const bool use_serial = true;
		const int baudrate = 9600;
		
		void set_max_speed(int new_max_speed);
		void reset_max_speed();
}

Motor::Motor() : nh_(){


}

void Motor::init_serial(){
	fd = serial_open("/dev/arduino", &baudrate, O_WRONLY);
	while (fd == -1){
		fd = serial_open("/dev/arduino", &baudrate, O_WRONLY);
		ros::Duration(0.01).sleep();
	}
}

void Motor::drive_serial(){
	init_serial();
	uint8_t message[6] = { 255, servo < 0, abs(servo), speed < 0, abs(speed), 254 };
	if (write(fd, message, sizeof(message)) == -1){
		std::cerr << "write: " << errno_string() << std::endl;
		close(fd);
		init_serial();
    }
}

void Motor::set_max_speed(int new_max_speed){
	max_speed = new_max_speed > 255 ? 255 : new_max_speed;
}

void Motor::reset_max_speed(){
	max_speed = INIT_VMAX;
}

void Motor::motors_callback(){

}

void Motor::spin()
{
	motors_sub = nh_.subscribe("motor", 1, &Motor::motors_callback);

	uint8_t max_speed_par;
	nh_.param("max_speed", max_speed_par);
	
	while(ros::ok()){
		ros::spinOnce();
		


		if(use_serial){
			drive_serial();
		}
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_subscriber");
  
  Motor motor;
  motor.spin();
  
  return 0;
}