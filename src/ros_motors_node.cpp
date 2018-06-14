/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <ros/ros.h>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <thunder_trekking/Motor.h>
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
		void motors_callback(const thunder_trekking::Motor::ConstPtr& motor_msg);
	
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub;
	
		void init_serial();
		void drive_serial();
		int servo;
		int speed;
	
		int max_speed = INIT_VMAX;
		int servo = 0, speed = 0, cur_speed = 80;
	
		int fd;
		const bool use_serial = true;
		const int baudrate = 9600;
		
		void set_max_speed(int new_max_speed);
		void reset_max_speed();
}

Motor::Motor() : nh_(){
	motors_sub = nh_.subscribe("motor", 1, &Motor::motors_callback);

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

void Motor::motors_callback(const thunder_trekking::Motor::ConstPtr& motor_msg){
	if (motor_msg -> header.stamp == ros::Time(0))
    {
        ROS_INFO("Invalid motor message.");
        return;
    }
	servo = motor_msg -> servo;
	speed = motor_msg -> motor;
}

void Motor::spin(){

	uint8_t max_speed_par;
	nh_.param("max_speed", max_speed_par, INIT_VMAX);
	
	while(ros::ok()){
		ros::spinOnce();
		
		if(nh_.getParam("max_speed", max_speed_par) && max_speed_par != max_speed){
			set_max_speed(max_speed_par);
		}

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