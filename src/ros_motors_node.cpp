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

#define INIT_VMAX 255

#define abs(x) ((x) > 0 ? (x) : -(x))

#define _DEFAULT_SOURCE
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#define OPTIONS "b:d:h"
#define nitens(list) 1 [&list] - list
#define try(cmd, msg) do { if ((cmd) == -1) { std::perror(msg); return -1; } } while (0)  // NOLINT(whitespace/braces)

// Ver man 3 termios para as flags para terminais

/*
 * Possíveis constantes de baud:
 * B0
 * B50
 * B75
 * B110
 * B134
 * B150
 * B200
 * B300
 * B600
 * B1200
 * B1800
 * B2400
 * B4800
 * B9600
 * B19200
 * B38400
 * B57600
 * B115200
 * B230400
 * Tem valores consecutivos (http://www.delorie.com/djgpp/doc/incs/termios.h). Uma busca binária em uma array com os
 * valores retorna o valor certo da constante.
 */
namespace Serial{
	static const unsigned int baud_list[] =
	{ 0,    50,   75,   110,  134,   150,   200,   300,    600,   1200,
	  1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
	};

	static int cmp_uints(const void *cmp1, const void *cmp2)
	{
	  const unsigned int *n1 = reinterpret_cast<const unsigned int *>(cmp1);
	  const unsigned int *n2 = reinterpret_cast<const unsigned int *>(cmp2);
	  return *n1 - *n2;
	}

	inline static int baud_bsearch(const int *baud)
	{
	  unsigned int *found = NULL;

	  found = (unsigned int *)bsearch(baud, &baud_list, nitens(baud_list), sizeof(baud_list[0]), cmp_uints);
	  if (!found)
	    return -1;
	  return found - baud_list;
	}

	/***********************************************************************
	 * serial_open
	 * Abre um dispositivo serial. O driver para terminais do Linux suporta
	 * várias flags e configurações, essa função abre:
	 * -Com baud de entrada e saída iguais
	 * -Sem checagem de paridade
	 * -Sem processamento de saída ou geração de sinais
	 * -Transmissão de 8 bits
	 * -Modo canônico
	 **********************************************************************/
	int serial_open(char *dev, const int *baud, int flags)
	{
	  struct termios tty;
	  int fd, speed;

	  // Baud de entrada/saída
	  if ((speed = baud_bsearch(baud)) == -1)
	  {
	    fprintf(stderr, "serial_open - Invalid baud rate\n");
	    return -1;
	  }

	  try
	    (fd = open(dev, flags | O_NOCTTY), "serial_open - open");

	  memset(&tty, 0, sizeof(tty));
	  cfsetospeed(&tty, (speed_t)speed);
	  cfsetispeed(&tty, (speed_t)speed);

	  // Sem paridade
	  tty.c_iflag = IGNPAR;

	  // Sem processamento adicional da saída
	  // (ver manual, há flags para conversão entre convenções como NL -> CRNL automáticas)
	  tty.c_oflag = 0;

	  // 8 bits de dados, mas a especificação diz que o bit 7 é sempre 0 para NMEA
	  tty.c_cflag |= (CS8 | CLOCAL | CREAD);

	  // VMIN e VTIME setam número mínimo de caracteres para ler antes de retornar read
	  // e timeout

	  // Modo canônico
	  tty.c_lflag = ICANON;
	  tcflush(fd, TCIFLUSH);
	  try
	    (tcsetattr(fd, TCSAFLUSH, &tty), "serial_open - tcsetattr");
	  return fd;
	}  // namespace Trekking
}

class Motor
{
	public:
		Motor();
		void spin();
		void motors_callback(const thunder_trekking::Motor::ConstPtr& motor_msg);
	
	private:
		ros::NodeHandle nh_;

		ros::Subscriber motors_sub;
	
		void init_serial();
		void drive_serial();
		void drive_serial(int servo, int speed);
	
		int max_speed = INIT_VMAX;
		int servo = 0, speed = 0, cur_speed = 80;
	
		int fd;
		const bool use_serial = true;
		const int baudrate = 9600;
		
		void set_max_speed(int new_max_speed);
		void reset_max_speed();
		void stop(int servo_offset, int vel);
}

Motor::Motor() : nh_(){
	motors_sub = nh_.subscribe("motor", 1, &Motor::motors_callback);

}

void Motor::init_serial(){
	fd = Serial::serial_open("/dev/arduino", &baudrate, O_WRONLY);
	while (fd == -1){
		fd = Serial::serial_open("/dev/arduino", &baudrate, O_WRONLY);
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
void Motor::drive_serial(int c_servo, int c_speed){
	this->servo = c_servo;
	this->speed = c_speed;
	drive_serial();
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

void Motor::stop(int servo_offset, int vel)
{
	cur_speed = vel;
	while (cur_speed != 0)
	{
		drive_serial(servo_offset, cur_speed--);
		ros::Duration(0.01).sleep();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_subscriber");
  
  Motor motor;
  motor.spin();
  
  return 0;
}