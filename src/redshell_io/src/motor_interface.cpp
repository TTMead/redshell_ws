#include "motor_interface.hpp"

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> 		// Contains file controls like O_RDWR
#include <errno.h> 		// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()

#define CMDVEL_TIMEOUT_S 0.5	// The amount of time to wait for a cmd_vel msg before stopping motors
#define CMDVEL_MIN_PERIOD_S 0.1		// The amount of time to ignore cmd_vel msg after one has been received
#define MOTOR_DEADZONE 9.0				// The threshold of percentage PWM that causes the motor to start moving

MotorInterface::MotorInterface() : Node("redshell_motor_interface")
{
	// Subscribe to /cmd_vel topic
	_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
		"/cmd_vel", 10, 
		std::bind(&MotorInterface::cmd_vel_callback, this, std::placeholders::_1)
	);

	// Open serial port connection to the motor interface board
	this->_serial_port = open(this->SERIAL_PORT_LOCATION, O_RDWR);
	if (this->_serial_port < 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i while attempting to open serial connection '%s': \"%s\"\n", errno, this->SERIAL_PORT_LOCATION, strerror(errno));
	}
	configure_serial_port();

	// Initialise command timeout timer
	_last_command_timestamp = rclcpp::Node::now();
	_command_timout_timer = this->create_wall_timer(
    	std::chrono::milliseconds(500), 
		std::bind(&MotorInterface::timout_timer_callback, this)
	);
}

MotorInterface::~MotorInterface() {
	close(this->_serial_port);
}

void
MotorInterface::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	if (msg->linear.y != 0 ||
		msg->linear.z != 0 ||
		msg->angular.x != 0 ||
		msg->angular.y != 0) 
	{
		RCLCPP_WARN(this->get_logger(), "Motor interface module has received a /cmd_vel message that cannot be translated into motor commands.");
	}

	// If we received /cmd_vel at a rate too high, ignore the message (to prevent flooding the UART bus)
	rclcpp::Time current_time = rclcpp::Node::now();
	if ((current_time - _last_command_timestamp) < rclcpp::Duration::from_seconds(CMDVEL_MIN_PERIOD_S)) {
		return;
	}

	write_motor_command(msg->linear.x, msg->angular.z);

	// Reset the motor command time
	_last_command_timestamp = rclcpp::Node::now();
}


void
MotorInterface::timout_timer_callback() {
	rclcpp::Time current_time = rclcpp::Node::now();

	// Stop the motors if we havent received a command for over 1 second
	if ((current_time - _last_command_timestamp) > rclcpp::Duration::from_seconds(CMDVEL_TIMEOUT_S)) {
		write_motor_command(0, 0);
	}
}

int
MotorInterface::scale_motor_command(int command)
{
	// If below deadzone, don't run
	if (command < MOTOR_DEADZONE) {
		return 0;
	}

	// Else return scaled value
	float command_f = static_cast<float>(command);
	command_f = ((command_f * (99.0 - MOTOR_DEADZONE)) / 99.0) + MOTOR_DEADZONE;
	return static_cast<int>(command_f);
}


void
MotorInterface::write_motor_command(float throttle, float yaw_rate){
	// Speed values are within the range of [-1 to 1]
	float left_speed = (throttle - yaw_rate)/2;    
	float right_speed = (throttle + yaw_rate)/2;

	// Motor speed commands are within the range of [00 to 99]
	int left_cmd = std::ceil(abs(left_speed)*(99.0));
	int right_cmd = std::ceil(abs(right_speed)*(99.0));

	// Scale the motor speeds out of the deadzone
	left_cmd = scale_motor_command(left_cmd);
	right_cmd = scale_motor_command(right_cmd);

	char left_dir;
	char right_dir;

	if (left_speed >= 0) {
		left_dir = 'F';
	} else {
		left_dir = 'B';
	}
	
	if(right_speed >= 0) {
		right_dir = 'F';
	} else {
		right_dir = 'B';
	}

	char left_cmd_str[3];
	char right_cmd_str[3];
	snprintf(left_cmd_str, 3, "%02d", left_cmd);
	snprintf(right_cmd_str, 3, "%02d", right_cmd);

	char motor_command[] = {
		'=',
		'L',
		left_dir,
		left_cmd_str[0],
		left_cmd_str[1],
		'=',
		'R',
		right_dir,
		right_cmd_str[0],
		right_cmd_str[1],
		'\r'
	};

	RCLCPP_INFO(this->get_logger(), "Sending motor command '%s'.", motor_command);

	write(this->_serial_port, motor_command, sizeof(motor_command));
}

void
MotorInterface::configure_serial_port()
{
	/* Critical serial specifications:
	 * Baud: 19200
	 * Data Size: 8 bits
	 * Parity: None
	 * Stop Bits: 1 */
	
	struct termios tty;

	// Read in existing settings, and handle any error
	if(tcgetattr(this->_serial_port, &tty) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return;
	}

	// Control mode flags
	tty.c_cflag &= ~PARENB; 		// Clear parity bit, disabling parity
	tty.c_cflag &= ~CSTOPB; 		// Clear stop field, only one stop bit used in communication
	tty.c_cflag &= ~CSIZE; 			// Clear all bits that set the data size 
	tty.c_cflag |= CS8; 			// 8 bits per data segment
	tty.c_cflag &= ~CRTSCTS; 		// Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; 	// Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// Local mode flags
	tty.c_lflag &= ~ICANON;		// Disable canonical mode
	tty.c_lflag &= ~ECHO; 		// Disable echo
	tty.c_lflag &= ~ECHOE; 		// Disable erasure
	tty.c_lflag &= ~ECHONL;		// Disable new-line echo
	tty.c_lflag &= ~ISIG; 		// Disable interpretation of INTR, QUIT and SUSP

	// Input mode flags
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); 							// Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 	// Disable any special handling of received bytes

	// Output mode flags
	tty.c_oflag &= ~OPOST; 		// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; 		// Prevent conversion of newline to carriage return/line feed

	// Timeout configuration
	// Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN] = 0;

	// Baudrate configuration
	cfsetispeed(&tty, B19200);
	cfsetospeed(&tty, B19200);

	// Save tty settings
	if (tcsetattr(this->_serial_port, TCSANOW, &tty) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return;
	}
}


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorInterface>());
	rclcpp::shutdown();
	return 0;
}