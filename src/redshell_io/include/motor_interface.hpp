#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "redshell/redshell_messages.h"

class MotorInterface : public rclcpp::Node
{
public:
	MotorInterface();
	~MotorInterface();
	
private:
	void run();
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void timout_timer_callback();
	void write_motor_command(float throttle, float yaw_rate);
	void configure_serial_port();
	int scale_motor_command(int command) const;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
	rclcpp::TimerBase::SharedPtr _command_timout_timer;
	rclcpp::TimerBase::SharedPtr _run_timer;
	rclcpp::Time _last_command_timestamp;
	
	int _serial_port;
	char _command_buffer[REDSHELL_MESSAGE_SIZE];
	uint8_t _command_index{0};
};