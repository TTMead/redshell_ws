#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

class MotorInterface : public rclcpp::Node
{
	public:
		MotorInterface();
		~MotorInterface();

		const char* SERIAL_PORT_LOCATION = "/dev/ttyUSB0";

	private:
		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
		void timout_timer_callback();
		void write_motor_command(float throttle, float yaw_rate);
		void configure_serial_port();
		int scale_motor_command(int command);

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
		rclcpp::TimerBase::SharedPtr _command_timout_timer;
		rclcpp::Time _last_command_timestamp;
		int _serial_port;
};