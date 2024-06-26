#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class PathPlanner : public rclcpp::Node
{
    public:
        PathPlanner();

    private:
        void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void add_wave(nav_msgs::msg::OccupancyGrid& costmap, geometry_msgs::msg::Pose& robot_pose, double bearing_rad);
        nav_msgs::msg::Path generate_path(nav_msgs::msg::OccupancyGrid& costmap, geometry_msgs::msg::TransformStamped& map_to_robot);
        double distance(int32_t from_row, int32_t from_col, int32_t to_row, int32_t to_col);


        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _occupancy_grid_sub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _distance_transform_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
};