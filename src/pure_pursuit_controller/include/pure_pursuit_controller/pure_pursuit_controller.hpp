#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <cmath>
#include <algorithm>

namespace pure_pursuit_controller
{

struct Waypoint
{
    double x;
    double y;
    double yaw;
    double velocity;
};

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();
    ~PurePursuitController() = default;

private:
    // Callback functions
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void reverseCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void autonomousControlCallback(const std_msgs::msg::Int8::SharedPtr msg);
    
    // Pure pursuit algorithm functions
    void controlLoop();
    int findClosestWaypoint();
    int findLookaheadWaypoint(int closest_idx);
    ackermann_msgs::msg::AckermannDriveStamped calculateControlCommand(const Waypoint& lookahead_point);
    double calculateDistance(double x1, double y1, double x2, double y2);
    double normalizeAngle(double angle);
    
    // Waypoint management
    void loadWaypointsFromPath(const nav_msgs::msg::Path& path);
    void reverseWaypoints();
    
    // Publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr autonomous_control_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Current state
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_;
    bool path_received_;
    bool reverse_mode_;
    bool autonomous_control_enabled_;
    
    // Waypoints
    std::vector<Waypoint> waypoints_;
    std::vector<Waypoint> original_waypoints_;
    int current_waypoint_idx_;
    
    // Pure pursuit parameters
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double lookahead_ratio_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    double wheel_base_;
    double control_frequency_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace pure_pursuit_controller

#endif // PURE_PURSUIT_CONTROLLER_HPP 