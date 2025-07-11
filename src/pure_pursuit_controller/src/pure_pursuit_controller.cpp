#include "pure_pursuit_controller/pure_pursuit_controller.hpp"

namespace pure_pursuit_controller
{

PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller"),
  pose_received_(false),
  path_received_(false),
  reverse_mode_(false),
  autonomous_control_enabled_(false),
  current_waypoint_idx_(0)
{
    // Declare parameters
    this->declare_parameter("min_lookahead_distance", 1.0);
    this->declare_parameter("max_lookahead_distance", 5.0);
    this->declare_parameter("lookahead_ratio", 2.0);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 1.0);
    this->declare_parameter("goal_tolerance", 0.5);
    this->declare_parameter("wheel_base", 2.5);
    this->declare_parameter("control_frequency", 10.0);
    
    // Get parameters
    min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
    max_lookahead_distance_ = this->get_parameter("max_lookahead_distance").as_double();
    lookahead_ratio_ = this->get_parameter("lookahead_ratio").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    wheel_base_ = this->get_parameter("wheel_base").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create subscribers
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/pcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PurePursuitController::poseCallback, this, std::placeholders::_1));
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/follow_path", 10,
        std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
    
    reverse_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/reverse_mode", 10,
        std::bind(&PurePursuitController::reverseCallback, this, std::placeholders::_1));
    
    autonomous_control_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/enable_autonomous_control", 10,
        std::bind(&PurePursuitController::autonomousControlCallback, this, std::placeholders::_1));
    
    // Create publishers
    cmd_vel_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/current_path", 10);
    
    // Create control timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&PurePursuitController::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized");
}

void PurePursuitController::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Convert PoseWithCovarianceStamped to PoseStamped
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    pose_received_ = true;
}

void PurePursuitController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    loadWaypointsFromPath(*msg);
    path_received_ = true;
    current_waypoint_idx_ = 0;
    
    // Publish the current path for visualization
    path_pub_->publish(*msg);
    
    RCLCPP_INFO(this->get_logger(), "Received path with %zu waypoints", waypoints_.size());
}

void PurePursuitController::reverseCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data != reverse_mode_)
    {
        reverse_mode_ = msg->data;
        if (reverse_mode_)
        {
            reverseWaypoints();
            RCLCPP_INFO(this->get_logger(), "Switched to reverse mode");
        }
        else
        {
            waypoints_ = original_waypoints_;
            RCLCPP_INFO(this->get_logger(), "Switched to forward mode");
        }
        current_waypoint_idx_ = 0;
    }
}

void PurePursuitController::autonomousControlCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
        if (!autonomous_control_enabled_)
        {
            autonomous_control_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Autonomous control enabled");
        }
        else
        {
            autonomous_control_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "Autonomous control disabled");
            // Publish zero command when disabling
            ackermann_msgs::msg::AckermannDriveStamped cmd_vel;
            cmd_vel.header.stamp = this->get_clock()->now();
            cmd_vel.header.frame_id = "base_link";
            cmd_vel.drive.speed = 0.0;
            cmd_vel.drive.steering_angle = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
        }
}

void PurePursuitController::controlLoop()
{
    if (!pose_received_ || !path_received_ || waypoints_.empty() || !autonomous_control_enabled_)
    {
        return;
    }
    
    // Find closest waypoint
    int closest_idx = findClosestWaypoint();
    if (closest_idx == -1)
    {
        return;
    }
    
    // Check if we reached the goal
    double distance_to_goal = calculateDistance(
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        waypoints_.back().x, waypoints_.back().y);
    
    if (distance_to_goal < goal_tolerance_)
    {
        // Stop the robot
        ackermann_msgs::msg::AckermannDriveStamped cmd_vel;
        cmd_vel.header.stamp = this->get_clock()->now();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.drive.speed = 0.0;
        cmd_vel.drive.steering_angle = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
    }
    
    // Find lookahead waypoint
    int lookahead_idx = findLookaheadWaypoint(closest_idx);
    if (lookahead_idx == -1)
    {
        return;
    }
    
    // Calculate control command
    ackermann_msgs::msg::AckermannDriveStamped cmd_vel = calculateControlCommand(waypoints_[lookahead_idx]);
    
    // Publish command
    cmd_vel_pub_->publish(cmd_vel);
}

int PurePursuitController::findClosestWaypoint()
{
    if (waypoints_.empty())
    {
        return -1;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        double distance = calculateDistance(
            current_pose_.pose.position.x, current_pose_.pose.position.y,
            waypoints_[i].x, waypoints_[i].y);
        
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    return closest_idx;
}

int PurePursuitController::findLookaheadWaypoint(int closest_idx)
{
    if (waypoints_.empty())
    {
        return -1;
    }
    
    // Adaptive lookahead distance based on velocity
    double velocity = std::sqrt(
        std::pow(current_pose_.pose.position.x - waypoints_[closest_idx].x, 2) +
        std::pow(current_pose_.pose.position.y - waypoints_[closest_idx].y, 2)) * lookahead_ratio_;
    
    double adaptive_lookahead = std::min(std::max(velocity, min_lookahead_distance_), max_lookahead_distance_);
    
    // Find the waypoint at lookahead distance
    for (size_t i = closest_idx; i < waypoints_.size(); ++i)
    {
        double distance = calculateDistance(
            current_pose_.pose.position.x, current_pose_.pose.position.y,
            waypoints_[i].x, waypoints_[i].y);
        
        if (distance >= adaptive_lookahead)
        {
            return i;
        }
    }
    
    // If no waypoint found at lookahead distance, return the last waypoint
    return waypoints_.size() - 1;
}

ackermann_msgs::msg::AckermannDriveStamped PurePursuitController::calculateControlCommand(const Waypoint& lookahead_point)
{
    ackermann_msgs::msg::AckermannDriveStamped cmd_vel;
    cmd_vel.header.stamp = this->get_clock()->now();
    cmd_vel.header.frame_id = "base_link";
    
    // Get current yaw from quaternion
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Calculate the vector from current position to lookahead point
    double dx = lookahead_point.x - current_pose_.pose.position.x;
    double dy = lookahead_point.y - current_pose_.pose.position.y;
    
    // Transform to robot frame
    double alpha = std::atan2(dy, dx) - yaw;
    alpha = normalizeAngle(alpha);
    
    // Calculate lookahead distance
    double lookahead_dist = std::sqrt(dx * dx + dy * dy);
    
    // Pure pursuit steering calculation
    double curvature = 2.0 * std::sin(alpha) / lookahead_dist;
    double steering_angle = std::atan(wheel_base_ * curvature);
    
    // Calculate velocities
    double target_velocity = lookahead_point.velocity;
    if (reverse_mode_)
    {
        target_velocity = -target_velocity;
        steering_angle = -steering_angle; // Reverse steering for reverse mode
    }
    
    cmd_vel.drive.speed = std::min(std::max(target_velocity, -max_linear_velocity_), max_linear_velocity_);
    cmd_vel.drive.steering_angle = std::min(std::max(steering_angle, -max_angular_velocity_), max_angular_velocity_);
    cmd_vel.drive.steering_angle_velocity = 0.0;
    cmd_vel.drive.acceleration = 0.0;
    cmd_vel.drive.jerk = 0.0;
    
    return cmd_vel;
}

double PurePursuitController::calculateDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double PurePursuitController::normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

void PurePursuitController::loadWaypointsFromPath(const nav_msgs::msg::Path& path)
{
    waypoints_.clear();
    original_waypoints_.clear();
    
    for (const auto& pose_stamped : path.poses)
    {
        Waypoint wp;
        wp.x = pose_stamped.pose.position.x;
        wp.y = pose_stamped.pose.position.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        wp.yaw = yaw;
        
        // Set default velocity (can be extended to read from path if available)
        wp.velocity = max_linear_velocity_ * 0.5; // Default to half max velocity
        
        waypoints_.push_back(wp);
        original_waypoints_.push_back(wp);
    }
}

void PurePursuitController::reverseWaypoints()
{
    waypoints_ = original_waypoints_;
    std::reverse(waypoints_.begin(), waypoints_.end());
    
    // Adjust yaw angles for reverse direction
    for (auto& wp : waypoints_)
    {
        wp.yaw = normalizeAngle(wp.yaw + M_PI);
    }
}

} // namespace pure_pursuit_controller 
