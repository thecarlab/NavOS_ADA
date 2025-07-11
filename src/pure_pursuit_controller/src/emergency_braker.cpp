#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cmath>

class EmergencyBraker : public rclcpp::Node
{
public:
    EmergencyBraker() : Node("emergency_braker"),
                       collision_time_threshold_(1.0),
                       min_safe_distance_(0.5),
                       vehicle_width_(1.0),
                       current_speed_(0.0),
                       last_odom_time_(this->get_clock()->now())
    {
        // Subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_in",rclcpp::SensorDataQoS(),
            std::bind(&EmergencyBraker::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&EmergencyBraker::odomCallback, this, std::placeholders::_1));

        // Publisher
        emergency_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/emergency_cmd", 10);

        // Parameters
        this->declare_parameter("collision_time_threshold", 1.0);
        this->declare_parameter("min_safe_distance", 0.5);
        this->declare_parameter("vehicle_width", 1.0);
        this->declare_parameter("max_check_distance", 5.0);
        this->declare_parameter("min_speed_threshold", 0.1);

        collision_time_threshold_ = this->get_parameter("collision_time_threshold").as_double();
        min_safe_distance_ = this->get_parameter("min_safe_distance").as_double();
        vehicle_width_ = this->get_parameter("vehicle_width").as_double();
        max_check_distance_ = this->get_parameter("max_check_distance").as_double();
        min_speed_threshold_ = this->get_parameter("min_speed_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Emergency Braker Node started");
        RCLCPP_INFO(this->get_logger(), "Collision time threshold: %.2f seconds", collision_time_threshold_);
        RCLCPP_INFO(this->get_logger(), "Min safe distance: %.2f meters", min_safe_distance_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Calculate current speed from odometry
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_speed_ = std::sqrt(vx * vx + vy * vy);
        
        last_odom_time_ = this->get_clock()->now();
        has_odom_data_ = true;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Check if we have recent odometry data
        auto now = this->get_clock()->now();
        if (!has_odom_data_ || 
            (now - last_odom_time_).seconds() > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "No recent odometry data, cannot perform emergency braking check");
            return;
        }

        // Skip processing if vehicle is not moving
        if (std::abs(current_speed_) < min_speed_threshold_) {
            return;
        }

        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Check for obstacles in the vehicle's path
        if (detectObstacleInPath(cloud)) {
            publishEmergencyStop();
        }
    }

    bool detectObstacleInPath(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->empty()) {
            return false;
        }

        // Calculate stopping distance based on current speed
        double stopping_distance = current_speed_ * collision_time_threshold_;
        double check_distance = std::min(stopping_distance, max_check_distance_);
        
        // Define the region of interest (ROI) in front of the vehicle
        // Assuming vehicle coordinate frame: x-forward, y-left, z-up
        double roi_min_x = min_safe_distance_;
        double roi_max_x = check_distance;
        double roi_min_y = -vehicle_width_ / 2.0;
        double roi_max_y = vehicle_width_ / 2.0;
        double roi_min_z = -1.0;  // Ground level consideration
        double roi_max_z = 2.0;   // Maximum obstacle height to consider

        // Filter points within the ROI
        pcl::CropBox<pcl::PointXYZ> crop_filter;
        crop_filter.setInputCloud(cloud);
        crop_filter.setMin(Eigen::Vector4f(roi_min_x, roi_min_y, roi_min_z, 1.0));
        crop_filter.setMax(Eigen::Vector4f(roi_max_x, roi_max_y, roi_max_z, 1.0));

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        crop_filter.filter(*filtered_cloud);

        // Check if there are enough points to indicate an obstacle
        int obstacle_point_threshold = 10; // Minimum points to consider as obstacle
        bool obstacle_detected = filtered_cloud->size() > obstacle_point_threshold;

        if (obstacle_detected) {
            // Find the closest obstacle point
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& point : filtered_cloud->points) {
                double distance = std::sqrt(point.x * point.x + point.y * point.y);
                min_distance = std::min(min_distance, distance);
            }

            // Calculate time to collision
            double time_to_collision = min_distance / std::abs(current_speed_);

            RCLCPP_WARN(this->get_logger(), 
                       "Obstacle detected! Distance: %.2fm, Speed: %.2fm/s, Time to collision: %.2fs",
                       min_distance, current_speed_, time_to_collision);

            return time_to_collision <= collision_time_threshold_;
        }

        return false;
    }

    void publishEmergencyStop()
    {
        auto emergency_msg = ackermann_msgs::msg::AckermannDriveStamped();
        emergency_msg.header.stamp = this->get_clock()->now();
        emergency_msg.header.frame_id = "base_link";
        
        // Emergency stop command
        emergency_msg.drive.speed = 0.0;
        emergency_msg.drive.steering_angle = 0.0;
        emergency_msg.drive.acceleration = -5.0;  // Hard braking
        emergency_msg.drive.jerk = 0.0;
        emergency_msg.drive.steering_angle_velocity = 0.0;

        emergency_pub_->publish(emergency_msg);
        
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
        
        // Keep publishing emergency stop for a short duration
        emergency_stop_count_++;
        if (emergency_stop_count_ > 20) { // Reset after 1 second at ~20Hz
            emergency_stop_count_ = 0;
        }
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr emergency_pub_;

    // Parameters
    double collision_time_threshold_;
    double min_safe_distance_;
    double vehicle_width_;
    double max_check_distance_;
    double min_speed_threshold_;

    // State variables
    double current_speed_;
    bool has_odom_data_ = false;
    rclcpp::Time last_odom_time_;
    int emergency_stop_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyBraker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
