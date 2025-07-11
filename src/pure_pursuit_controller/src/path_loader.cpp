#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

class PathLoader : public rclcpp::Node
{
public:
    PathLoader() : Node("path_loader")
    {
        // Declare parameters
        this->declare_parameter("waypoints_file", "waypoints.yaml");
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("publish_on_startup", true);
        
        waypoints_file_ = this->get_parameter("waypoints_file").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_on_startup_ = this->get_parameter("publish_on_startup").as_bool();
        
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/follow_path", 10);
        
        // Subscribers
        load_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/load_path", 10,
            std::bind(&PathLoader::loadCallback, this, std::placeholders::_1));
        
        // Load and publish path on startup if requested
        if (publish_on_startup_)
        {
            loadAndPublishPath();
        }
        
        RCLCPP_INFO(this->get_logger(), "Path Loader initialized");
        RCLCPP_INFO(this->get_logger(), "Waypoints file: %s", waypoints_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish to /load_path to reload waypoints");
    }

private:
    void loadCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            loadAndPublishPath();
        }
    }
    
    void loadAndPublishPath()
    {
        try
        {
            if (!std::ifstream(waypoints_file_).good())
            {
                RCLCPP_ERROR(this->get_logger(), "Waypoints file not found: %s", waypoints_file_.c_str());
                return;
            }
            
            YAML::Node waypoints_node = YAML::LoadFile(waypoints_file_);
            
            nav_msgs::msg::Path path;
            path.header.frame_id = frame_id_;
            path.header.stamp = this->get_clock()->now();
            
            // Override frame_id if specified in file
            if (waypoints_node["frame_id"])
            {
                path.header.frame_id = waypoints_node["frame_id"].as<std::string>();
            }
            
            if (!waypoints_node["waypoints"])
            {
                RCLCPP_ERROR(this->get_logger(), "No waypoints found in file");
                return;
            }
            
            for (const auto& waypoint_node : waypoints_node["waypoints"])
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header = path.header;
                
                pose_stamped.pose.position.x = waypoint_node["x"].as<double>();
                pose_stamped.pose.position.y = waypoint_node["y"].as<double>();
                pose_stamped.pose.position.z = waypoint_node["z"].as<double>(0.0);
                
                pose_stamped.pose.orientation.x = waypoint_node["qx"].as<double>(0.0);
                pose_stamped.pose.orientation.y = waypoint_node["qy"].as<double>(0.0);
                pose_stamped.pose.orientation.z = waypoint_node["qz"].as<double>(0.0);
                pose_stamped.pose.orientation.w = waypoint_node["qw"].as<double>(1.0);
                
                path.poses.push_back(pose_stamped);
            }
            
            if (path.poses.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid waypoints loaded");
                return;
            }
            
            // Publish the path
            path_pub_->publish(path);
            
            RCLCPP_INFO(this->get_logger(), "Loaded and published %zu waypoints from %s", 
                       path.poses.size(), waypoints_file_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
        }
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr load_sub_;
    
    // Parameters
    std::string waypoints_file_;
    std::string frame_id_;
    bool publish_on_startup_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PathLoader>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Path Loader...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 