#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>

class WaypointRecorder : public rclcpp::Node
{
public:
    WaypointRecorder() : Node("waypoint_recorder"), recording_(false)
    {
        // Declare parameters
        this->declare_parameter("output_file", "waypoints.yaml");
        this->declare_parameter("recording_distance_threshold", 0.5);
        
        output_file_ = this->get_parameter("output_file").as_string();
        distance_threshold_ = this->get_parameter("recording_distance_threshold").as_double();
        
        // Subscribers
	pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
	    "/pcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
	    std::bind(&WaypointRecorder::poseCallback, this, std::placeholders::_1));
        
        record_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start_recording", 10,
            std::bind(&WaypointRecorder::recordCallback, this, std::placeholders::_1));
        
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/recorded_path", 10);
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Recorder initialized");
        RCLCPP_INFO(this->get_logger(), "Publish to /start_recording to start/stop recording");
        RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
	{
	    if (!recording_)
	    {
		return;
	    }

	    if (!recorded_path_.poses.empty())
	    {
		const auto& last_pose = recorded_path_.poses.back();
		double dx = msg->pose.pose.position.x - last_pose.pose.position.x;
		double dy = msg->pose.pose.position.y - last_pose.pose.position.y;
		double distance = std::sqrt(dx * dx + dy * dy);

		if (distance < distance_threshold_)
		{
		    return;
		}
	    }

	    // Convert PoseWithCovarianceStamped to PoseStamped
	    geometry_msgs::msg::PoseStamped pose_stamped;
	    pose_stamped.header = msg->header;
	    pose_stamped.pose = msg->pose.pose;

	    recorded_path_.poses.push_back(pose_stamped);
	    recorded_path_.header = msg->header;

	    path_pub_->publish(recorded_path_);

	    RCLCPP_INFO(this->get_logger(), "Recorded waypoint %zu at (%.2f, %.2f)", 
		       recorded_path_.poses.size(), msg->pose.pose.position.x, msg->pose.pose.position.y);
	}

    
    void recordCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !recording_)
        {
            // Start recording
            recording_ = true;
            recorded_path_.poses.clear();
            recorded_path_.header.frame_id = "map";
            recorded_path_.header.stamp = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "Started recording waypoints");
        }
        else if (!msg->data && recording_)
        {
            // Stop recording and save
            recording_ = false;
            saveWaypoints();
            RCLCPP_INFO(this->get_logger(), "Stopped recording waypoints");
        }
    }
    
    void saveWaypoints()
    {
        if (recorded_path_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints to save");
            return;
        }
        
        try
        {
            YAML::Node waypoints_node;
            waypoints_node["frame_id"] = recorded_path_.header.frame_id;
            
            for (size_t i = 0; i < recorded_path_.poses.size(); ++i)
            {
                const auto& pose = recorded_path_.poses[i];
                YAML::Node waypoint;
                waypoint["x"] = pose.pose.position.x;
                waypoint["y"] = pose.pose.position.y;
                waypoint["z"] = pose.pose.position.z;
                waypoint["qx"] = pose.pose.orientation.x;
                waypoint["qy"] = pose.pose.orientation.y;
                waypoint["qz"] = pose.pose.orientation.z;
                waypoint["qw"] = pose.pose.orientation.w;
                waypoint["velocity"] = 1.0; // Default velocity
                
                waypoints_node["waypoints"].push_back(waypoint);
            }
            
            std::ofstream file(output_file_);
            file << waypoints_node;
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "Saved %zu waypoints to %s", 
                       recorded_path_.poses.size(), output_file_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save waypoints: %s", e.what());
        }
    }
    
    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // State
    bool recording_;
    nav_msgs::msg::Path recorded_path_;
    std::string output_file_;
    double distance_threshold_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WaypointRecorder>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Waypoint Recorder...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 
