#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <chrono>
#include <memory>

class ControlCenterNode : public rclcpp::Node
{
public:
    ControlCenterNode() : Node("control_center_node"), 
                         message_timeout_(std::chrono::seconds(1))
    {
        // Publishers
        ackermann_cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackermann_cmd", 10);

        // Subscribers
        joy_cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "joy_cmd", 10, 
            std::bind(&ControlCenterNode::joyCallback, this, std::placeholders::_1));

        purepursuit_cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "purepursuit_cmd", 10,
            std::bind(&ControlCenterNode::purepursuitCallback, this, std::placeholders::_1));

        emergency_cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "emergency_cmd", 10,
            std::bind(&ControlCenterNode::emergencyCallback, this, std::placeholders::_1));
        
        stopsign_cmd_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "stopsign_cmd", 10,
            std::bind(&ControlCenterNode::stopsignCallback, this, std::placeholders::_1));
        

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&ControlCenterNode::publishCmd, this));

        // Initialize message timestamps
        joy_last_received_ = this->get_clock()->now();
        purepursuit_last_received_ = this->get_clock()->now();
        emergency_last_received_ = this->get_clock()->now();
        stopsign_last_received_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Control Center Node started");
    }

private:
    void joyCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        joy_cmd_ = *msg;
        joy_last_received_ = this->get_clock()->now();
        has_joy_msg_ = true;
    }

    void purepursuitCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        purepursuit_cmd_ = *msg;
        purepursuit_last_received_ = this->get_clock()->now();
        has_purepursuit_msg_ = true;
    }

    void emergencyCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        emergency_cmd_ = *msg;
        emergency_last_received_ = this->get_clock()->now();
        has_emergency_msg_ = true;
    }
    
    void stopsignCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        stopsign_cmd_ = *msg;
        stopsign_last_received_ = this->get_clock()->now();
        has_stopsign_msg_ = true;
    }

    void publishCmd()
    {
        auto now = this->get_clock()->now();
        
        // Check if messages are still valid (not timed out)
        bool emergency_valid = has_emergency_msg_ && 
                              (now - emergency_last_received_) < message_timeout_;
        bool purepursuit_valid = has_purepursuit_msg_ && 
                                (now - purepursuit_last_received_) < message_timeout_;
        bool joy_valid = has_joy_msg_ && 
                        (now - joy_last_received_) < message_timeout_;
        bool stopsign_valid = has_stopsign_msg_ && 
                        (now - stopsign_last_received_) < message_timeout_; 
                        
                          
        ackermann_msgs::msg::AckermannDriveStamped cmd_to_publish;
        std::string active_source = "none";

        // Priority logic: emergency > purepursuit > joy
        if (emergency_valid) {
            cmd_to_publish = emergency_cmd_;
            cmd_to_publish.header.stamp = now;
            active_source = "emergency";
        }else if (stopsign_valid) {
            cmd_to_publish.drive.speed = stopsign_cmd_.drive.speed;    
            cmd_to_publish.drive.steering_angle = purepursuit_cmd_.drive.steering_angle;
            cmd_to_publish.drive.acceleration = purepursuit_cmd_.drive.acceleration;
            cmd_to_publish.drive.jerk = purepursuit_cmd_.drive.jerk;
            cmd_to_publish.drive.steering_angle_velocity = purepursuit_cmd_.drive.steering_angle_velocity;
            cmd_to_publish.header.stamp = now;
            active_source = "stopsign only"; 
        }  
        
        else if (stopsign_valid && purepursuit_valid) {
            if(stopsign_cmd_.drive.speed != 0.0){
            	cmd_to_publish.drive.speed = 0.0;	
            }else{
            	cmd_to_publish.drive.speed = purepursuit_cmd_.drive.speed * 0.5;
            }
        
            cmd_to_publish.drive.steering_angle = purepursuit_cmd_.drive.steering_angle;
            cmd_to_publish.drive.acceleration = purepursuit_cmd_.drive.acceleration;
            cmd_to_publish.drive.jerk = purepursuit_cmd_.drive.jerk;
            cmd_to_publish.drive.steering_angle_velocity = purepursuit_cmd_.drive.steering_angle_velocity;
            cmd_to_publish.header.stamp = now;
            active_source = "stopsign"; 
        } else if (purepursuit_valid) {
            cmd_to_publish = purepursuit_cmd_;
            cmd_to_publish.header.stamp = now;
            active_source = "purepursuit";
        } else if (joy_valid) {
            cmd_to_publish = joy_cmd_;
            cmd_to_publish.header.stamp = now;
            active_source = "joy";
        } else {
            // No valid commands, publish a stop command
            cmd_to_publish.header.stamp = now;
            cmd_to_publish.header.frame_id = "map";
            cmd_to_publish.drive.speed = 0.0;
            cmd_to_publish.drive.steering_angle = 0.0;
            cmd_to_publish.drive.acceleration = 0.0;
            cmd_to_publish.drive.jerk = 0.0;
            cmd_to_publish.drive.steering_angle_velocity = 0.0;
            active_source = "stop";
        }

        // Publish the command
        ackermann_cmd_pub_->publish(cmd_to_publish);

        // Log the active source periodically
        static int log_counter = 0;
        if (log_counter % 40 == 0) {  // Log every 2 seconds at 20 Hz
            RCLCPP_INFO(this->get_logger(), "Active command source: %s", active_source.c_str());
        }
        log_counter++;
    }

    // Publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_cmd_pub_;

    // Subscribers
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr joy_cmd_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr purepursuit_cmd_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr emergency_cmd_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr stopsign_cmd_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Message storage
    ackermann_msgs::msg::AckermannDriveStamped joy_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped purepursuit_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped emergency_cmd_;
    ackermann_msgs::msg::AckermannDriveStamped stopsign_cmd_;

    // Message validity tracking
    bool has_joy_msg_ = false;
    bool has_purepursuit_msg_ = false;
    bool has_emergency_msg_ = false;
    bool has_stopsign_msg_ = false;

    // Timestamps
    rclcpp::Time joy_last_received_;
    rclcpp::Time purepursuit_last_received_;
    rclcpp::Time emergency_last_received_;
    rclcpp::Time stopsign_last_received_;

    // Message timeout
    rclcpp::Duration message_timeout_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlCenterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
