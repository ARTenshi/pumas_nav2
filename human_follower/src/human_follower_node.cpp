#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Standard
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
///

#define RATE 30

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 11
#define SM_GOAL_POSE_ACCEL 1
#define SM_GOAL_POSE_CRUISE 2
#define SM_GOAL_POSE_DECCEL 3
#define SM_GOAL_POSE_CORRECT_ANGLE 4
#define SM_GOAL_POSE_FINISH 10
#define SM_GOAL_POSE_FAILED 12
#define SM_GOAL_PATH_ACCEL 5
#define SM_GOAL_PATH_CRUISE 6
#define SM_GOAL_PATH_DECCEL 7
#define SM_GOAL_PATH_FINISH 8
#define SM_GOAL_PATH_FAILED 81
#define SM_COLLISION_RISK 9

class HumanFollowerNode : public rclcpp::Node
{
public:
    HumanFollowerNode() : Node("human_follower_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<bool>("use_namespace",          false);
        this->declare_parameter<double>("control_alpha",        0.6548);
        this->declare_parameter<double>("control_beta",         0.3);
        this->declare_parameter<double>("max_linear",           0.3);
        this->declare_parameter<double>("max_angular",          0.7);
        this->declare_parameter<double>("dist_to_human",        0.9);

        this->declare_parameter<bool>("move_backwards",         false);
        this->declare_parameter<bool>("move_head",              false);
        this->declare_parameter<bool>("pot_fields",             false);
        
        this->declare_parameter<std::string>("legs_pose_topic", "/hri/leg_finder/leg_pose");
        this->declare_parameter<std::string>("cmd_vel_topic",   "/cmd_vel");
        this->declare_parameter<std::string>("head_topic",      "/hardware/head/goal_pose");
        
        this->declare_parameter<std::string>("base_link_frame",  "base_footprint");

        // Initialize internal variables from declared parameters
        this->get_parameter("use_namespace",      use_namespace_);
        this->get_parameter("control_alpha",      control_alpha_);
        this->get_parameter("control_beta",       control_beta_);
        this->get_parameter("max_linear",         max_linear_);
        this->get_parameter("max_angular",        max_angular_);
        this->get_parameter("dist_to_human",      dist_to_human_);
        
        this->get_parameter("move_backwards",     move_backwards_);
        this->get_parameter("move_head",          move_head_);
        this->get_parameter("pot_fields",         pot_fields_);
        
        this->get_parameter("legs_pose_topic",    legs_pose_topic_);
        this->get_parameter("cmd_vel_topic",      cmd_vel_topic_);
        this->get_parameter("head_topic",         head_topic_);
        
        this->get_parameter("base_link_frame",     base_link_frame_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&HumanFollowerNode::on_parameter_change, this, std::placeholders::_1));


        //############
        // Publishers
        pub_cmd_vel_   = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 
            rclcpp::QoS(10).transient_local());
        pub_head_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            head_topic_, 
            rclcpp::QoS(10).transient_local());

        //############
        // Subscribers
        sub_rejection_force_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            make_name("/navigation/potential_fields/pf_rejection_force"),
            rclcpp::SensorDataQoS(),
            std::bind(&HumanFollowerNode::callback_rejection_force, this, std::placeholders::_1));

        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            make_name("/hri/human_following/enable"),
            rclcpp::SensorDataQoS(),
            std::bind(&HumanFollowerNode::callback_enable, this, std::placeholders::_1));

        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/stop"),
            rclcpp::SensorDataQoS(),
            std::bind(&HumanFollowerNode::callback_stop, this, std::placeholders::_1));
        
        //############
        // Wait for transforms
        //wait_for_transforms("base_link", "map");

        // Simple Move main processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&HumanFollowerNode::human_follower_processing, this));

        RCLCPP_INFO(this->get_logger(), "HumanFollowerNode.-> HumanFollowerNode is ready.");
    }

private:
    //############
    // Parameters / state
    bool use_namespace_      = false;

    double control_alpha_    = 0.6548;
    double control_beta_     = 0.3;
    double max_linear_       = 0.3;
    double max_angular_      = 0.7;
    double dist_to_human_    = 0.9;
    bool   move_backwards_   = false;
    bool   move_head_        = false;
    bool   pot_fields_       = false;

    std::string legs_pose_topic_ = "/hri/leg_finder/leg_pose";
    std::string cmd_vel_topic_   = "/cmd_vel";
    std::string head_topic_      = "/hardware/head/goal_pose";
    
    std::string base_link_frame_  = "base_footprint";

    bool  new_legs_pose_ = false;
    bool  enable_        = false;
    int   no_legs_count_ = 0;
    double repulsive_force_y_ = 0.0;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    //############
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr               pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        pub_head_pose_;

    //############
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          sub_rejection_force_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr     sub_legs_pose_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                  sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr                 sub_stop_;


    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Main processing loop
    rclcpp::TimerBase::SharedPtr processing_timer_;


    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name() == "use_namespace")          use_namespace_    = param.as_bool();
            else if (param.get_name() == "control_alpha")     control_alpha_    = param.as_double();
            else if (param.get_name() == "control_beta")      control_beta_     = param.as_double();
            else if (param.get_name() == "max_linear")        max_linear_       = param.as_double();
            else if (param.get_name() == "max_angular")       max_angular_      = param.as_double();
            else if (param.get_name() == "dist_to_human")     dist_to_human_    = param.as_double();
            else if (param.get_name() == "move_backwards")    move_backwards_   = param.as_bool();
            else if (param.get_name() == "move_head")         move_head_        = param.as_bool();
            else if (param.get_name() == "pot_fields")        pot_fields_       = param.as_bool();
            else if (param.get_name() == "legs_pose_topic")   legs_pose_topic_  = param.as_string();
            else if (param.get_name() == "cmd_vel_topic")     cmd_vel_topic_    = param.as_string();
            else if (param.get_name() == "head_topic")        head_topic_       = param.as_string();
            else if (param.get_name() == "base_link_frame")    base_link_frame_   = param.as_string();

            else {
                result.successful = false;
                result.reason = "HumanFollower.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "HumanFollower.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    std::string make_name(const std::string &suffix) const
    {
        // Ensure suffix starts with "/"
        std::string sfx = suffix;
        if (!sfx.empty() && sfx.front() != '/')
            sfx = "/" + sfx;

        std::string name;

        if (use_namespace_) {
            // Use node namespace prefix
            name = this->get_namespace() + sfx;

            // Avoid accidental double slash (e.g., when namespace is "/")
            if (name.size() > 1 && name[0] == '/' && name[1] == '/')
                name.erase(0, 1);
        } else {
            // Use global namespace (no node namespace prefix)
            name = sfx;
        }

        return name;
    }

    // Wait for transforms 
    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "HumanFollower.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

        rclcpp::Time start_time = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0); 

        bool transform_ok = false;

        while (rclcpp::ok() && (this->now() - start_time) < timeout) {
            try {
                tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1)
                );
                transform_ok = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "HumanFollower.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "HumanFollower.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "HumanFollower.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }


    //############
    //Human Follower additional functions
    geometry_msgs::msg::Twist calculate_speeds(double goal_x, double goal_y)
    {
        double angle_error = std::atan2(goal_y, goal_x);
        double distance = std::sqrt(goal_x*goal_x + goal_y*goal_y) - dist_to_human_;
        if (!move_backwards_) 
        {
            if (distance < 0.0) distance = 0.0;
        }
        if (distance > max_linear_) distance = max_linear_;

        geometry_msgs::msg::Twist result;
        if (distance > 0.0 || move_backwards_) 
        {
            result.linear.x = distance * std::exp(-(angle_error * angle_error) / control_alpha_);
            result.linear.y = 0.0;
            if (pot_fields_) result.linear.y = repulsive_force_y_;
            result.angular.z = max_angular_ * (2.0 / (1.0 + std::exp(-angle_error / control_beta_)) - 1.0);
        } else 
        {
            result.linear.x = 0.0;
            result.linear.y = 0.0;
            if (std::fabs(angle_error) >= M_PI_4 / 6.0) // retain original condition
                result.angular.z = max_angular_ * (2.0 / (1.0 + std::exp(-angle_error / control_beta_)) - 1.0);
            else
                result.angular.z = 0.0;
        }
        return result;
    }

    bool transform_to_robot_position(double in_x, double in_y, const std::string &frame_id,
                                     double &out_x, double &out_y)
    {
        try 
        {
            geometry_msgs::msg::PointStamped p_in, p_out;
            p_in.header.stamp = this->get_clock()->now();
            p_in.header.frame_id = frame_id;
            p_in.point.x = in_x;
            p_in.point.y = in_y;
            p_in.point.z = 0.0;

            // Transform to base_link
            geometry_msgs::msg::TransformStamped tf = 
                tf_buffer_.lookupTransform(base_link_frame_, frame_id, tf2::TimePointZero);
            
            tf2::doTransform(p_in, p_out, tf);

            out_x = p_out.point.x;
            out_y = p_out.point.y;

            return true;
        } catch (const tf2::TransformException &ex) 
        {
            RCLCPP_WARN(this->get_logger(), "HumanFollower.-> TF transform error: %s", ex.what());
            return false;
        }
    }


    //############
    //Human Follower callbacks
    void callback_legs_pose(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        double human_x = msg->point.x;
        double human_y = msg->point.y;

        if (msg->header.frame_id != "base_link") {
            (void)transform_to_robot_position(human_x, human_y, msg->header.frame_id, human_x, human_y);
        }

        if (move_head_) {
            std_msgs::msg::Float64MultiArray head_poses;
            head_poses.data.push_back(std::atan2(msg->point.y, msg->point.x));
            head_poses.data.push_back(-0.6);
            pub_head_pose_->publish(head_poses);
        }

        pub_cmd_vel_->publish(calculate_speeds(human_x, human_y));
        new_legs_pose_ = true;
    }


    void callback_rejection_force(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        repulsive_force_y_ = msg->y;
    }


    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        enable_ = msg->data;

        if (enable_) {
            RCLCPP_INFO(this->get_logger(), "HumanFollower.-> Enable received.");
            if (!sub_legs_pose_) {
                sub_legs_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                    legs_pose_topic_, rclcpp::SensorDataQoS(),
                    std::bind(&HumanFollowerNode::callback_legs_pose, this, std::placeholders::_1));
            }
        } else {
            if (sub_legs_pose_) {
                sub_legs_pose_.reset();
            }
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr)
    {
        if (sub_legs_pose_) {
            sub_legs_pose_.reset();
        }
    }

    //############
    //Human Follower main processing
    void human_follower_processing() 
    {
        try
        {
            if (!enable_) return;

            if (new_legs_pose_) {
                new_legs_pose_ = false;
                no_legs_count_ = 0;
            } else {
                // roughly every tick is ~33ms
                if (++no_legs_count_ > 30) {
                    no_legs_count_ = 0;
                    geometry_msgs::msg::Twist stop;
                    stop.linear.x = 0.0;
                    stop.angular.z = 0.0;
                    pub_cmd_vel_->publish(stop);
                }
            }
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "HumanFollower.-> Error in HumanFollower processing: %s", e.what());
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanFollowerNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
