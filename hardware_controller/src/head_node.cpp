// Copyright (C) 2016 Toyota Motor Corporation
// maintainer ry0hei-kobayashi
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <string>
#include <cmath>

class HeadController : public rclcpp::Node
{
public:
  HeadController()
  : Node("head_controller")
  {
    //############
    // Declare parameters with default values
    this->declare_parameter<bool>(        "use_namespace",            false);
    this->declare_parameter<std::string>( "head_cmd_topic_",          "/head_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>( "head_state_topic",         "/head_trajectory_controller/controller_state");
    this->declare_parameter<std::string>( "head_goal_topic",          "/hardware/head/goal_pose");
    this->declare_parameter<std::string>( "head_current_topic",       "/hardware/head/current_pose");
    this->declare_parameter<std::string>( "head_goal_reached_topic",  "/hardware/head/goal_reached");
    this->declare_parameter<double>("pan_min", -3.14);
    this->declare_parameter<double>("pan_max", 1.74);
    this->declare_parameter<double>("tilt_min", -0.9);
    this->declare_parameter<double>("tilt_max", 0.47);


    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace",            use_namespace_);
    this->get_parameter("head_cmd_topic_",          head_cmd_topic_);
    this->get_parameter("head_state_topic",         head_state_topic_);
    this->get_parameter("head_goal_topic",          head_goal_topic_);
    this->get_parameter("head_current_topic",       head_current_topic_);
    this->get_parameter("head_goal_reached_topic",  head_goal_reached_topic_);
    this->get_parameter("pan_min", pan_min_);
    this->get_parameter("pan_max", pan_max_);
    this->get_parameter("tilt_min", tilt_min_);
    this->get_parameter("tilt_max", tilt_max_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HeadController::on_parameter_change, this, std::placeholders::_1));

    // Publishers
    pub_head_goal_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      head_cmd_topic_, 
      rclcpp::QoS(10).reliable());
    pub_head_current_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      head_current_topic_, 
      rclcpp::QoS(10).reliable());
    pub_head_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
      head_goal_reached_topic_, 
      rclcpp::QoS(10).reliable());

    // Subscribers
    sub_head_goal_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      head_goal_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&HeadController::headGoalPoseCallback, this, std::placeholders::_1));

    sub_head_state_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      head_state_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&HeadController::headStateCallback, this, std::placeholders::_1));

    // Init
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HeadController::timerCallback, this));

    //initialize head position
    head_goal_pose_.assign(2, 0.0f);
    head_current_pose_.assign(2, 0.0f);


    RCLCPP_INFO(this->get_logger(), "HeadController.->Node has been started.");
  }

private:

  bool use_namespace_   = false;

  std::string head_cmd_topic_;
  std::string head_state_topic_;
  std::string head_goal_topic_;
  std::string head_current_topic_;
  std::string head_goal_reached_topic_;

  double pan_min_, pan_max_, tilt_min_, tilt_max_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr  pub_head_goal_traj_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr       pub_head_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                    pub_head_goal_reached_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr    sub_head_goal_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_head_state_;

  std::vector<float> head_goal_pose_;
  std::vector<float> head_current_pose_;
  bool goal_received_{false};

  bool starup_initialized_ = false;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr timer_;

  //############
  // Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params)
    {
      if (param.get_name()      == "use_namespace")           use_namespace_            = param.as_bool();

      else if (param.get_name() == "head_cmd_topic_")         head_cmd_topic_           = param.as_string();
      else if (param.get_name() == "head_state_topic")        head_state_topic_         = param.as_string();
      else if (param.get_name() == "head_goal_topic")         head_goal_topic_          = param.as_string();
      else if (param.get_name() == "head_current_topic")      head_current_topic_       = param.as_string();
      else if (param.get_name() == "head_goal_reached_topic") head_goal_reached_topic_  = param.as_string();

      else {
        result.successful = false;
        result.reason = "HeadController.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(this->get_logger(), "HeadController.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
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

  void headStateCallback(
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    // expected: positions.size() >= 2  (0:pan, 1:tilt)
    if (msg->feedback.positions.size() >= 2) {
      head_current_pose_[0] = static_cast<float>(msg->feedback.positions[0]); // pan
      head_current_pose_[1] = static_cast<float>(msg->feedback.positions[1]); // tilt

      std_msgs::msg::Float32MultiArray arr;
      arr.data = head_current_pose_;
      pub_head_current_pose_->publish(arr);
    }

    if (!starup_initialized_) {
      sendHeadGoalTrajectory(0.0f, 0.0f);
      starup_initialized_ = true;
      RCLCPP_WARN(this->get_logger(), "HeadController.-> Head Pose Initialized.");
    }

  }

  void headGoalPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(this->get_logger(),
                   "[head_controller] Head goal must be 2 values [pan, tilt] (got %zu)",
                   msg->data.size());
      return;
    }

    float pan  = static_cast<float>(msg->data[0]);
    float tilt = static_cast<float>(msg->data[1]);

    // clamp to limits
    head_goal_pose_[0] = std::clamp(static_cast<float>(pan),  static_cast<float>(pan_min_),  static_cast<float>(pan_max_));
    head_goal_pose_[1] = std::clamp(static_cast<float>(tilt), static_cast<float>(tilt_min_), static_cast<float>(tilt_max_));

    goal_received_ = true;
    sendHeadGoalTrajectory(head_goal_pose_[0], head_goal_pose_[1]);
  }

  void timerCallback()
  {
    // reach check
    constexpr float eps_pan  = 0.01f;
    constexpr float eps_tilt = 0.01f;

    bool reached = (std::fabs(head_current_pose_[0] - head_goal_pose_[0]) <= eps_pan) &&
                   (std::fabs(head_current_pose_[1] - head_goal_pose_[1]) <= eps_tilt);

    std_msgs::msg::Bool b;
    b.data = reached;
    pub_head_goal_reached_->publish(b);
  }

  void sendHeadGoalTrajectory(float pan, float tilt)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_pan_joint", "head_tilt_joint"}; 

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {pan, tilt};
    p.time_from_start = rclcpp::Duration::from_seconds(0.0);

    traj.points.push_back(p);
    pub_head_goal_traj_->publish(traj);
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

