// Copyright (C) 2016 Toyota Motor Corporation
// maintainer by ry0hei-kobayashi 
// WIP
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <actionlib_msgs/msg/goal_status.hpp>

#include <string>
#include <cmath>

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    //############
    // Declare parameters with default values
    this->declare_parameter<bool>(        "use_namespace",   false);
    this->declare_parameter<std::string>( "arm_cmd_topic",   "/arm_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>( "arm_state_topic", "/arm_trajectory_controller/controller_state");

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace",    use_namespace_);
    this->get_parameter("arm_cmd_topic",    arm_cmd_topic_);
    this->get_parameter("arm_state_topic",  arm_state_topic_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArmController::on_parameter_change, this, std::placeholders::_1));

    // Publishers
    pub_torso_current_pose_ = this->create_publisher<std_msgs::msg::Float32>(
          make_name("/hardware/torso/current_pose"), 
          rclcpp::QoS(10).transient_local());
    pub_arm_current_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
          make_name("/hardware/arm/current_pose"), 
          rclcpp::QoS(10).transient_local());
    pub_arm_goal_pose_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_cmd_topic_, 
      rclcpp::QoS(10).transient_local());
    pub_arm_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/hardware/arm/goal_reached"), 
      rclcpp::QoS(10).transient_local());
    pub_torso_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/hardware/torso/goal_reached"), 
      rclcpp::QoS(10).transient_local());

    // qos
    auto qos_goal = rclcpp::QoS(rclcpp::KeepLast(10))
                    .reliability(rclcpp::ReliabilityPolicy::Reliable)
                    .durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                     .durability(rclcpp::DurabilityPolicy::Volatile);

    // Subscribers
    sub_arm_goal_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        make_name("/hardware/arm/goal_pose"), 
        qos_goal,
        std::bind(&ArmController::armGoalPoseCallback, this, std::placeholders::_1));

    sub_torso_goal_pose_ = this->create_subscription<std_msgs::msg::Float32>(
        make_name("/hardware/torso/goal_pose"), 
        qos_goal,
        std::bind(&ArmController::torsoGoalPoseCallback, this, std::placeholders::_1));

    sub_arm_state_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        arm_state_topic_, 
        qos_state,
        std::bind(&ArmController::armCurrentPoseCallback, this, std::placeholders::_1));

    // Init
    arm_goal_pose_.assign(4, 0.0f);
    arm_current_pose_.assign(4, 0.0f);
    torso_goal_pose_ = 0.0f;
    torso_current_pose_ = 0.0f;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArmController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Arm Controller Node has been started.");
  }

private:

  bool use_namespace_   = false;

  std::string arm_cmd_topic_;
  std::string arm_state_topic_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                pub_torso_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      pub_arm_current_pose_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_goal_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   pub_arm_goal_reached_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   pub_torso_goal_reached_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   sub_arm_goal_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr             sub_torso_goal_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_arm_state_;

  float torso_goal_pose_{0.0f};
  float torso_current_pose_{0.0f};
  std::vector<float> arm_goal_pose_;
  std::vector<float> arm_current_pose_;

  bool msg_arm_received_{false};
  bool msg_torso_received_{false};

  double torso_default_pose_{0.0};
  std::vector<double> arm_default_pose_{0.0, -1.57, -1.57, 0.0};

  bool default_send_once_{true};

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
      if (param.get_name()      == "use_namespace")   use_namespace_    = param.as_bool();

      else if (param.get_name() == "arm_cmd_topic")   arm_cmd_topic_    = param.as_string();
      else if (param.get_name() == "arm_state_topic") arm_state_topic_  = param.as_string();

      else {
        result.successful = false;
        result.reason = "ArmController.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(this->get_logger(), "ArmController.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
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

  void armCurrentPoseCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
  
    torso_current_pose_ = static_cast<float>(msg->feedback.positions[0]); // arm_lift_joint
  
    if (arm_current_pose_.size() != 4) arm_current_pose_.assign(4, 0.0f);
    arm_current_pose_[0] = static_cast<float>(msg->feedback.positions[1]); // arm_flex_joint
    arm_current_pose_[1] = static_cast<float>(msg->feedback.positions[2]); // arm_roll_joint
    arm_current_pose_[2] = static_cast<float>(msg->feedback.positions[3]); // wrist_flex_joint
    arm_current_pose_[3] = static_cast<float>(msg->feedback.positions[4]); // wrist_roll_joint
  
    // Publish current
    std_msgs::msg::Float32 torso_msg; torso_msg.data = torso_current_pose_;
    pub_torso_current_pose_->publish(torso_msg);
  
    std_msgs::msg::Float32MultiArray arm_msg; arm_msg.data = arm_current_pose_;
    pub_arm_current_pose_->publish(arm_msg);

    if (default_send_once_) {
      publish_default_pose();
      default_send_once_ = false;
    }

  }

  void armGoalPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "arm_node.->Arm data must be 4 values (got %zu)",
                   msg->data.size());
      return;
    }

    arm_goal_pose_ = std::vector<float>(msg->data.begin(), msg->data.end());
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received arm goal pose: [%f, %f, %f, %f]",
                 arm_goal_pose_[0], arm_goal_pose_[1], arm_goal_pose_[2], arm_goal_pose_[3]);
    msg_arm_received_ = true;
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received arm goal pose.");
    sendArmGoalTrajectory();
  }

  void torsoGoalPoseCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    torso_goal_pose_ = msg->data;
    msg_torso_received_ = true;
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received torso goal pose.");
  }

  void timerCallback()
  {
    constexpr float eps_arm = 0.01f;
    constexpr float eps_torso = 0.005f;

    bool arm_reached = true;
    for (size_t i = 0; i < 4; ++i) {
      if (std::fabs(arm_current_pose_[i] - arm_goal_pose_[i]) > eps_arm) {
        arm_reached = false;
        break;
      }
    }

    bool torso_reached = (std::fabs(torso_current_pose_ - torso_goal_pose_) <= eps_torso);

    std_msgs::msg::Bool b;
    b.data = arm_reached;
    pub_arm_goal_reached_->publish(b);

    b.data = torso_reached;
    pub_torso_goal_reached_->publish(b);

    //if (nav_trigger_){
    //    nav_trigger_ = false;
    //    publish_default_pose();
    //}
  }

  void publish_default_pose()
  {
    torso_goal_pose_ = static_cast<float>(torso_default_pose_);
    arm_goal_pose_.resize(4);
    arm_goal_pose_[0] = static_cast<float>(arm_default_pose_[0]);
    arm_goal_pose_[1] = static_cast<float>(arm_default_pose_[1]);
    arm_goal_pose_[2] = static_cast<float>(arm_default_pose_[2]);
    arm_goal_pose_[3] = static_cast<float>(arm_default_pose_[3]);

    sendArmGoalTrajectory();

    RCLCPP_INFO(this->get_logger(),
      "arm_node.->Publish Default Arm Joints: torso=%.3f arm=[%.3f %.3f %.3f %.3f]",
      torso_goal_pose_, arm_goal_pose_[0], arm_goal_pose_[1], arm_goal_pose_[2], arm_goal_pose_[3]);

  }

  void sendArmGoalTrajectory()
  {
  
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "arm_lift_joint","arm_flex_joint","arm_roll_joint","wrist_flex_joint","wrist_roll_joint"
    };
    traj.points.resize(1);
    auto &pt = traj.points[0];
    pt.positions.resize(5);
  
    pt.positions[0] = static_cast<double>(torso_goal_pose_);  // arm_lift_joint
    pt.positions[1] = static_cast<double>(arm_goal_pose_[0]); // arm_flex_joint
    pt.positions[2] = static_cast<double>(arm_goal_pose_[1]); // arm_roll_joint
    pt.positions[3] = static_cast<double>(arm_goal_pose_[2]); // wrist_flex_joint   
    pt.positions[4] = static_cast<double>(arm_goal_pose_[3]); // wrist_roll_joint
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0); 
  
    pub_arm_goal_pose_->publish(traj);
  
    msg_arm_received_   = false;
    msg_torso_received_ = false;
  
    RCLCPP_INFO(this->get_logger(),
      "arm_node.->Publish Arm Joints: torso=%.3f arm=[%.3f %.3f %.3f %.3f]",
      pt.positions[0], pt.positions[1], pt.positions[2], pt.positions[3], pt.positions[4]);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto arm_controller_node = std::make_shared<ArmController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_controller_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
