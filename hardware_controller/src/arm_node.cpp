// Copyright (C) 2016 Toyota Motor Corporation
// maintainer by ry0hei-kobayashi 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <actionlib_msgs/msg/goal_status.hpp>

#include <pumas_interfaces/msg/motion_pose.hpp>

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
    this->declare_parameter("torso_default_pose", 0.0);
    this->declare_parameter<std::vector<double>>("arm_default_pose", {0.0, -1.57, -1.57, 0.0});


    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace",    use_namespace_);
    this->get_parameter("arm_cmd_topic",    arm_cmd_topic_);
    this->get_parameter("arm_state_topic",  arm_state_topic_);
    this->get_parameter("torso_default_pose", torso_default_pose_);
    this->get_parameter("arm_default_pose", arm_default_pose_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArmController::on_parameter_change, this, std::placeholders::_1));

    // Publishers
    pub_torso_current_pose_ = this->create_publisher<std_msgs::msg::Float32>(
          make_name("/hardware/torso/current_pose"), 
          rclcpp::QoS(10).reliable());
    pub_arm_current_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
          make_name("/hardware/arm/current_pose"), 
          rclcpp::QoS(10).reliable());
    pub_arm_goal_pose_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      arm_cmd_topic_, 
      rclcpp::QoS(10).reliable());
    pub_arm_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/hardware/arm/goal_reached"), 
      rclcpp::QoS(10).reliable());
    pub_torso_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/hardware/torso/goal_reached"), 
      rclcpp::QoS(10).reliable());

    // Subscribers
    sub_arm_goal_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        make_name("/hardware/arm/goal_pose"),
        rclcpp::QoS(10).reliable(),
        std::bind(&ArmController::armGoalPoseCallback, this, std::placeholders::_1));

    sub_torso_goal_pose_ = this->create_subscription<std_msgs::msg::Float32>(
        make_name("/hardware/torso/goal_pose"),
        rclcpp::QoS(10).reliable(),
        std::bind(&ArmController::torsoGoalPoseCallback, this, std::placeholders::_1));

    // motion_synth publishes a MotionPose (full Joints + motion_execution_time)
    // on a single topic shared with head_controller. We pick up the lift/arm
    // fields and ignore the head fields.
    sub_motion_pose_ = this->create_subscription<pumas_interfaces::msg::MotionPose>(
        "/hardware/motion_pose",
        rclcpp::QoS(10).reliable(),
        std::bind(&ArmController::motionPoseCallback, this, std::placeholders::_1));

    sub_arm_state_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        arm_state_topic_, 
        rclcpp::QoS(10).best_effort(),
        std::bind(&ArmController::armCurrentPoseCallback, this, std::placeholders::_1));

    // Init
    arm_goal_pose_.assign(4, 0.0f);
    arm_current_pose_.assign(4, 0.0f);
    torso_goal_pose_ = 0.0f;
    torso_current_pose_ = 0.0f;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArmController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "arm_node.->Arm Controller Node has been started.");
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
  rclcpp::Subscription<pumas_interfaces::msg::MotionPose>::SharedPtr  sub_motion_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_arm_state_;

  // Default time_from_start for trajectories. Used when the caller does not
  // override (legacy simple_move path). motion_synth provides its own value
  // via ArmGoalPose.motion_execution_time.
  static constexpr double kDefaultArmTimeFromStart = 0.2;
  double arm_time_from_start_{kDefaultArmTimeFromStart};

  float torso_goal_pose_{0.0f};
  float torso_current_pose_{0.0f};
  std::vector<float> arm_goal_pose_;
  std::vector<float> arm_current_pose_;

  bool msg_arm_received_{false};
  bool msg_torso_received_{false};

  double torso_default_pose_{0.0};
  std::vector<double> arm_default_pose_{0.0, -1.57, -1.57, 0.0};

  // Startup default-pose handshake. The default pose must not be published
  // until the trajectory controller has actually connected to our command
  // publisher; a trajectory sent before discovery completes is silently
  // dropped (reliable but volatile QoS), which is why the arm sometimes never
  // moved to the default pose at startup. Once the subscription is matched a
  // single reliable publish is delivered (the protocol retransmits on loss),
  // so one send after connection is enough.
  bool init_done_{false};
  bool external_goal_received_{false};

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

    if (!init_done_) {
      if (external_goal_received_) {
        // A real goal already took over; skip the startup default pose.
        init_done_ = true;
      } else if (pub_arm_goal_pose_->get_subscription_count() > 0) {
        // Trajectory controller is connected now: a reliable publish will be
        // delivered. (Before the match completes the sample would be dropped.)
        publish_default_pose();
        init_done_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "arm_node.-> Sent default pose (controller connected).");
      }
      // else: controller not connected yet — wait for the next controller_state.
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
    external_goal_received_ = true;
    sendArmGoalTrajectory();
  }

  void torsoGoalPoseCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    torso_goal_pose_ = msg->data;
    msg_torso_received_ = true;
    RCLCPP_INFO(this->get_logger(), "arm_node.->Received torso goal pose.");
  }

  void motionPoseCallback(const pumas_interfaces::msg::MotionPose::SharedPtr msg)
  {
    torso_goal_pose_ = msg->joints.arm_lift_joint;
    arm_goal_pose_.resize(4);
    arm_goal_pose_[0] = msg->joints.arm_flex_joint;
    arm_goal_pose_[1] = msg->joints.arm_roll_joint;
    arm_goal_pose_[2] = msg->joints.wrist_flex_joint;
    arm_goal_pose_[3] = msg->joints.wrist_roll_joint;

    arm_time_from_start_ = (msg->motion_execution_time > 0.0f)
        ? static_cast<double>(msg->motion_execution_time)
        : kDefaultArmTimeFromStart;

    msg_arm_received_   = true;
    msg_torso_received_ = true;
    external_goal_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "arm_node.->Received motion pose: lift=%.3f arm=[%.3f %.3f %.3f %.3f] time=%.3f s",
                torso_goal_pose_, arm_goal_pose_[0], arm_goal_pose_[1],
                arm_goal_pose_[2], arm_goal_pose_[3], arm_time_from_start_);

    sendArmGoalTrajectory();
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
    pt.time_from_start = rclcpp::Duration::from_seconds(arm_time_from_start_);

    pub_arm_goal_pose_->publish(traj);

    // Reset to default after consuming so the override does not bleed into
    // subsequent commands from other publishers.
    arm_time_from_start_ = kDefaultArmTimeFromStart;

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
