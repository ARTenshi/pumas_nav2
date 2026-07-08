// Copyright (C) 2016 Toyota Motor Corporation
// maintainer ry0hei-kobayashi
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <pumas_interfaces/msg/motion_pose.hpp>

#include <algorithm>
#include <cmath>
#include <string>

class HeadController : public rclcpp::Node {
public:
  HeadController() : Node("head_controller") {
    // ############
    //  Declare parameters with default values
    this->declare_parameter<bool>("use_namespace", false);
    this->declare_parameter<std::string>(
        "head_cmd_topic", "/head_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>(
        "head_state_topic", "/head_trajectory_controller/controller_state");
    this->declare_parameter<std::string>("head_goal_topic",
                                         "/hardware/head/goal_pose");
    this->declare_parameter<std::string>("head_current_topic",
                                         "/hardware/head/current_pose");
    this->declare_parameter<std::string>("head_goal_reached_topic",
                                         "/hardware/head/goal_reached");
    this->declare_parameter<double>("pan_min", -3.14);
    this->declare_parameter<double>("pan_max", 1.74);
    this->declare_parameter<double>("tilt_min", -0.9);
    this->declare_parameter<double>("tilt_max", 0.47);
    this->declare_parameter<bool>("move_head", true);
    // When no explicit motion time is supplied (gaze / simple_move path), the
    // trajectory is time-parameterized from the travel distance so the
    // trajectory controller eases in/out instead of snapping (time_from_start=0).
    // dt = max(head_min_time, (|dpan| + |dtilt|) / head_default_speed)
    this->declare_parameter<double>("head_default_speed", 1.0); // [rad/s]
    this->declare_parameter<double>("head_min_time", 0.15);     // [s]

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("head_cmd_topic", head_cmd_topic_);
    this->get_parameter("head_state_topic", head_state_topic_);
    this->get_parameter("head_goal_topic", head_goal_topic_);
    this->get_parameter("head_current_topic", head_current_topic_);
    this->get_parameter("head_goal_reached_topic", head_goal_reached_topic_);
    this->get_parameter("pan_min", pan_min_);
    this->get_parameter("pan_max", pan_max_);
    this->get_parameter("tilt_min", tilt_min_);
    this->get_parameter("tilt_max", tilt_max_);
    this->get_parameter("move_head", move_head_);
    this->get_parameter("head_default_speed", head_default_speed_);
    this->get_parameter("head_min_time", head_min_time_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &HeadController::on_parameter_change, this, std::placeholders::_1));

    // Publishers
    pub_head_goal_traj_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            head_cmd_topic_, rclcpp::QoS(10).reliable());
    pub_head_current_pose_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            head_current_topic_, rclcpp::QoS(10).reliable());
    pub_head_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(
        head_goal_reached_topic_, rclcpp::QoS(10).reliable());

    // Subscribers
    sub_head_goal_pose_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            head_goal_topic_, rclcpp::QoS(10).reliable(),
            std::bind(&HeadController::headGoalPoseCallback, this,
                      std::placeholders::_1));

    sub_head_state_ = this->create_subscription<
        control_msgs::msg::JointTrajectoryControllerState>(
        head_state_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&HeadController::headStateCallback, this,
                  std::placeholders::_1));

    sub_motion_pose_ =
        this->create_subscription<pumas_interfaces::msg::MotionPose>(
            "/hardware/motion_pose", rclcpp::QoS(10).reliable(),
            std::bind(&HeadController::motionPoseCallback, this,
                      std::placeholders::_1));

    // Init
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HeadController::timerCallback, this));

    // initialize head position
    head_goal_pose_.assign(2, 0.0f);
    head_current_pose_.assign(2, 0.0f);

    RCLCPP_INFO(this->get_logger(), "HeadController.->Node has been started.");
  }

private:
  bool use_namespace_ = false;

  std::string head_cmd_topic_;
  std::string head_state_topic_;
  std::string head_goal_topic_;
  std::string head_current_topic_;
  std::string head_goal_reached_topic_;

  double pan_min_, pan_max_, tilt_min_, tilt_max_;
  bool move_head_ = true;

  // Distance-proportional trajectory timing for the gaze / simple_move path
  // (used only when no explicit motion_execution_time override is given).
  double head_default_speed_ = 1.0; // [rad/s]
  double head_min_time_ = 0.15;     // [s]

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      pub_head_goal_traj_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      pub_head_current_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_head_goal_reached_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_head_goal_pose_;
  rclcpp::Subscription<pumas_interfaces::msg::MotionPose>::SharedPtr
      sub_motion_pose_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::
      SharedPtr sub_head_state_;

  static constexpr double kDefaultHeadTimeFromStart = 0.0;
  double head_time_from_start_{kDefaultHeadTimeFromStart};

  std::vector<float> head_goal_pose_;
  std::vector<float> head_current_pose_;
  bool goal_received_{false};

  bool startup_initialized_ = false;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ############
  //  Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
      if (param.get_name() == "use_namespace")
        use_namespace_ = param.as_bool();

      else if (param.get_name() == "head_cmd_topic")
        head_cmd_topic_ = param.as_string();
      else if (param.get_name() == "head_state_topic")
        head_state_topic_ = param.as_string();
      else if (param.get_name() == "head_goal_topic")
        head_goal_topic_ = param.as_string();
      else if (param.get_name() == "head_current_topic")
        head_current_topic_ = param.as_string();
      else if (param.get_name() == "head_goal_reached_topic")
        head_goal_reached_topic_ = param.as_string();

      else if (param.get_name() == "move_head") {
        move_head_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "HeadController.-> move_head set to %s",
                    move_head_ ? "true (motion_synth head enabled)"
                               : "false (gaze controller owns head)");
      }

      else if (param.get_name() == "head_default_speed")
        head_default_speed_ = param.as_double();
      else if (param.get_name() == "head_min_time")
        head_min_time_ = param.as_double();

      else {
        result.successful = false;
        result.reason =
            "HeadController.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(
            this->get_logger(),
            "HeadController.-> Attempted to update unsupported parameter: %s",
            param.get_name().c_str());
        break;
      }
    }

    return result;
  }

  std::string make_name(const std::string &suffix) const {
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
      const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    // expected: positions.size() >= 2  (0:pan, 1:tilt)
    if (msg->actual.positions.size() >= 2) {
      head_current_pose_[0] =
          static_cast<float>(msg->actual.positions[0]); // pan
      head_current_pose_[1] =
          static_cast<float>(msg->actual.positions[1]); // tilt

      std_msgs::msg::Float32MultiArray arr;
      arr.data = head_current_pose_;
      pub_head_current_pose_->publish(arr);
    }

    if (!startup_initialized_) {
      if (goal_received_) {
        // A real goal already took over; skip the startup centering pose.
        startup_initialized_ = true;
      } else if (pub_head_goal_traj_->get_subscription_count() > 0) {
        // Trajectory controller is connected now: a reliable publish will be
        // delivered. (Before the match completes the sample would be dropped.)
        sendHeadGoalTrajectory(0.0f, 0.0f);
        startup_initialized_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "HeadController.-> Head Pose Initialized (controller connected).");
      }
      // else: controller not connected yet — wait for the next
      // controller_state.
    }
  }

  void
  headGoalPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(
          this->get_logger(),
          "[head_controller] Head goal must be 2 values [pan, tilt] (got %zu)",
          msg->data.size());
      return;
    }

    float pan = static_cast<float>(msg->data[0]);
    float tilt = static_cast<float>(msg->data[1]);

    // clamp to limits
    head_goal_pose_[0] =
        std::clamp(static_cast<float>(pan), static_cast<float>(pan_min_),
                   static_cast<float>(pan_max_));
    head_goal_pose_[1] =
        std::clamp(static_cast<float>(tilt), static_cast<float>(tilt_min_),
                   static_cast<float>(tilt_max_));

    goal_received_ = true;
    sendHeadGoalTrajectory(head_goal_pose_[0], head_goal_pose_[1]);
  }

  void timerCallback() {
    // reach check
    constexpr float eps_pan = 0.01f;
    constexpr float eps_tilt = 0.01f;

    bool reached =
        (std::fabs(head_current_pose_[0] - head_goal_pose_[0]) <= eps_pan) &&
        (std::fabs(head_current_pose_[1] - head_goal_pose_[1]) <= eps_tilt);

    std_msgs::msg::Bool b;
    b.data = reached;
    pub_head_goal_reached_->publish(b);
  }

  void
  motionPoseCallback(const pumas_interfaces::msg::MotionPose::SharedPtr msg) {
    if (!move_head_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "head_node.-> move_head=false: ignoring motion_pose head "
                   "command (gaze active)");
      return;
    }

    head_time_from_start_ =
        (msg->motion_execution_time > 0.0f)
            ? static_cast<double>(msg->motion_execution_time)
            : kDefaultHeadTimeFromStart;

    head_goal_pose_[0] =
        std::clamp(msg->joints.head_pan_joint, static_cast<float>(pan_min_),
                   static_cast<float>(pan_max_));
    head_goal_pose_[1] =
        std::clamp(msg->joints.head_tilt_joint, static_cast<float>(tilt_min_),
                   static_cast<float>(tilt_max_));

    goal_received_ = true;
    RCLCPP_INFO(
        this->get_logger(),
        "head_node.->Received motion pose: pan=%.3f tilt=%.3f time=%.3f s",
        head_goal_pose_[0], head_goal_pose_[1], head_time_from_start_);

    sendHeadGoalTrajectory(head_goal_pose_[0], head_goal_pose_[1]);
  }

  void sendHeadGoalTrajectory(float pan, float tilt) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"head_pan_joint", "head_tilt_joint"};

    // Decide the trajectory duration. An explicit override from motion_synth
    // (motion_execution_time) wins. Otherwise (gaze / simple_move path) the
    // trajectory is time-parameterized from the travel distance so the
    // controller ramps to a zero-velocity stop instead of being asked to reach
    // the goal instantly (time_from_start=0), which looked mechanical.
    double time_from_start = head_time_from_start_;
    if (time_from_start <= 0.0) {
      const double speed =
          (head_default_speed_ > 1e-3) ? head_default_speed_ : 1.0;
      const double dpan =
          std::fabs(static_cast<double>(pan) - head_current_pose_[0]);
      const double dtilt =
          std::fabs(static_cast<double>(tilt) - head_current_pose_[1]);
      time_from_start = std::max(head_min_time_, (dpan + dtilt) / speed);
    }

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {pan, tilt};
    // Zero terminal velocity/acceleration -> decelerate into the target
    // (ease-out by construction, as in tmc_viewpoint_controller).
    p.velocities = {0.0, 0.0};
    p.accelerations = {0.0, 0.0};
    p.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

    traj.points.push_back(p);
    pub_head_goal_traj_->publish(traj);

    head_time_from_start_ = kDefaultHeadTimeFromStart;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
