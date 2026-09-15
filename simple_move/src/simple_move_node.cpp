#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

// Message types
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

// TF2 (Transform listener)
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

// Standard
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
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
#define SM_GOAL_REL_POSE 13
#define SM_GOAL_PATH_CONSTANT 14

class SimpleMoveNode : public rclcpp::Node {
public:
  SimpleMoveNode()
      : Node("simple_move_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // ############
    //  Declare parameters with default values
    this->declare_parameter<bool>("use_namespace", false);
    this->declare_parameter<float>("max_linear_speed", 0.3f);
    this->declare_parameter<float>("min_linear_speed", 0.05f);
    this->declare_parameter<float>("max_angular_speed", 1.0f);
    // Constant-speed (cruise-only) mode for path following.
    // When use_constant_speed is true the trapezoidal accel/cruise/deccel
    // profile is skipped and the robot follows the path at a fixed
    // constant_speed [m/s] (still attenuated on sharp turns by the controller).
    this->declare_parameter<bool>("use_constant_speed", false);
    this->declare_parameter<float>("constant_speed", 0.3f);
    // Last-mile omni. The unicycle controller drives the path (the body
    // naturally faces the travel direction); once the goal is closer than
    // yaw_correction_omni_distance [m] the tracking switches to the holonomic
    // controller, which slides onto the goal while yawing onto its orientation
    // instead of arriving and turning in place.
    this->declare_parameter<bool>("yaw_correction_omni_behavior", false);
    this->declare_parameter<float>("yaw_correction_omni_distance", 1.0f);
    // Yaw gain used only while the last-mile omni drives. Deliberately strong:
    // the goal is to arrive with so little yaw error that mvn_pln's final-angle
    // correction has nothing left to do.
    this->declare_parameter<float>("yaw_correction_omni_gain", 2.5f);
    // Ignore collision_risk and the pot-fields rejection during the last
    // mile. The goal usually sits right next to furniture, so the front
    // detection box and the rejection force fight the convergence exactly
    // when the robot is closing the final, slow stretch. If the way is
    // truly blocked the attempts timeout still aborts.
    this->declare_parameter<bool>("yaw_correction_ignore_obstacles", true);
    // Aim the head at the goal orientation during the last mile, so it turns
    // onto the goal yaw together with the body. When false the head keeps its
    // normal path tracking (looking toward the goal/travel direction) the whole
    // way in. The base still yaws onto the goal orientation either way.
    this->declare_parameter<bool>("yaw_correction_align_head", true);
    // How far down the path the camera looks, in path points. The planner emits
    // one point per map cell, so this is (points x resolution) metres ahead.
    // Raising it makes the head anticipate turns earlier.
    this->declare_parameter<int>("head_lookahead_points", 5);
    this->declare_parameter<float>("control_alpha", 0.6548f);
    this->declare_parameter<float>("control_beta", 0.2f);
    this->declare_parameter<float>("linear_acceleration", 0.1f);
    this->declare_parameter<float>("fine_dist_tolerance", 0.03f);
    this->declare_parameter<float>("coarse_dist_tolerance", 0.2f);
    this->declare_parameter<float>("angle_tolerance", 0.05f);

    this->declare_parameter<bool>("move_head", true);
    this->declare_parameter<bool>("use_pot_fields", false);

    // Active rotate-in-place recovery. Off by default: flickering obstacle
    // reports can make the base oscillate, so this is opt-in per environment
    // (narrow passages) rather than always-on.
    this->declare_parameter<bool>("allow_rotation_on_collision", false);
    this->declare_parameter<float>("rotation_collision_angle_threshold", 0.35f);
    this->declare_parameter<float>("rotation_collision_angular_speed", 0.4f);
    this->declare_parameter<float>("rotation_collision_max_time", 3.0f);

    this->declare_parameter<std::string>("base_link_name", "base_footprint");
    this->declare_parameter<std::string>("odom_name", "odom");

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("use_constant_speed", use_constant_speed_);
    this->get_parameter("constant_speed", constant_speed_);
    this->get_parameter("yaw_correction_omni_behavior",
                        yaw_correction_omni_behavior_);
    this->get_parameter("yaw_correction_omni_distance",
                        yaw_correction_omni_distance_);
    this->get_parameter("yaw_correction_omni_gain", yaw_correction_omni_gain_);
    this->get_parameter("yaw_correction_ignore_obstacles",
                        yaw_correction_ignore_obstacles_);
    this->get_parameter("yaw_correction_align_head",
                        yaw_correction_align_head_);
    this->get_parameter("head_lookahead_points", head_lookahead_points_);
    this->get_parameter("control_alpha", alpha_);
    this->get_parameter("control_beta", beta_);
    this->get_parameter("linear_acceleration", linear_acceleration_);
    this->get_parameter("fine_dist_tolerance", fine_dist_tolerance_);
    this->get_parameter("coarse_dist_tolerance", coarse_dist_tolerance_);
    this->get_parameter("angle_tolerance", angle_tolerance_);

    this->get_parameter("move_head", move_head_);
    this->get_parameter("use_pot_fields", use_pot_fields_);

    this->get_parameter("allow_rotation_on_collision",
                        allow_rotation_on_collision_);
    this->get_parameter("rotation_collision_angle_threshold",
                        rotation_collision_angle_threshold_);
    this->get_parameter("rotation_collision_angular_speed",
                        rotation_collision_angular_speed_);
    this->get_parameter("rotation_collision_max_time",
                        rotation_collision_max_time_);
    reset_rotation_budget();

    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("odom_name", odom_name_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &SimpleMoveNode::on_parameter_change, this, std::placeholders::_1));

    // ############
    //  Publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10).transient_local());
    pub_goal_reached_ = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
        make_name("/simple_move/goal_reached"),
        rclcpp::QoS(10).transient_local());
    pub_head_goal_pose_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            make_name("/hardware/head/goal_pose"),
            rclcpp::QoS(10).transient_local());

    // ############
    //  Subscribers
    //// stop
    sub_generalStop_ = this->create_subscription<std_msgs::msg::Empty>(
        make_name("/stop"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_general_stop, this,
                  std::placeholders::_1));

    sub_navCtrlStop_ = this->create_subscription<std_msgs::msg::Empty>(
        make_name("/navigation/stop"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_navigation_stop, this,
                  std::placeholders::_1));

    sub_navSimpleMvStop_ = this->create_subscription<std_msgs::msg::Empty>(
        make_name("/simple_move/stop"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_simple_move_stop, this,
                  std::placeholders::_1));

    //// goal
    sub_goalDistance_ = this->create_subscription<std_msgs::msg::Float32>(
        make_name("/simple_move/goal_dist"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_goal_dist, this,
                  std::placeholders::_1));

    sub_goalDistAngle_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_dist_angle"), rclcpp::SensorDataQoS(),
            std::bind(&SimpleMoveNode::callback_goal_dist_angle, this,
                      std::placeholders::_1));

    sub_goalPath_ = this->create_subscription<nav_msgs::msg::Path>(
        make_name("/simple_move/goal_path"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_goal_path, this,
                  std::placeholders::_1));

    //// motion
    sub_moveLateral_ = this->create_subscription<std_msgs::msg::Float32>(
        make_name("/simple_move/goal_dist_lateral"), rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_move_lateral, this,
                  std::placeholders::_1));

    // TODO rel is not srv or action
    sub_goalRelPose_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_rel_pose"), rclcpp::QoS(10).reliable(),
            std::bind(&SimpleMoveNode::callback_goal_rel_pose, this,
                      std::placeholders::_1));

    sub_collisionRisk_ = this->create_subscription<std_msgs::msg::Bool>(
        make_name("/navigation/potential_fields/collision_risk"),
        rclcpp::SensorDataQoS(),
        std::bind(&SimpleMoveNode::callback_collision_risk, this,
                  std::placeholders::_1));

    sub_rejectionForce_ =
        this->create_subscription<geometry_msgs::msg::Vector3>(
            make_name("/navigation/potential_fields/pf_rejection_force"),
            rclcpp::SensorDataQoS(),
            std::bind(&SimpleMoveNode::callback_rejection_force, this,
                      std::placeholders::_1));

    // ############
    //  Wait for transforms
    wait_for_transforms("map", base_link_name_);
    wait_for_transforms(odom_name_, base_link_name_);

    // Simple Move main processing.
    //
    // This must be a node-clock timer, not a wall timer. Every per-tick
    // quantity below (the linear_acceleration ramp, the `attempts` budgets,
    // rotation_ticks_remaining_) is written as "value / RATE", i.e. it assumes
    // one tick is exactly 1/RATE of a *simulated* second. create_wall_timer
    // ignores use_sim_time, so under a simulator running slower than realtime
    // the loop gets RATE/rtf ticks per simulated second: the acceleration ramp
    // is scaled by 1/rtf and every timeout expires after rtf times its
    // intended simulated duration. Measured on Isaac Sim at rtf 0.47 that is a
    // 2.4x faster ramp and timeouts firing at 47% of the configured time,
    // which aborts path tracking during the deceleration before the goal.
    // The period is derived from RATE so the two can no longer disagree (it
    // used to be a hardcoded 30 ms = 33.3 Hz against RATE = 30).
    processing_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / RATE),
        std::bind(&SimpleMoveNode::simple_move_processing, this));

    RCLCPP_INFO(this->get_logger(), "SimpleMove.-> SimpleMoveNode is ready.");
  }

private:
  // ############
  //  State variables
  bool use_namespace_ = false;
  float goal_distance_ = 0;
  float goal_angle_ = 0;
  bool new_pose_ = false;
  bool new_path_ = false;
  bool collision_risk_ = false;
  bool move_lat_ = false;
  bool stop_ = false;

  // Active rotate-in-place recovery (see collision_blocks_tracking).
  bool allow_rotation_on_collision_ = false;
  float rotation_collision_angle_threshold_ = 0.35f;
  float rotation_collision_angular_speed_ = 0.4f;
  float rotation_collision_max_time_ = 3.0f;
  // Budget in control ticks, so the rotation cannot spin forever while the
  // collision risk persists. Replenished per path, not per rotation attempt.
  int rotation_ticks_remaining_ = 0;

  // for lelative move
  bool new_rel_pose_ = false;
  float rel_x_ = 0;
  float rel_y_ = 0;
  float rel_yaw_ = 0;

  float robot_x_ = 0;
  float robot_y_ = 0;
  float robot_t_ = 0;
  float goal_x_ = 0;
  float goal_y_ = 0;
  float goal_t_ = 0;
  float global_goal_x_ = 0;
  float global_goal_y_ = 0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Path goal_path_;
  geometry_msgs::msg::Vector3 rejection_force_;

  // Internal parameter values
  float max_linear_speed_;
  float min_linear_speed_;
  float max_angular_speed_;
  bool use_constant_speed_ = false;
  float constant_speed_;
  bool yaw_correction_omni_behavior_ = false;
  float yaw_correction_omni_distance_;
  float yaw_correction_omni_gain_;
  bool yaw_correction_ignore_obstacles_ = true;
  bool yaw_correction_align_head_ = true;
  int head_lookahead_points_;
  float alpha_;
  float beta_;
  float linear_acceleration_;
  float fine_dist_tolerance_;
  float coarse_dist_tolerance_;
  float angle_tolerance_;

  bool move_head_;
  bool use_pot_fields_;
  std::string base_link_name_;
  std::string odom_name_;

  // Simple Move processing variables
  actionlib_msgs::msg::GoalStatus msg_goal_reached;

  int state = SM_INIT;
  float current_linear_speed = 0;
  int prev_pose_idx = 0;
  int next_pose_idx = 0;
  float temp_k = 0;
  int attempts = 0;
  float global_error = 0;

  // Goal orientation, carried on the last path pose by mvn_pln. Absent when the
  // path comes from somewhere that does not request a final yaw.
  float path_goal_t_ = 0;
  bool path_goal_t_valid_ = false;

  // ############
  //  Publishers
  rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr
      pub_goal_reached_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      pub_head_goal_pose_;

  // ############
  //  Subscribers
  //// stop
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_generalStop_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_navCtrlStop_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_navSimpleMvStop_;

  //// goal
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_goalDistance_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_goalDistAngle_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_goalPath_;

  //// motion
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_moveLateral_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_goalRelPose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collisionRisk_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
      sub_rejectionForce_;

  // ############
  //  Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Main processing loop
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // ############
  //  Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
      if (param.get_name() == "use_namespace")
        use_namespace_ = param.as_bool();
      else if (param.get_name() == "max_linear_speed")
        max_linear_speed_ = param.as_double();
      else if (param.get_name() == "min_linear_speed")
        min_linear_speed_ = param.as_double();
      else if (param.get_name() == "max_angular_speed")
        max_angular_speed_ = param.as_double();
      else if (param.get_name() == "use_constant_speed")
        use_constant_speed_ = param.as_bool();
      else if (param.get_name() == "constant_speed")
        constant_speed_ = param.as_double();
      else if (param.get_name() == "yaw_correction_omni_behavior")
        yaw_correction_omni_behavior_ = param.as_bool();
      else if (param.get_name() == "yaw_correction_omni_distance")
        yaw_correction_omni_distance_ = param.as_double();
      else if (param.get_name() == "yaw_correction_omni_gain")
        yaw_correction_omni_gain_ = param.as_double();
      else if (param.get_name() == "yaw_correction_ignore_obstacles")
        yaw_correction_ignore_obstacles_ = param.as_bool();
      else if (param.get_name() == "yaw_correction_align_head")
        yaw_correction_align_head_ = param.as_bool();
      else if (param.get_name() == "head_lookahead_points")
        head_lookahead_points_ = param.as_int();
      else if (param.get_name() == "control_alpha")
        alpha_ = param.as_double();
      else if (param.get_name() == "control_beta")
        beta_ = param.as_double();
      else if (param.get_name() == "linear_acceleration")
        linear_acceleration_ = param.as_double();
      else if (param.get_name() == "fine_dist_tolerance")
        fine_dist_tolerance_ = param.as_double();
      else if (param.get_name() == "coarse_dist_tolerance")
        coarse_dist_tolerance_ = param.as_double();
      else if (param.get_name() == "angle_tolerance")
        angle_tolerance_ = param.as_double();

      else if (param.get_name() == "move_head")
        move_head_ = param.as_bool();
      else if (param.get_name() == "use_pot_fields")
        use_pot_fields_ = param.as_bool();

      else if (param.get_name() == "allow_rotation_on_collision")
        allow_rotation_on_collision_ = param.as_bool();
      else if (param.get_name() == "rotation_collision_angle_threshold")
        rotation_collision_angle_threshold_ = param.as_double();
      else if (param.get_name() == "rotation_collision_angular_speed")
        rotation_collision_angular_speed_ = param.as_double();
      else if (param.get_name() == "rotation_collision_max_time") {
        rotation_collision_max_time_ = param.as_double();
        reset_rotation_budget();
      }

      else if (param.get_name() == "base_link_name")
        base_link_name_ = param.as_string();
      else if (param.get_name() == "odom_name")
        odom_name_ = param.as_string();

      else {
        result.successful = false;
        result.reason =
            "SimpleMove.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(
            this->get_logger(),
            "SimpleMove.-> Attempted to update unsupported parameter: %s",
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

  // Wait for transforms
  void wait_for_transforms(const std::string &target_frame,
                           const std::string &source_frame) {
    RCLCPP_INFO(this->get_logger(),
                "SimpleMove.-> Waiting for transform from '%s' to '%s'...",
                source_frame.c_str(), target_frame.c_str());

    // Steady clock, deliberately not this->now(). This runs in the constructor,
    // before the executor spins the node, so with use_sim_time the node's
    // TimeSource has not received /clock yet and this->now() would stay pinned
    // at 0 - the elapsed time would never grow and the loop would never time
    // out. TF itself still arrives here because tf2_ros::TransformListener
    // spins its own internal node on a dedicated thread.
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);

    bool transform_ok = false;

    while (rclcpp::ok() &&
           (std::chrono::steady_clock::now() - start_time) < timeout) {
      try {
        tf_buffer_.lookupTransform(target_frame, source_frame,
                                   tf2::TimePointZero,
                                   tf2::durationFromSec(0.1));
        transform_ok = true;
        break;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "SimpleMove.-> Still waiting for transform: %s",
                             ex.what());
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (!transform_ok) {
      RCLCPP_WARN(this->get_logger(),
                  "SimpleMove.-> Timeout while waiting for transform from '%s' "
                  "to '%s'.",
                  source_frame.c_str(), target_frame.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "SimpleMove.-> Transform from '%s' to '%s' is now available.",
                  source_frame.c_str(), target_frame.c_str());
    }
  }

  // ############
  // Simple Move callbacks
  void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> General Stop signal received" << std::endl;
      stop_ = true;
      new_pose_ = false;
      new_path_ = false;
      move_lat_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_general_stop: %s",
                   e.what());
    }
  }

  void callback_navigation_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> Navigation Stop signal received" << std::endl;
      stop_ = true;
      new_pose_ = false;
      new_path_ = false;
      move_lat_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "SimpleMove.-> Error processing callback_navigation_stop: %s",
          e.what());
    }
  }

  void callback_simple_move_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> Simple move stop signal received"
                << std::endl;
      stop_ = true;
      new_pose_ = false;
      new_path_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "SimpleMove.-> Error processing callback_simple_move_stop: %s",
          e.what());
    }
  }

  void callback_goal_dist(const std_msgs::msg::Float32::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> New move received: goal dist= " << msg->data
                << std::endl;
      goal_distance_ = msg->data;
      goal_angle_ = 0;
      new_pose_ = true;
      new_path_ = false;
      move_lat_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_goal_dist: %s",
                   e.what());
    }
  }

  void callback_goal_dist_angle(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> New move received: goal dist= "
                << msg->data[0] << " and goal angle= " << msg->data[1]
                << std::endl;
      goal_distance_ = msg->data[0];
      goal_angle_ = msg->data[1];
      new_pose_ = true;
      new_path_ = false;
      new_rel_pose_ = false;
      move_lat_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "SimpleMove.-> Error processing callback_goal_dist_angle: %s",
          e.what());
    }
  }

  void callback_goal_path(const nav_msgs::msg::Path::SharedPtr msg) {
    try {
      std::cout << "SimpleMove.-> New path received with " << msg->poses.size()
                << " points. Stamp= " << msg->header.stamp.sec
                << "s, Frame=" << msg->header.frame_id << std::endl;
      move_lat_ = false;
      if (msg->poses.size() <= 0) {
        new_pose_ = false;
        new_path_ = false;
      } else {
        goal_path_ = *msg;
        new_pose_ = false;
        new_path_ = true;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_goal_path: %s",
                   e.what());
    }
  }

  void callback_move_lateral(const std_msgs::msg::Float32::SharedPtr msg) {
    try {
      goal_distance_ = msg->data;
      goal_angle_ = 0;
      new_pose_ = true;
      new_path_ = false;
      new_rel_pose_ = false;
      move_lat_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_move_lateral: %s",
                   e.what());
    }
  }

  // Relative holonomic move: data = [x, y, yaw] in the robot (base) frame.
  // x: forward(+)/back(-), y: left(+)/right(-), yaw: rotate [rad].
  void callback_goal_rel_pose(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    try {
      if (msg->data.size() != 3) {
        RCLCPP_ERROR(
            this->get_logger(),
            "SimpleMove.-> goal_rel_pose needs 3 values [x, y, yaw] (got %zu)",
            msg->data.size());
        return;
      }
      std::cout << "SimpleMove.-> New relative move: x=" << msg->data[0]
                << " y=" << msg->data[1] << " yaw=" << msg->data[2]
                << std::endl;
      rel_x_ = msg->data[0];
      rel_y_ = msg->data[1];
      rel_yaw_ = msg->data[2];
      new_rel_pose_ = true;
      new_pose_ = false;
      new_path_ = false;
      move_lat_ = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_goal_rel_pose: %s",
                   e.what());
    }
  }

  void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg) {
    try {
      collision_risk_ = msg->data;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error processing callback_collision_risk: %s",
                   e.what());
    }
  }

  void
  callback_rejection_force(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    try {
      rejection_force_ = *msg;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "SimpleMove.-> Error processing callback_rejection_force: %s",
          e.what());
    }
  }

  // ############
  // Simple Move additional functions
  geometry_msgs::msg::Twist
  calculate_speeds(float robot_x, float robot_y, float robot_t, float goal_x,
                   float goal_y, float min_linear_speed, float max_linear_speed,
                   float angular_speed, float alpha, float beta, bool backwards,
                   bool move_lat, bool use_pot_fields = false,
                   double rejection_force_y = 0.0) {
    float angle_error = 0;
    if (backwards)
      angle_error = (atan2(robot_y - goal_y, robot_x - goal_x) - robot_t);
    else
      angle_error = (atan2(goal_y - robot_y, goal_x - robot_x) - robot_t);
    if (angle_error > M_PI)
      angle_error -= 2 * M_PI;
    if (angle_error <= -M_PI)
      angle_error += 2 * M_PI;
    if (move_lat)
      angle_error -= M_PI / 2;
    if (angle_error <= -M_PI)
      angle_error += 2 * M_PI;
    if (backwards)
      max_linear_speed *= -1;

    geometry_msgs::msg::Twist result;
    if (move_lat)
      result.linear.y =
          max_linear_speed * exp(-(angle_error * angle_error) / (alpha));
    else
      result.linear.x =
          max_linear_speed * exp(-(angle_error * angle_error) / (alpha));
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);

    if (fabs(result.linear.x) < min_linear_speed)
      result.linear.x = 0;
    if (fabs(result.linear.y) < min_linear_speed)
      result.linear.y = 0;

    if (use_pot_fields && !move_lat)
      result.linear.y = rejection_force_y;

    return result;
  }

  geometry_msgs::msg::Twist calculate_speeds(float robot_angle,
                                             float goal_angle,
                                             float angular_speed, float beta) {
    float angle_error = goal_angle - robot_angle;
    if (angle_error > M_PI)
      angle_error -= 2 * M_PI;
    if (angle_error <= -M_PI)
      angle_error += 2 * M_PI;

    geometry_msgs::msg::Twist result;
    result.linear.x = 0;
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);

    return result;
  }

  // mvn_pln stamps the goal orientation onto the last path pose. The path
  // planner zeroes every orientation it writes, and an all-zero quaternion is
  // not a rotation, so its norm tells a requested yaw from an absent one.
  void read_path_goal_orientation() {
    path_goal_t_valid_ = false;
    if (goal_path_.poses.empty())
      return;

    const auto &q = goal_path_.poses.back().pose.orientation;
    if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-6)
      return;

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    path_goal_t_ = static_cast<float>(yaw);
    path_goal_t_valid_ = true;
  }

  static float wrap_to_pi(float angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle <= -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  // Holonomic path tracking. The base translates straight at the lookahead
  // point whatever its yaw is, so the exp(-angle_error^2/alpha) attenuation of
  // the unicycle controller -- which is what makes the robot turn its body
  // before it can move -- is gone. Heading the camera is left to the head pan,
  // which get_next_goal_head_angles() already commands as this same angle.
  geometry_msgs::msg::Twist
  calculate_speeds_omni(float robot_x, float robot_y, float robot_t,
                        float goal_x, float goal_y, float min_linear_speed,
                        float linear_speed, float max_angular_speed,
                        bool use_pot_fields, double rejection_force_x,
                        double rejection_force_y) {
    float heading =
        wrap_to_pi(atan2(goal_y - robot_y, goal_x - robot_x) - robot_t);

    float vx = linear_speed * cos(heading);
    float vy = linear_speed * sin(heading);

    // The deadzone is on the magnitude, not per axis: a diagonal command whose
    // two components are each below min_linear_speed is still a valid move.
    if (sqrt(vx * vx + vy * vy) < min_linear_speed) {
      vx = 0;
      vy = 0;
    } else if (use_pot_fields) {
      // Both axes are free here, so the rejection force adds to the tracking
      // velocity rather than overwriting the lateral one as in the unicycle
      // controller. It is only applied while translating, which keeps the
      // "rotating in place" case of collision_blocks_tracking() exact.
      vx += rejection_force_x;
      vy += rejection_force_y;
      float mag = sqrt(vx * vx + vy * vy);
      if (mag > max_linear_speed_) {
        vx *= max_linear_speed_ / mag;
        vy *= max_linear_speed_ / mag;
      }
    }

    geometry_msgs::msg::Twist result;
    result.linear.x = vx;
    result.linear.y = vy;
    result.angular.z = omni_yaw_speed(heading, max_angular_speed);
    return result;
  }

  float clamp_yaw(float w, float max_angular_speed) const {
    return std::max(-max_angular_speed, std::min(max_angular_speed, w));
  }

  // The base turns hard onto the goal yaw during the last-mile omni. The
  // camera is aimed at the goal yaw here (see publish_path_head_goal), not at
  // the path, so there is no pan range to protect: the whole rotation goes
  // toward the goal orientation. Without a goal orientation on the path there
  // is nothing to converge onto and the base simply yaws down the path.
  float omni_yaw_speed(float heading, float max_angular_speed) const {
    float yaw_error =
        path_goal_t_valid_ ? wrap_to_pi(path_goal_t_ - robot_t_) : heading;
    return clamp_yaw(yaw_correction_omni_gain_ * yaw_error, max_angular_speed);
  }

  // Last-mile omni: unicycle tracking has handed over to the holonomic
  // controller for the final stretch to the goal.
  bool last_mile_omni_active() const {
    if (!yaw_correction_omni_behavior_)
      return false;
    return std::hypot(global_goal_x_ - robot_x_, global_goal_y_ - robot_y_) <
           yaw_correction_omni_distance_;
  }

  // Obstacle interference (collision_risk abort + pot-fields rejection) is
  // suspended while the last-mile omni converges onto a goal that usually
  // sits right against furniture.
  bool obstacles_suspended() const {
    return yaw_correction_ignore_obstacles_ && last_mile_omni_active();
  }

  geometry_msgs::msg::Twist path_tracking_speeds() {
    if (last_mile_omni_active())
      return calculate_speeds_omni(robot_x_, robot_y_, robot_t_, goal_x_,
                                   goal_y_, min_linear_speed_,
                                   current_linear_speed, max_angular_speed_,
                                   use_pot_fields_ && !obstacles_suspended(),
                                   rejection_force_.x, rejection_force_.y);
    return calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_,
                            min_linear_speed_, current_linear_speed,
                            max_angular_speed_, alpha_, beta_, false, move_lat_,
                            use_pot_fields_, rejection_force_.y);
  }

  void reset_rotation_budget() {
    rotation_ticks_remaining_ =
        static_cast<int>(rotation_collision_max_time_ * RATE);
  }

  bool rotation_budget_remaining() const {
    return rotation_ticks_remaining_ > 0;
  }

  bool collision_blocks_tracking(geometry_msgs::msg::Twist &cmd) {
    if (!collision_risk_ || obstacles_suspended())
      return false;

    // Active recovery: rather than failing the path the moment the tracking
    // controller wants to translate into an obstacle, turn toward the next
    // path goal first. get_next_goal_from_path() has already run this tick, so
    // goal_x_/goal_y_ is that next goal. Only worth doing when the heading
    // error is large enough that turning actually changes the situation.
    // Never during the omni last mile, which is converging on a goal that
    // normally sits right against furniture and owns the yaw itself.
    if (allow_rotation_on_collision_ && !last_mile_omni_active() &&
        rotation_budget_remaining()) {
      const float angle_error =
          wrap_to_pi(atan2(goal_y_ - robot_y_, goal_x_ - robot_x_) - robot_t_);
      if (std::fabs(angle_error) > rotation_collision_angle_threshold_) {
        rotation_ticks_remaining_--;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z =
            clamp_yaw(std::copysign(rotation_collision_angular_speed_,
                                    angle_error),
                      max_angular_speed_);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SimpleMove.-> Collision risk: rotating in place to recover. "
            "heading_error=%.2f, budget=%.1f s",
            angle_error, rotation_ticks_remaining_ / float(RATE));
        return false;
      }
    }

    const bool translating =
        last_mile_omni_active()
            ? (cmd.linear.x != 0.0 || cmd.linear.y != 0.0)
            : (move_lat_ ? (cmd.linear.y != 0.0) : (cmd.linear.x != 0.0));
    if (translating)
      return true;

    // Rotating in place: drop any lateral pot-fields push, keep angular.z.
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    return false;
  }

  bool get_robot_position_wrt_map() {
    try {
      geometry_msgs::msg::TransformStamped transformStamped =
          tf_buffer_.lookupTransform("map", base_link_name_,
                                     tf2::TimePointZero);

      robot_x_ = transformStamped.transform.translation.x;
      robot_y_ = transformStamped.transform.translation.y;

      tf2::Quaternion q(transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_t_ = static_cast<float>(yaw);

      return true;
    } catch (const tf2::TransformException &ex) {
      // Every caller of these getters discards the bool, so on failure
      // robot_x_/robot_y_/robot_t_ silently keep their previous values (0,0,0
      // before the first successful lookup) and the controller steers from a
      // phantom pose. Throttled so a 30 Hz loop cannot flood the log.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SimpleMove.-> TF lookup failed, reusing last known "
                           "robot pose: %s",
                           ex.what());
      return false;
    }
  }

  bool get_robot_position_wrt_odom() {
    try {
      geometry_msgs::msg::TransformStamped transformStamped =
          tf_buffer_.lookupTransform(odom_name_, base_link_name_,
                                     tf2::TimePointZero);

      robot_x_ = transformStamped.transform.translation.x;
      robot_y_ = transformStamped.transform.translation.y;

      tf2::Quaternion q(transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_t_ = static_cast<float>(yaw);

      return true;
    } catch (const tf2::TransformException &ex) {
      // Every caller of these getters discards the bool, so on failure
      // robot_x_/robot_y_/robot_t_ silently keep their previous values (0,0,0
      // before the first successful lookup) and the controller steers from a
      // phantom pose. Throttled so a 30 Hz loop cannot flood the log.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SimpleMove.-> TF lookup failed, reusing last known "
                           "robot pose: %s",
                           ex.what());
      return false;
    }
  }

  bool get_goal_position_wrt_odom() {
    try {
      geometry_msgs::msg::TransformStamped transformStamped =
          tf_buffer_.lookupTransform(odom_name_, base_link_name_,
                                     tf2::TimePointZero);

      float robot_x = transformStamped.transform.translation.x;
      float robot_y = transformStamped.transform.translation.y;

      tf2::Quaternion q(transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      float robot_t = static_cast<float>(yaw);

      goal_t_ = robot_t + goal_angle_;
      if (goal_t_ > M_PI)
        goal_t_ -= 2 * M_PI;
      if (goal_t_ <= -M_PI)
        goal_t_ += 2 * M_PI;

      goal_x_ = robot_x + goal_distance_ * cos(robot_t + goal_angle_ +
                                               (move_lat_ ? M_PI / 2 : 0));
      goal_y_ = robot_y + goal_distance_ * sin(robot_t + goal_angle_ +
                                               (move_lat_ ? M_PI / 2 : 0));

      return true;
    } catch (const tf2::TransformException &ex) {
      // Every caller of these getters discards the bool, so on failure
      // robot_x_/robot_y_/robot_t_ silently keep their previous values (0,0,0
      // before the first successful lookup) and the controller steers from a
      // phantom pose. Throttled so a 30 Hz loop cannot flood the log.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SimpleMove.-> TF lookup failed, reusing last known "
                           "robot pose: %s",
                           ex.what());
      return false;
    }
  }

  float get_path_total_distance(nav_msgs::msg::Path &path) {
    float dist = 0;
    for (size_t i = 1; i < path.poses.size(); i++)
      dist += sqrt(
          pow(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
              2) +
          pow(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y,
              2));
    return dist;
  }

  void get_next_goal_from_path(int &last_pose_idx, int &next_pose_idx) {
    if (next_pose_idx >= goal_path_.poses.size())
      next_pose_idx = goal_path_.poses.size() - 1;
    float prev_goal_x = goal_path_.poses[last_pose_idx].pose.position.x;
    float prev_goal_y = goal_path_.poses[last_pose_idx].pose.position.y;
    float robot_error_x = robot_x_ - prev_goal_x;
    float robot_error_y = robot_y_ - prev_goal_y;
    float error = 0;
    do {
      goal_x_ = goal_path_.poses[next_pose_idx].pose.position.x;
      goal_y_ = goal_path_.poses[next_pose_idx].pose.position.y;
      float path_vector_x = goal_x_ - prev_goal_x;
      float path_vector_y = goal_y_ - prev_goal_y;
      float path_vector_mag =
          sqrt(path_vector_x * path_vector_x + path_vector_y * path_vector_y);
      path_vector_x = path_vector_mag > 0 ? path_vector_x / path_vector_mag : 0;
      path_vector_y = path_vector_mag > 0 ? path_vector_y / path_vector_mag : 0;
      float scalar_projection =
          robot_error_x * path_vector_x + robot_error_y * path_vector_y;
      // Error is not euclidean distance, but it is measured projected on the
      // current path segment
      error = path_vector_mag - scalar_projection;
      if (error < 0.25)
        last_pose_idx = next_pose_idx;
    } while (error < 0.25 && ++next_pose_idx < goal_path_.poses.size());
  }

  std_msgs::msg::Float32MultiArray
  get_next_goal_head_angles(int next_pose_idx) {
    std_msgs::msg::Float32MultiArray msg;

    int last = static_cast<int>(goal_path_.poses.size()) - 1;
    if (last < 0)
      return msg; // empty path guard

    int idx = std::min(next_pose_idx + head_lookahead_points_, last);
    float goal_x = goal_path_.poses[idx].pose.position.x;
    float goal_y = goal_path_.poses[idx].pose.position.y;
    float angle = atan2(goal_y - robot_y_, goal_x - robot_x_) - robot_t_;
    if (angle > M_PI)
      angle -= 2 * M_PI;
    if (angle <= -M_PI)
      angle += 2 * M_PI;
    msg.data.push_back(angle);
    msg.data.push_back(-1.0);
    return msg;
  }

  // Head target during path following. When yaw_correction_align_head is set,
  // the last-mile omni turns the camera to where the robot will end up facing
  // (the goal yaw) as the base yaws onto it, instead of chasing the last few
  // path points; otherwise the head keeps its normal path tracking.
  void publish_path_head_goal(int idx) {
    if (yaw_correction_align_head_ && last_mile_omni_active() &&
        path_goal_t_valid_) {
      std_msgs::msg::Float32MultiArray msg;
      msg.data.push_back(wrap_to_pi(path_goal_t_ - robot_t_));
      msg.data.push_back(-1.0);
      pub_head_goal_pose_->publish(msg);
      return;
    }
    pub_head_goal_pose_->publish(get_next_goal_head_angles(idx));
  }

  // ############
  // Simple Move main processing
  void simple_move_processing() {
    try {
      if (stop_) {
        stop_ = false;
        state = SM_INIT;
        collision_risk_ = false;
        new_rel_pose_ = false;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        pub_goal_reached_->publish(msg_goal_reached);
      }
      if (new_pose_ || new_path_ || new_rel_pose_)
        state = SM_WAITING_FOR_TASK;

      switch (state) {
      case SM_INIT:
        std::cout
            << "SimpleMove.-> Low level control ready. Waiting for new goal. "
            << std::endl;
        collision_risk_ = false;
        state = SM_WAITING_FOR_TASK;
        break;

      case SM_WAITING_FOR_TASK:
        if (new_pose_) {
          collision_risk_ = false;
          get_goal_position_wrt_odom();
          state = SM_GOAL_POSE_ACCEL;
          new_pose_ = false;
          msg_goal_reached.goal_id.id = "-1";
          attempts =
              (int)((fabs(goal_distance_) * 5) / max_linear_speed_ * RATE +
                    fabs(goal_angle_ * 5) / max_angular_speed_ * RATE +
                    RATE * 5);
        }
        if (new_path_) {
          collision_risk_ = false;
          state =
              use_constant_speed_ ? SM_GOAL_PATH_CONSTANT : SM_GOAL_PATH_ACCEL;
          new_path_ = false;
          read_path_goal_orientation();
          reset_rotation_budget();
          prev_pose_idx = 0;
          next_pose_idx = 0;
          global_goal_x_ =
              goal_path_.poses[goal_path_.poses.size() - 1].pose.position.x;
          global_goal_y_ =
              goal_path_.poses[goal_path_.poses.size() - 1].pose.position.y;
          // Goal id echoed back to mvn_pln so it can tell which path this
          // SUCCEEDED/ABORTED belongs to. Keys on the whole stamp, not just
          // .sec: under simulated time the stamp starts at 0 and mvn_pln can
          // republish several paths within one simulated second, so a .sec-only
          // id would let a stale result be accepted for the current path.
          // mvn_pln builds the same string in mvn_pln_node.cpp (search for
          // simple_move_sequencer) - keep them in sync.
          msg_goal_reached.goal_id.id =
              std::to_string(goal_path_.header.stamp.sec) + "." +
              std::to_string(goal_path_.header.stamp.nanosec);
          // Timeout is sized from the speed actually used, otherwise a slow
          // constant_speed would run out of attempts before reaching the goal.
          float plan_speed = use_constant_speed_ && constant_speed_ > 0
                                 ? constant_speed_
                                 : max_linear_speed_;
          attempts = (int)(get_path_total_distance(goal_path_) / plan_speed *
                               4 * RATE +
                           5 * RATE);
        }
        if (new_rel_pose_) {
          collision_risk_ = false;
          new_rel_pose_ = false;

          get_robot_position_wrt_odom();
          goal_x_ = robot_x_ + rel_x_ * cos(robot_t_) - rel_y_ * sin(robot_t_);
          goal_y_ = robot_y_ + rel_x_ * sin(robot_t_) + rel_y_ * cos(robot_t_);
          goal_t_ = robot_t_ + rel_yaw_;
          if (goal_t_ > M_PI)
            goal_t_ -= 2 * M_PI;
          if (goal_t_ <= -M_PI)
            goal_t_ += 2 * M_PI;
          state = SM_GOAL_REL_POSE;
          msg_goal_reached.goal_id.id = "-1";
          attempts =
              (int)((hypot(rel_x_, rel_y_) * 5) / max_linear_speed_ * RATE +
                    fabs(rel_yaw_ * 5) / max_angular_speed_ * RATE + RATE * 5);
        }
        break;

      case SM_GOAL_REL_POSE: { // TODO its moves only holonomic robot
        get_robot_position_wrt_odom();
        float dx = goal_x_ - robot_x_;
        float dy = goal_y_ - robot_y_;
        float dist = sqrt(dx * dx + dy * dy);
        float yaw_err = goal_t_ - robot_t_;
        if (yaw_err > M_PI)
          yaw_err -= 2 * M_PI;
        if (yaw_err <= -M_PI)
          yaw_err += 2 * M_PI;

        if (dist < fine_dist_tolerance_ && fabs(yaw_err) < angle_tolerance_) {
          std::cout << "SimpleMove.-> Relative move finished." << std::endl;
          state = SM_INIT;
          current_linear_speed = 0;
          msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
          pub_goal_reached_->publish(msg_goal_reached);
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          break;
        }
        if (--attempts <= 0) {
          std::cout << "SimpleMove.-> Timeout on relative move." << std::endl;
          state = SM_INIT;
          current_linear_speed = 0;
          msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
          pub_goal_reached_->publish(msg_goal_reached);
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          break;
        }

        float ex = dx * cos(robot_t_) + dy * sin(robot_t_);
        float ey = -dx * sin(robot_t_) + dy * cos(robot_t_);
        geometry_msgs::msg::Twist cmd;
        const float kp_lin = 2.0f, kp_ang = 2.0f;
        float vx = kp_lin * ex, vy = kp_lin * ey;
        float vmag = sqrt(vx * vx + vy * vy);
        if (vmag > max_linear_speed_) {
          vx *= max_linear_speed_ / vmag;
          vy *= max_linear_speed_ / vmag;
        }
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z =
            std::max(-max_angular_speed_,
                     std::min(max_angular_speed_, kp_ang * yaw_err));
        pub_cmd_vel_->publish(cmd);
        break;
      }

      case SM_GOAL_POSE_ACCEL:
        get_robot_position_wrt_odom();
        global_error = sqrt((goal_x_ - robot_x_) * (goal_x_ - robot_x_) +
                            (goal_y_ - robot_y_) * (goal_y_ - robot_y_));

        if (global_error < fine_dist_tolerance_)
          state = SM_GOAL_POSE_CORRECT_ANGLE;
        else if (global_error < current_linear_speed * current_linear_speed /
                                    (linear_acceleration_ * 5)) {
          state = SM_GOAL_POSE_DECCEL;
          temp_k = current_linear_speed / sqrt(global_error);
        } else if (current_linear_speed >= max_linear_speed_) {
          state = SM_GOAL_POSE_CRUISE;
          current_linear_speed = max_linear_speed_;
        }
        if (--attempts <= 0) {
          state = SM_GOAL_POSE_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal position. Current state: GOAL_POSE_ACCEL."
                    << std::endl;
        }
        // pub_cmd_vel_->publish(calculate_speeds(
        //     robot_x_, robot_y_, robot_t_, goal_x_, goal_y_,
        //     min_linear_speed_, current_linear_speed, max_angular_speed_,
        //     alpha_ * 2, beta_ / 4, goal_distance_ < 0, move_lat_));
        pub_cmd_vel_->publish(calculate_speeds(
            robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, min_linear_speed_,
            current_linear_speed, max_angular_speed_, alpha_ * 2, beta_ / 4,
            goal_distance_ < 0, move_lat_, use_pot_fields_,
            rejection_force_.y)); // use pot field
        current_linear_speed += (linear_acceleration_ * 5) / RATE;
        break;

      case SM_GOAL_POSE_CRUISE:
        get_robot_position_wrt_odom();
        global_error = sqrt((goal_x_ - robot_x_) * (goal_x_ - robot_x_) +
                            (goal_y_ - robot_y_) * (goal_y_ - robot_y_));
        if (global_error < fine_dist_tolerance_)
          state = SM_GOAL_POSE_CORRECT_ANGLE;
        else if (global_error < current_linear_speed * current_linear_speed /
                                    (linear_acceleration_ * 5)) {
          state = SM_GOAL_POSE_DECCEL;
          temp_k = current_linear_speed / sqrt(global_error);
        }
        if (--attempts <= 0) {
          state = SM_GOAL_POSE_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal position. Current state: GOAL_POSE_CRUISE."
                    << std::endl;
        }
        // pub_cmd_vel_->publish(calculate_speeds(
        //     robot_x_, robot_y_, robot_t_, goal_x_, goal_y_,
        //     min_linear_speed_, current_linear_speed, max_angular_speed_,
        //     alpha_ * 2, beta_ / 4, goal_distance_ < 0, move_lat_));
        pub_cmd_vel_->publish(calculate_speeds(
            robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, min_linear_speed_,
            current_linear_speed, max_angular_speed_, alpha_ * 2, beta_ / 4,
            goal_distance_ < 0, move_lat_, use_pot_fields_,
            rejection_force_.y)); // use pot field
        break;

      case SM_GOAL_POSE_DECCEL:
        get_robot_position_wrt_odom();
        global_error = sqrt((goal_x_ - robot_x_) * (goal_x_ - robot_x_) +
                            (goal_y_ - robot_y_) * (goal_y_ - robot_y_));

        if (global_error < fine_dist_tolerance_)
          state = SM_GOAL_POSE_CORRECT_ANGLE;
        if (--attempts <= 0) {
          state = SM_GOAL_POSE_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal position. Current state: GOAL_POSE_DECCEL."
                    << std::endl;
        }
        current_linear_speed = temp_k * sqrt(global_error);
        if (current_linear_speed < min_linear_speed_)
          current_linear_speed = min_linear_speed_;
        pub_cmd_vel_->publish(calculate_speeds(
            robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, min_linear_speed_,
            current_linear_speed, max_angular_speed_, alpha_ * 2, beta_ / 4,
            goal_distance_ < 0, move_lat_, use_pot_fields_,
            rejection_force_.y));
        break;

      case SM_GOAL_POSE_CORRECT_ANGLE:
        get_robot_position_wrt_odom();
        global_error = (goal_t_ - robot_t_);
        if (global_error > M_PI)
          global_error -= 2 * M_PI;
        if (global_error <= -M_PI)
          global_error += 2 * M_PI;
        global_error = fabs(global_error);
        if (global_error < angle_tolerance_)
          state = SM_GOAL_POSE_FINISH;
        pub_cmd_vel_->publish(
            calculate_speeds(robot_t_, goal_t_, max_angular_speed_, beta_ / 4));
        if (--attempts <= 0) {
          state = SM_GOAL_POSE_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal position. Current state: GOAL_POSE_CORRECT_ANGLE."
                    << std::endl;
        }
        break;

      case SM_GOAL_POSE_FINISH:
        std::cout << "SimpleMove.-> Successful move with dist="
                  << goal_distance_ << " angle=" << goal_angle_ << std::endl;
        state = SM_INIT;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
        pub_goal_reached_->publish(msg_goal_reached);
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        current_linear_speed = 0;
        break;

      case SM_GOAL_POSE_FAILED:
        std::cout << "SimpleMove.-> FAILED move with dist=" << goal_distance_
                  << " angle=" << goal_angle_ << std::endl;
        state = SM_INIT;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
        pub_goal_reached_->publish(msg_goal_reached);
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        current_linear_speed = 0;
        break;

      case SM_GOAL_PATH_ACCEL: {
        get_robot_position_wrt_map();
        get_next_goal_from_path(prev_pose_idx, next_pose_idx);
        auto cmd = path_tracking_speeds();

        if (collision_blocks_tracking(cmd)) {
          state = SM_GOAL_PATH_FAILED;
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!"
                    << std::endl;
          break;
        }

        global_error =
            sqrt((global_goal_x_ - robot_x_) * (global_goal_x_ - robot_x_) +
                 (global_goal_y_ - robot_y_) * (global_goal_y_ - robot_y_));

        if (global_error < coarse_dist_tolerance_) {
          state = SM_GOAL_PATH_FINISH;
        } else if (global_error < current_linear_speed * current_linear_speed /
                                      linear_acceleration_) {
          state = SM_GOAL_PATH_DECCEL;
          temp_k = current_linear_speed / sqrt(global_error);
        } else if (current_linear_speed >= max_linear_speed_) {
          state = SM_GOAL_PATH_CRUISE;
          current_linear_speed = max_linear_speed_;
        }
        if (--attempts <= 0) {
          state = SM_GOAL_PATH_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal path. Current state: GOAL_PATH_ACCEL."
                    << std::endl;
        }
        pub_cmd_vel_->publish(cmd);
        if (move_head_)
          publish_path_head_goal(next_pose_idx);
        current_linear_speed += linear_acceleration_ / RATE;
        break;
      }

      case SM_GOAL_PATH_CRUISE: {
        get_robot_position_wrt_map();
        get_next_goal_from_path(prev_pose_idx, next_pose_idx);
        auto cmd = path_tracking_speeds();

        if (collision_blocks_tracking(cmd)) {
          state = SM_GOAL_PATH_FAILED;
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!"
                    << std::endl;
          break;
        }

        global_error =
            sqrt((global_goal_x_ - robot_x_) * (global_goal_x_ - robot_x_) +
                 (global_goal_y_ - robot_y_) * (global_goal_y_ - robot_y_));
        if (global_error < coarse_dist_tolerance_)
          state = SM_GOAL_PATH_FINISH;
        else if (global_error < current_linear_speed * current_linear_speed /
                                    linear_acceleration_) {
          state = SM_GOAL_PATH_DECCEL;
          temp_k = current_linear_speed / sqrt(global_error);
        }
        if (--attempts <= 0) {
          state = SM_GOAL_PATH_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal path. Current state: GOAL_PATH_CRUISE."
                    << std::endl;
        }
        pub_cmd_vel_->publish(cmd);
        if (move_head_)
          publish_path_head_goal(next_pose_idx);
        break;
      }

      case SM_GOAL_PATH_DECCEL: {
        get_robot_position_wrt_map();
        get_next_goal_from_path(prev_pose_idx, next_pose_idx);
        global_error =
            sqrt((global_goal_x_ - robot_x_) * (global_goal_x_ - robot_x_) +
                 (global_goal_y_ - robot_y_) * (global_goal_y_ - robot_y_));

        // Speed ramp must be applied before the twist is built, since the gate
        // below inspects that twist.
        current_linear_speed = temp_k * sqrt(global_error);
        if (current_linear_speed < min_linear_speed_)
          current_linear_speed = min_linear_speed_;

        auto cmd = path_tracking_speeds();

        if (collision_blocks_tracking(cmd)) {
          state = SM_GOAL_PATH_FAILED;
          pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
          std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!"
                    << std::endl;
          break;
        }

        if (global_error < coarse_dist_tolerance_)
          state = SM_GOAL_PATH_FINISH;
        if (--attempts <= 0) {
          state = SM_GOAL_PATH_FAILED;
          std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                       "goal path. Current state:GOAL_PATH_DECCEL."
                    << std::endl;
        }
        pub_cmd_vel_->publish(cmd);
        if (move_head_)
          publish_path_head_goal(next_pose_idx);
        break;
      }

      case SM_GOAL_PATH_CONSTANT:
        // Constant-speed path following: like CRUISE but the speed is held at
        // constant_speed_ for the whole path (no accel/deccel ramp). The
        // controller still attenuates linear speed on sharp turns.
        {
          get_robot_position_wrt_map();
          get_next_goal_from_path(prev_pose_idx, next_pose_idx);
          current_linear_speed = constant_speed_;

          auto cmd = path_tracking_speeds();

          if (collision_blocks_tracking(cmd)) {
            state = SM_GOAL_PATH_FAILED;
            pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
            std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!"
                      << std::endl;
            break;
          }

          global_error =
              sqrt((global_goal_x_ - robot_x_) * (global_goal_x_ - robot_x_) +
                   (global_goal_y_ - robot_y_) * (global_goal_y_ - robot_y_));
          if (global_error < coarse_dist_tolerance_)
            state = SM_GOAL_PATH_FINISH;
          if (--attempts <= 0) {
            state = SM_GOAL_PATH_FAILED;
            std::cout << "SimpleMove.-> Timeout exceeded while trying to reach "
                         "goal path. Current state: GOAL_PATH_CONSTANT."
                      << std::endl;
          }
          pub_cmd_vel_->publish(cmd);
          if (move_head_)
            publish_path_head_goal(next_pose_idx);
        }
        break;

      case SM_GOAL_PATH_FINISH:
        std::cout << "SimpleMove.-> Path succesfully followed." << std::endl;
        state = SM_INIT;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
        pub_goal_reached_->publish(msg_goal_reached);
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        current_linear_speed = 0;
        break;

      case SM_GOAL_PATH_FAILED:
        std::cout << "SimpleMove.-> FAILED path traking." << std::endl;
        reset_rotation_budget();
        state = SM_INIT;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
        pub_goal_reached_->publish(msg_goal_reached);
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        current_linear_speed = 0;
        break;

      default:
        std::cout << "SimpleMove.-> A VERY STUPID PERSON PROGRAMMED THIS SHIT. "
                     "APOLOGIES FOR THE INCONVINIENCE :'("
                  << std::endl;
        return;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleMove.-> Error in SimpleMove processing: %s",
                   e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleMoveNode>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
