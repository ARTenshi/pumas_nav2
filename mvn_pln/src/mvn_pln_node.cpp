#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

// Message types
#include "actionlib_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "nav_msgs/srv/get_plan.hpp"
#include "pumas_interfaces/srv/get_plan_with_via.hpp"
#include "std_srvs/srv/trigger.hpp"

// TF2 (Transform listener)
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/utils.h>                             // for getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // for doTransform

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

// For motion_synthesizer action client // motion synthesis maintain by
// ry0hei-kobayashi
#include "pumas_interfaces/action/motion_synthesis.hpp"
#include "pumas_interfaces/msg/joints.hpp"
#include "pumas_interfaces/msg/start_and_end_joints.hpp"

#include "pumas_interfaces/action/pumas_nav.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

// Standard
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip> // for std::hex and std::setw
#include <iostream>
#include <sstream> // for std::stringstream
#include <vector>
///

// #define RATE 10

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_CHECK_IF_INSIDE_OBSTACLES 19
#define SM_WAITING_FOR_MOVE_BACKWARDS 18
#define SM_CHECK_IF_OBSTACLES 21
#define SM_WAIT_FOR_NO_OBSTACLES 22
#define SM_ENABLE_POT_FIELDS 23
#define SM_WAIT_FOR_POT_FIELDS 24
#define SM_WAIT_FOR_NOT_POT_FIELDS 124
#define SM_WAIT_FOR_CLEAR_MEMORY_RESPONSE 125
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_FINAL 17

// Collision-risk recovery states
#define SM_COLLISION_RECOVERY 40
#define SM_RECOVERY_WAIT_MAP 41
#define SM_RECOVERY_WAIT_BACKUP 42
#define SM_RECOVERY_WAIT_ROTATE 43

// Flags for clients WAIT casers
#define SM_WAIT_FOR_PATH_RESPONSE 102
#define SM_WAIT_FOR_INSIDE_OBSTACLES_RESPONSE 119
#define SM_WAIT_FOR_IF_OBSTACLES_RESPONSE 121
#define SM_WAIT_FOR_NO_OBSTACLES_RESPONSE 122

// motionsynthesis client by r.kobayashi
using MotionSynthesis = pumas_interfaces::action::MotionSynthesis;
using PumasNav = pumas_interfaces::action::PumasNav;
using GoalHandleMotionSynthesis =
    rclcpp_action::ClientGoalHandle<MotionSynthesis>;
using GoalHandlePumasNav = rclcpp_action::ServerGoalHandle<PumasNav>;

class MotionPlannerNode : public rclcpp::Node {
public:
  MotionPlannerNode()
      : Node("motion_planner_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // ############
    //  Declare parameters with default values
    this->declare_parameter("use_namespace", false);
    this->declare_parameter<bool>("patience", true);
    this->declare_parameter<float>("proximity_criterion", 2.0f);
    this->declare_parameter<std::string>("base_link_name", "base_footprint");
    this->declare_parameter<bool>("memory_all_obstacles", false);

    // If the planned path ends farther than this [m] from the requested goal,
    // treat it as a goal relocation (the goal was enclosed/unreachable and the
    // planner returned the nearest reachable point) and report it accordingly.
    this->declare_parameter<double>("goal_relocation_report_threshold", 0.30);
    this->declare_parameter<int>("collision_recovery_trigger_count", 3);

    // Online navigation must NEVER drop the point cloud during recovery (an
    // unknown -1 cell is treated as navigable online, so the cloud is the only
    // thing keeping the robot off real obstacles). The cloud-off last-resort
    // recovery tier is therefore enabled only when use_online is false.
    this->declare_parameter<bool>("use_online", false);

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("patience", patience_);
    this->get_parameter("proximity_criterion", proximity_criterion_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("memory_all_obstacles", memory_all_obstacles_);
    this->get_parameter("goal_relocation_report_threshold",
                        goal_relocation_report_threshold_);
    this->get_parameter("collision_recovery_trigger_count",
                        collision_recovery_trigger_count_);
    this->get_parameter("use_online", use_online_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &MotionPlannerNode::on_parameter_change, this, std::placeholders::_1));

    // Initialize service clients (non-blocking)
    init_service_clients();

    // ############
    //  Publishers
    //
    pub_pot_fields_enable_ = this->create_publisher<std_msgs::msg::Bool>(
        make_name("/navigation/potential_fields/enable"),
        rclcpp::QoS(10).transient_local());
    pub_pot_fields_enable_cloud_ = this->create_publisher<std_msgs::msg::Bool>(
        make_name("/navigation/potential_fields/enable_cloud"),
        rclcpp::QoS(10).transient_local());
    pub_map_aug_enable_ = this->create_publisher<std_msgs::msg::Bool>(
        make_name("/navigation/map_augmenter/enable"),
        rclcpp::QoS(10).transient_local());
    pub_map_aug_enable_cloud_ = this->create_publisher<std_msgs::msg::Bool>(
        make_name("/navigation/map_augmenter/enable_cloud"),
        rclcpp::QoS(10).transient_local());

    pub_goal_path_ = this->create_publisher<nav_msgs::msg::Path>(
        make_name("/simple_move/goal_path"), rclcpp::QoS(10).transient_local());
    pub_goal_dist_angle_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_dist_angle"),
            rclcpp::QoS(10).transient_local());
    // Holonomic relative move [x, y, yaw] (base frame) for the recovery escape.
    pub_goal_rel_pose_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_rel_pose"),
            rclcpp::QoS(10).reliable());
    pub_status_ = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
        make_name("/navigation/status"), rclcpp::QoS(10).transient_local());
    pub_simple_move_stop_ = this->create_publisher<std_msgs::msg::Empty>(
        make_name("/simple_move/stop"), rclcpp::QoS(10).transient_local());
    pub_detection_scale_ = this->create_publisher<std_msgs::msg::Float32>(
        make_name("/navigation/potential_fields/detection_scale"),
        rclcpp::QoS(10).transient_local());

    // ############
    //  Subscribers
    //// stop
    sub_general_stop_ = this->create_subscription<std_msgs::msg::Empty>(
        make_name("/stop"), rclcpp::SensorDataQoS(),
        std::bind(&MotionPlannerNode::callback_general_stop, this,
                  std::placeholders::_1));

    sub_nav_ctrl_stop_ = this->create_subscription<std_msgs::msg::Empty>(
        make_name("/navigation/stop"), rclcpp::SensorDataQoS(),
        std::bind(&MotionPlannerNode::callback_navigation_stop, this,
                  std::placeholders::_1));

    sub_move_goal_status_ =
        this->create_subscription<actionlib_msgs::msg::GoalStatus>(
            make_name("/simple_move/goal_reached"), rclcpp::SensorDataQoS(),
            std::bind(&MotionPlannerNode::callback_simple_move_goal_status,
                      this, std::placeholders::_1));

    //// motion
    sub_set_patience_ = this->create_subscription<std_msgs::msg::Bool>(
        make_name("/navigation/set_patience"), rclcpp::SensorDataQoS(),
        std::bind(&MotionPlannerNode::callback_set_patience, this,
                  std::placeholders::_1));

    sub_collision_risk_ = this->create_subscription<std_msgs::msg::Bool>(
        make_name("/navigation/potential_fields/collision_risk"),
        rclcpp::SensorDataQoS(),
        std::bind(&MotionPlannerNode::callback_collision_risk, this,
                  std::placeholders::_1));

    //// motion synth
    sub_arm_goal_ =
        this->create_subscription<pumas_interfaces::msg::StartAndEndJoints>(
            make_name("/motion_synth/joint_pose"), rclcpp::SensorDataQoS(),
            std::bind(&MotionPlannerNode::callback_arm_joints, this,
                      std::placeholders::_1));

    motion_synth_client_ =
        rclcpp_action::create_client<MotionSynthesis>(this, "/motion_synth");
    if (!motion_synth_client_->wait_for_action_server(
            std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> MotionSynth action server not available.");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "MotionPlanner.-> MotionSynth action server is ready.");
    }

    // pumas nav action
    nav_action_server_ = rclcpp_action::create_server<PumasNav>(
        this, "/pumasnav",
        std::bind(&MotionPlannerNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&MotionPlannerNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&MotionPlannerNode::handle_accepted, this,
                  std::placeholders::_1));

    // ############
    //  Wait for transforms
    wait_for_transforms("map", base_link_name_);

    // Simple Move main processing
    processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&MotionPlannerNode::motion_planner_processing, this));

    RCLCPP_INFO(this->get_logger(),
                "MotionPlanner.-> MotionPlannerNode is ready.");
  }

private:
  // ############
  //  State variables
  bool use_namespace_ = false;
  bool stop_ = false;
  bool collision_risk_ = false;
  bool new_global_goal_ = false;
  int simple_move_status_id_ = 0;

  geometry_msgs::msg::Pose global_goal_;
  // Ordered via points for the current goal (empty -> plain A*). Carried in the
  // PumasNav goal; progressively popped as the robot passes each one.
  std::vector<geometry_msgs::msg::Pose> via_points_;
  // Per-goal stop-short distance [m]. When > 0, halt + succeed as soon as
  // straight-line dist(robot, goal) < goal_distance_. <= 0 = full arrival.
  float goal_distance_ = 0.0f;
  actionlib_msgs::msg::GoalStatus simple_move_goal_status_;

  float robot_x_ = 0.0f;
  float robot_y_ = 0.0f;
  float robot_t_ = 0.0f;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Path path_;
  bool is_path_ = false;
  bool is_path_response_ = false;

  // Internal parameter values
  bool patience_;
  float proximity_criterion_;
  double goal_relocation_report_threshold_ = 0.30;
  std::string base_link_name_;

  // Motion Planner processing variables
  float error = 0.0f;

  int state = SM_INIT;
  int simple_move_sequencer = -1;
  int goal_id = -1;
  int current_status = 0;

  bool near_goal_sent = false;

  // Set when the planner returned a path to the nearest reachable cell because
  // the requested goal was enclosed/unreachable (see SM_FINAL reporting).
  bool goal_relocated_ = false;
  double goal_relocated_dist_ = 0.0;

  // ROS 2 message types
  std_msgs::msg::Bool msg_bool;
  std_msgs::msg::Float32MultiArray msg_goal_dist_angle;

  // Flags for waiting clients in Switch/Case
  bool is_in_obstacles_ = false;
  bool is_in_obstacles_response_ = false;
  bool are_there_obs_ = false;
  bool is_check_obs_response_ = false;
  bool are_still_obs_ = false;
  bool is_wait_obs_response_ = false;
  bool wait_obs_failed_ = false;

  bool pot_fields_received_ = false;
  bool is_pot_fields_response_ = false;

  // motion_synth
  bool arm_goal_received = false;
  pumas_interfaces::msg::StartAndEndJoints target_arm_pose;

  rclcpp::Time no_cloud_pot_fields_start_time_;
  rclcpp::Duration no_cloud_pot_fields_duration_{2, 0}; // 2 seconds
  bool is_temporary_no_cloud_pot_fields_ = false;

  // ############
  //  Collision-risk recovery
  //  Trigger: count collision-risk path aborts; once >= trigger_count_, run
  //  active recovery. Tier 0 = risk-aware escape move: a 15 cm holonomic move
  //  AWAY from the obstacle (back / left / right / diagonal, chosen from the
  //  obstacle's lateral offset on the augmented map, clearance-checked); tier
  //  >= 1 = progressively shrink the cloud detection box (lidar box is left
  //  full as the reliable collision_risk trigger); the final tier
  //  (recovery_max_shrink_steps_ + 1) is the last resort: temporarily disable
  //  the point cloud entirely (lidar only) when even the floored cloud box
  //  cannot get through. Counters reset on recovery escape (forward progress
  //  past the stuck spot), NOT at the start->goal boundary. Never gives up:
  //  after the cloud-off tier the level wraps back to 0 to re-attempt an escape
  //  move, with the cloud kept off until reset_recovery_state().
  int collision_recovery_trigger_count_ = 3; // ROS param
  bool use_online_ = false; // ROS param; gates the cloud-off recovery tier
  int collision_event_count_ = 0;
  int recovery_level_ = 0;
  bool collision_risk_latched_ =
      false; // a collision-risk True seen while moving
  bool recovery_stuck_recorded_ = false;
  // Distance-to-goal captured when first stuck. Escape = the robot has closed
  // this by recovery_progress_dist_. Measured toward the goal (not Euclidean
  // from the stuck spot) so a back-up, which moves away from the goal, never
  // counts as escaping.
  float stuck_goal_dist_ = 0.0f;
  float detection_scale_ = 1.0f;
  // Last-resort recovery: the point cloud has been temporarily disabled (via
  // both /navigation/potential_fields/enable_cloud and
  // /navigation/map_augmenter/enable_cloud, so the reactive and planning layers
  // drop the camera in lockstep) because even the floored cloud box could not
  // get through. Latched until reset_recovery_state() (recovery escape / goal
  // reached / new task). The lidar box stays full throughout.
  bool recovery_cloud_disabled_ = false;
  // Async augmented-map fetch used for the rear-clearance check.
  nav_msgs::msg::OccupancyGrid recovery_map_;
  bool recovery_map_ready_ = false;
  bool recovery_map_response_ = false;
  // Fixed tuning (edit in code; kept out of ROS params on purpose).
  const float recovery_back_dist_ = 0.15f; // back-up distance [m]
  const float recovery_progress_dist_ =
      0.50f;                                  // advance to count as escaped [m]
  const float recovery_shrink_factor_ = 0.8f; // detection scale mult / step
  const float recovery_min_detection_scale_ = 0.50f; // detection scale floor
  const float recovery_rear_lateral_ = 0.15f; // rear-check lateral offset [m]
  const int recovery_max_shrink_steps_ = 3;   // shrink steps before wrapping
  // ONLINE sensor-safe escape: when no translational escape is clear (the robot
  // is boxed in on all sides, typically by a directional point-cloud phantom),
  // rotate IN PLACE. The base center does not move, so the rotation cannot drive
  // the robot into an obstacle (collision-safe), and it reorients the head
  // camera, which sheds the directional phantom. Rotation is accumulated; once a
  // full scan still finds no way through, the robot is genuinely enclosed, so we
  // stop at the nearest reachable point instead of looping forever.
  const float recovery_rotate_step_ =
      0.7853982f;                              // in-place rotation / step [rad] (~45 deg)
  const float recovery_rotate_max_ = 2.0f * M_PI; // total rotation before giving up [rad]
  float recovery_rotated_total_ = 0.0f;

  // Memory obstacle clear flow (used when memory_all_obstacles is enabled and
  // path planning has just failed — we wipe accumulated memory then retry)
  bool memory_all_obstacles_ = false;
  bool clear_memory_request_sent_ = false;
  bool clear_memory_response_received_ = false;
  bool clear_memory_success_ = false;
  bool clear_memory_failed_ = false;

  rclcpp::Time pot_fields_start_time_;

  // ############
  //  Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pot_fields_enable_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      pub_pot_fields_enable_cloud_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_map_aug_enable_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_map_aug_enable_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_goal_path_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      pub_goal_dist_angle_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      pub_goal_rel_pose_;
  rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_simple_move_stop_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_update_maps_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_detection_scale_;

  // ############
  //  Subscribers
  //// stop
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_general_stop_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_nav_ctrl_stop_;

  //// goal
  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr
      sub_move_goal_status_;

  //// motion
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_set_patience_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collision_risk_;

  //// motion synth
  rclcpp::Subscription<pumas_interfaces::msg::StartAndEndJoints>::SharedPtr
      sub_arm_goal_;
  rclcpp_action::Client<MotionSynthesis>::SharedPtr motion_synth_client_;
  std::shared_ptr<GoalHandleMotionSynthesis> motion_synth_goal_handle_;

  // nav action
  rclcpp_action::Server<PumasNav>::SharedPtr nav_action_server_;
  std::shared_ptr<GoalHandlePumasNav> active_goal_handle_;
  bool action_active_ = false;
  bool cancel_requested_ = false;

  // ############
  //  Service clients
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr clt_plan_path_static_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr clt_plan_path_augmented_;
  rclcpp::Client<pumas_interfaces::srv::GetPlanWithVia>::SharedPtr
      clt_plan_path_with_via_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_aug_map_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_aug_costmap_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_are_there_obs_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_is_in_obstacles_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      clt_clear_memory_all_obstacles_;

  std::vector<std::thread> threads_;

  // ############
  //  Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Timer and readiness flag
  rclcpp::TimerBase::SharedPtr service_check_timer_;
  bool services_ready_ = false;

  // Used to wait for first message
  bool waiting_for_potential_fields_ = false;

  // Main processing loop
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // pumas action
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const PumasNav::Goal> goal) {
    (void)uuid;
    (void)goal;

    if (action_active_) {
      RCLCPP_WARN(this->get_logger(),
                  "MotionPlanner.-> Rejecting new goal because another "
                  "navigation task is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(),
                "MotionPlanner.-> Navigation action goal accepted request.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandlePumasNav> goal_handle) {
    (void)goal_handle;
    RCLCPP_WARN(this->get_logger(),
                "MotionPlanner.-> Cancel request received.");
    cancel_requested_ = true;
    stop_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ############
  //  Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
      if (param.get_name() == "use_namespace")
        use_namespace_ = param.as_bool();
      else if (param.get_name() == "patience")
        patience_ = param.as_bool();
      else if (param.get_name() == "proximity_criterion")
        proximity_criterion_ = param.as_double();
      else if (param.get_name() == "base_link_name")
        base_link_name_ = param.as_string();
      else if (param.get_name() == "memory_all_obstacles")
        memory_all_obstacles_ = param.as_bool();
      else if (param.get_name() == "goal_relocation_report_threshold")
        goal_relocation_report_threshold_ = param.as_double();
      else if (param.get_name() == "collision_recovery_trigger_count")
        collision_recovery_trigger_count_ = param.as_int();
      else if (param.get_name() == "use_online")
        use_online_ = param.as_bool();

      else {
        result.successful = false;
        result.reason =
            "MotionPlanner.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(
            this->get_logger(),
            "MotionPlanner.-> Attempted to update unsupported parameter: %s",
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
                "MotionPlanner.-> Waiting for transform from '%s' to '%s'...",
                source_frame.c_str(), target_frame.c_str());

    rclcpp::Time start_time = this->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0);

    bool transform_ok = false;

    while (rclcpp::ok() && (this->now() - start_time) < timeout) {
      try {
        tf_buffer_.lookupTransform(target_frame, source_frame,
                                   tf2::TimePointZero,
                                   tf2::durationFromSec(0.1));
        transform_ok = true;
        break;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "MotionPlanner.-> Still waiting for transform: %s",
                             ex.what());
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (!transform_ok) {
      RCLCPP_WARN(this->get_logger(),
                  "MotionPlanner.-> Timeout while waiting for transform from "
                  "'%s' to '%s'.",
                  source_frame.c_str(), target_frame.c_str());
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "MotionPlanner.-> Transform from '%s' to '%s' is now available.",
          source_frame.c_str(), target_frame.c_str());
    }
  }

  // ############
  //  Initialize service clients (non-blocking)
  void init_service_clients() {
    clt_plan_path_static_ = this->create_client<nav_msgs::srv::GetPlan>(
        make_name("/path_planner/plan_path_with_static"));
    clt_plan_path_augmented_ = this->create_client<nav_msgs::srv::GetPlan>(
        make_name("/path_planner/plan_path_with_augmented"));
    clt_plan_path_with_via_ =
        this->create_client<pumas_interfaces::srv::GetPlanWithVia>(
            make_name("/path_planner/plan_path_with_via"));
    clt_get_aug_map_ = this->create_client<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_augmented_map"));
    clt_get_aug_costmap_ = this->create_client<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_augmented_cost_map"));
    clt_are_there_obs_ = this->create_client<std_srvs::srv::Trigger>(
        make_name("/map_augmenter/are_there_obstacles"));
    clt_is_in_obstacles_ = this->create_client<std_srvs::srv::Trigger>(
        make_name("/map_augmenter/is_inside_obstacles"));
    clt_clear_memory_all_obstacles_ =
        this->create_client<std_srvs::srv::Trigger>(
            make_name("/map_augmenter/clear_memory_all_obstacles"));

    service_check_timer_ =
        this->create_wall_timer(std::chrono::seconds(1), [this]() {
          if (clt_plan_path_static_->wait_for_service(
                  std::chrono::seconds(0)) &&
              clt_plan_path_augmented_->wait_for_service(
                  std::chrono::seconds(0)) &&
              clt_get_aug_map_->wait_for_service(std::chrono::seconds(0)) &&
              clt_get_aug_costmap_->wait_for_service(std::chrono::seconds(0)) &&
              clt_are_there_obs_->wait_for_service(std::chrono::seconds(0)) &&
              clt_is_in_obstacles_->wait_for_service(std::chrono::seconds(0)) &&
              clt_clear_memory_all_obstacles_->wait_for_service(
                  std::chrono::seconds(0))) {
            RCLCPP_INFO(this->get_logger(),
                        "MotionPlanner.-> All motion planner clients are now "
                        "available.");
            services_ready_ = true;
            service_check_timer_->cancel();
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "MotionPlanner.-> Waiting for motion planner clients "
                        "to become available...");
          }
        });
  }

  // ############
  // Motion Planner callbacks
  void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    try {
      std::cout << "MotionPlanner.-> General Stop signal received" << std::endl;
      stop_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "MotionPlanner.-> Error processing callback_general_stop: %s",
          e.what());
    }
  }

  void callback_navigation_stop(const std_msgs::msg::Empty::SharedPtr msg) {
    try {
      std::cout << "MotionPlanner.-> Navigation Stop signal received"
                << std::endl;
      stop_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "MotionPlanner.-> Error processing callback_navigation_stop: %s",
          e.what());
    }
  }

  void callback_simple_move_goal_status(
      const actionlib_msgs::msg::GoalStatus::SharedPtr msg) {
    try {
      simple_move_goal_status_ = *msg;

      std::stringstream ss;
      ss << msg->goal_id.id;
      ss >> simple_move_status_id_;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> Error processing "
                   "callback_simple_move_goal_status: %s",
                   e.what());
    }
  }

  void callback_set_patience(const std_msgs::msg::Bool::SharedPtr msg) {
    try {
      std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False")
                << std::endl;
      patience_ = msg->data;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          this->get_logger(),
          "MotionPlanner.-> Error processing callback_set_patience: %s",
          e.what());
    }
  }

  void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg) {
    try {
      pot_fields_received_ = true;
      is_pot_fields_response_ = true;

      collision_risk_ = msg->data;
      // Latch a True so the collision-risk abort can be distinguished from a
      // generic (timeout) abort even after the robot stops and potential_fields
      // (speed-gated) goes back to False. Cleared at each path-follow start.
      if (msg->data)
        collision_risk_latched_ = true;
    } catch (const std::exception &e) {
      pot_fields_received_ = false;
      is_pot_fields_response_ = true;
      RCLCPP_ERROR(
          this->get_logger(),
          "MotionPlanner.-> Error processing callback_collision_risk: %s",
          e.what());
    }
  }

  void callback_arm_joints(
      const pumas_interfaces::msg::StartAndEndJoints::SharedPtr msg) {
    target_arm_pose = *msg;
    arm_goal_received = true;
  }

  // ############
  // Motion Planner additional functions

  // Compare the planned path endpoint with the requested goal. If they differ
  // by more than goal_relocation_report_threshold_, the planner relocated the
  // goal to the nearest reachable cell (the requested goal was enclosed /
  // unreachable). Records the flag + distance for SM_FINAL reporting.
  void detect_goal_relocation(float goal_x, float goal_y) {
    goal_relocated_ = false;
    goal_relocated_dist_ = 0.0;
    if (path_.poses.empty())
      return;
    const auto &end = path_.poses.back().pose.position;
    double d = std::hypot(end.x - goal_x, end.y - goal_y);
    if (d > goal_relocation_report_threshold_) {
      goal_relocated_ = true;
      goal_relocated_dist_ = d;
      RCLCPP_WARN(this->get_logger(),
                  "MotionPlanner.-> Goal appears relocated: path ends %.2f m "
                  "from the requested goal (enclosed/unreachable goal).",
                  d);
    }
  }

  void get_plan_path_from_augmented_map(float robot_x, float robot_y,
                                        float goal_x, float goal_y) {
    threads_.push_back(
        std::thread(std::bind(&MotionPlannerNode::plan_path_from_augmented_map,
                              this, robot_x, robot_y, goal_x, goal_y)));
  }

  void plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x,
                                    float goal_y) {
    is_path_ = false;
    is_path_response_ = false;
    goal_relocated_ = false;
    // Wait for service to be available
    if (!clt_plan_path_augmented_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> Service not available.");
      is_path_ = false;
      is_path_response_ = true;
      return;
    }

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start.pose.position.x = robot_x;
    request->start.pose.position.y = robot_y;
    request->start.header.frame_id = "map";
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, robot_t_);
      request->start.pose.orientation = tf2::toMsg(q);
    }

    request->goal.pose.position.x = goal_x;
    request->goal.pose.position.y = goal_y;
    request->goal.header.frame_id = "map";

    auto result_future = clt_plan_path_augmented_->async_send_request(request);

    try {
      this->path_ = result_future.get()->plan;
      if (path_.poses.size() > 0) {
        this->is_path_ = true;
        detect_goal_relocation(goal_x, goal_y);
      }

      this->is_path_response_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "MotionPlanner.-> Path received successfully with size: %zu",
                  path_.poses.size());
    } catch (const std::exception &e) {
      this->is_path_response_ = true;
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> Failed to get path from "
                   "/path_planner/plan_path_with_augmented: %s",
                   e.what());
    }
  }

  void get_plan_path_with_via(float robot_x, float robot_y, float goal_x,
                              float goal_y) {
    threads_.push_back(
        std::thread(std::bind(&MotionPlannerNode::plan_path_with_via, this,
                              robot_x, robot_y, goal_x, goal_y)));
  }

  void plan_path_with_via(float robot_x, float robot_y, float goal_x,
                          float goal_y) {
    is_path_ = false;
    is_path_response_ = false;
    goal_relocated_ = false;
    if (!clt_plan_path_with_via_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(
          this->get_logger(),
          "MotionPlanner.-> plan_path_with_via service not available.");
      is_path_ = false;
      is_path_response_ = true;
      return;
    }

    auto request =
        std::make_shared<pumas_interfaces::srv::GetPlanWithVia::Request>();
    request->start.pose.position.x = robot_x;
    request->start.pose.position.y = robot_y;
    request->start.header.frame_id = "map";
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, robot_t_);
      request->start.pose.orientation = tf2::toMsg(q);
    }

    request->goal.pose.position.x = goal_x;
    request->goal.pose.position.y = goal_y;
    request->goal.header.frame_id = "map";

    for (const auto &p : via_points_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose = p;
      request->via_points.push_back(ps);
    }

    auto result_future = clt_plan_path_with_via_->async_send_request(request);

    try {
      this->path_ = result_future.get()->plan;
      if (path_.poses.size() > 0) {
        this->is_path_ = true;
        detect_goal_relocation(goal_x, goal_y);
      }

      this->is_path_response_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "MotionPlanner.-> Path (via %zu) received successfully with "
                  "size: %zu",
                  request->via_points.size(), path_.poses.size());
    } catch (const std::exception &e) {
      this->is_path_response_ = true;
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> Failed to get path from "
                   "/path_planner/plan_path_with_via: %s",
                   e.what());
    }
  }

  // Pop via points the robot has already reached (within threshold) so a
  // replan does not route back to them.
  void update_via_points() {
    const double thr = 0.5; // [m]
    while (!via_points_.empty()) {
      double dx = via_points_.front().position.x - robot_x_;
      double dy = via_points_.front().position.y - robot_y_;
      if (std::sqrt(dx * dx + dy * dy) <= thr) {
        RCLCPP_INFO(this->get_logger(),
                    "MotionPlanner.-> Reached via point, %zu remaining",
                    via_points_.size() - 1);
        via_points_.erase(via_points_.begin());
      } else {
        break;
      }
    }
  }

  void get_robot_position() {
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

      return;
    } catch (const tf2::TransformException &ex) {
      // RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s",
      // ex.what());
      // robot_x_ = robot_y_ = robot_t_ = 0.0f;
      return;
    }
  }

  // ############
  //  Collision-recovery helpers
  void publish_detection_scale(float scale) {
    std_msgs::msg::Float32 m;
    m.data = scale;
    pub_detection_scale_->publish(m);
  }

  // Clear all recovery state and restore the cloud box. Called on recovery
  // escape (forward progress) and at goal boundaries.
  void reset_recovery_state() {
    collision_event_count_ = 0;
    recovery_level_ = 0;
    recovery_stuck_recorded_ = false;
    recovery_rotated_total_ = 0.0f;
    if (detection_scale_ != 1.0f) {
      detection_scale_ = 1.0f;
      publish_detection_scale(1.0f);
    }
    if (recovery_cloud_disabled_) {
      std_msgs::msg::Bool m;
      m.data = true;
      pub_pot_fields_enable_cloud_->publish(m);
      pub_map_aug_enable_cloud_->publish(m);
      recovery_cloud_disabled_ = false;
      std::cout << "MotionPlanner.-> Recovery reset: point cloud RE-ENABLED."
                << std::endl;
    }
  }

  // Step the recovery tier. OFFLINE (use_online_ == false): after the shrink
  // steps comes the cloud-off tier (recovery_max_shrink_steps_ + 1); past that
  // the level wraps to tier 0 (re-attempt an escape move) while leaving the
  // scale at its floor and the cloud disabled -> never gives up. ONLINE: the
  // ladder stops at the last shrink step and wraps there, so recovery_level_
  // never reaches the cloud-off tier and the cloud is never dropped.
  void advance_recovery_level() {
    recovery_level_++;
    const int max_level = use_online_ ? recovery_max_shrink_steps_
                                      : recovery_max_shrink_steps_ + 1;
    if (recovery_level_ > max_level)
      recovery_level_ = 0;
  }

  // Asynchronously fetch the augmented occupancy map for the rear check.
  void request_augmented_map() {
    recovery_map_response_ = false;
    recovery_map_ready_ = false;
    if (!clt_get_aug_map_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(),
                  "MotionPlanner.-> get_augmented_map service unavailable.");
      recovery_map_response_ = true; // no map -> rear treated as unknown
      return;
    }
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_aug_map_->async_send_request(
        request,
        [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
          try {
            recovery_map_ = future.get()->map;
            recovery_map_ready_ = true;
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "MotionPlanner.-> Failed to get augmented map: %s",
                         e.what());
            recovery_map_ready_ = false;
          }
          recovery_map_response_ = true;
        });
  }

  // Is the augmented-map cell at world (px,py) KNOWN-occupied (>0)? Fail-OPEN:
  // missing map, off-map and unknown (-1) all return false (not blocked), so a
  // short escape move is never deadlocked by the map being incomplete.
  bool map_cell_blocked(float px, float py) const {
    const auto &info = recovery_map_.info;
    if (info.resolution <= 0.0f || recovery_map_.data.empty())
      return false;
    const int cx =
        static_cast<int>((px - info.origin.position.x) / info.resolution);
    const int cy =
        static_cast<int>((py - info.origin.position.y) / info.resolution);
    if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) ||
        cy >= static_cast<int>(info.height))
      return false;
    const int cell = cy * static_cast<int>(info.width) + cx;
    if (cell < 0 || cell >= static_cast<int>(recovery_map_.data.size()))
      return false;
    return recovery_map_.data[cell] > 0;
  }

  // Is a relative move (rel_x fwd+, rel_y left+, base frame) clear? Checks the
  // destination plus ± footprint offsets perpendicular to the motion.
  bool dir_clear(float rel_x, float rel_y) const {
    const float ct = cos(robot_t_), st = sin(robot_t_);
    const float len = std::hypot(rel_x, rel_y);
    const float ux = (len > 1e-3f) ? rel_x / len : 0.0f;
    const float uy = (len > 1e-3f) ? rel_y / len : 0.0f;
    const float perpx = -uy, perpy = ux; // perpendicular to motion
    const float offs[3] = {-recovery_rear_lateral_, 0.0f,
                           recovery_rear_lateral_};
    for (float o : offs) {
      const float bx = rel_x + perpx * o;
      const float by = rel_y + perpy * o;
      const float px = robot_x_ + bx * ct - by * st;
      const float py = robot_y_ + bx * st + by * ct;
      if (map_cell_blocked(px, py))
        return false;
    }
    return true;
  }

  // Choose direction escape move (base frame [x fwd+, y left+])
  bool choose_escape_move(float &rel_x, float &rel_y) {
    const float ct = cos(robot_t_), st = sin(robot_t_);
    double sum_y = 0.0;
    int n = 0;
    for (float bx = 0.15f; bx <= 0.65f; bx += 0.05f) {
      for (float by = -0.45f; by <= 0.45f; by += 0.05f) {
        const float px = robot_x_ + bx * ct - by * st;
        const float py = robot_y_ + bx * st + by * ct;
        if (map_cell_blocked(px, py)) {
          sum_y += by;
          n++;
        }
      }
    }
    const float avg_y = (n > 0) ? static_cast<float>(sum_y / n) : 0.0f;

    const float D = recovery_back_dist_; // 15 cm
    const float Dd = D * 0.70710678f;    // diagonal component (|move| = D)
    const float lat_thresh = 0.10f;
    std::vector<std::array<float, 2>> cands;
    if (n > 0 && avg_y > lat_thresh) {
      // obstacle on the LEFT -> escape RIGHT first
      cands = {{{0, -D}}, {{-Dd, -Dd}}, {{-D, 0}}, {{-Dd, Dd}}, {{0, D}}};
    } else if (n > 0 && avg_y < -lat_thresh) {
      // obstacle on the RIGHT -> escape LEFT first
      cands = {{{0, D}}, {{-Dd, Dd}}, {{-D, 0}}, {{-Dd, -Dd}}, {{0, -D}}};
    } else {
      // centered / no obstacle found -> straight back first
      cands = {{{-D, 0}}, {{-Dd, Dd}}, {{-Dd, -Dd}}, {{0, D}}, {{0, -D}}};
    }
    for (const auto &m : cands) {
      if (dir_clear(m[0], m[1])) {
        rel_x = m[0];
        rel_y = m[1];
        return true;
      }
    }
    return false;
  }

  int publish_status(int status, int id, const std::string &text) {
    actionlib_msgs::msg::GoalStatus msg;
    std::stringstream ss;
    ss << id;
    ss >> msg.goal_id.id;
    msg.status = status;
    msg.text = text;
    msg.goal_id.stamp = rclcpp::Clock().now();

    pub_status_->publish(msg);

    return status;
  }

  void send_motion_synth_goal(
      const geometry_msgs::msg::Pose &nav_goal_pose,
      const pumas_interfaces::msg::StartAndEndJoints &arm_joints) {
    auto goal_msg = MotionSynthesis::Goal();
    goal_msg.goal_location.x = nav_goal_pose.position.x;
    goal_msg.goal_location.y = nav_goal_pose.position.y;

    tf2::Quaternion q(nav_goal_pose.orientation.x, nav_goal_pose.orientation.y,
                      nav_goal_pose.orientation.z, nav_goal_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal_msg.goal_location.theta = static_cast<float>(yaw);

    goal_msg.apply_start_pose = arm_joints.has_arm_start_pose;
    goal_msg.apply_goal_pose = arm_joints.has_arm_end_pose;
    goal_msg.start_pose = arm_joints.start_pose;
    goal_msg.goal_pose = arm_joints.end_pose;
    goal_msg.motion_execution_time = arm_joints.motion_execution_time;

    auto send_goal_options =
        rclcpp_action::Client<MotionSynthesis>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<GoalHandleMotionSynthesis> goal_handle) {
          if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(),
                        "motion_synth goal was rejected by server");
            motion_synth_goal_handle_.reset();
          } else {
            motion_synth_goal_handle_ = goal_handle;
          }
        };
    send_goal_options.result_callback =
        [this](const GoalHandleMotionSynthesis::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "motion_synth action SUCCEEDED");
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "motion_synth action failed or canceled (code: %d)",
                        static_cast<int>(result.code));
          }
          motion_synth_goal_handle_.reset();
        };

    motion_synth_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void request_clear_memory_all_obstacles() {
    if (clear_memory_request_sent_) {
      return;
    }

    clear_memory_request_sent_ = true;
    clear_memory_response_received_ = false;
    clear_memory_success_ = false;
    clear_memory_failed_ = false;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    clt_clear_memory_all_obstacles_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          try {
            auto response = future.get();
            clear_memory_success_ = response->success;
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "MotionPlanner.-> Failed to call "
                         "clear_memory_all_obstacles: %s",
                         e.what());
            clear_memory_failed_ = true;
          }

          clear_memory_response_received_ = true;
          clear_memory_request_sent_ = false;
        });
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePumasNav> goal_handle) {
    active_goal_handle_ = goal_handle;
    action_active_ = true;
    cancel_requested_ = false;

    const auto goal = goal_handle->get_goal();

    global_goal_ = goal->goal.pose;
    new_global_goal_ = true;

    // Via points for this goal (empty -> plain A* in SM_CALCULATE_PATH).
    via_points_ = goal->via_points.poses;
    RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> Goal with %zu via points",
                via_points_.size());

    // Stop-short distance for this goal (<= 0 disables; full arrival).
    goal_distance_ = goal->goal_distance;

    if (goal->use_arm) {
      target_arm_pose = goal->arm_joints;
      arm_goal_received = true;
    } else {
      arm_goal_received = false;
    }

    near_goal_sent = false;
  }

  void publish_nav_feedback(const std::string &state_name,
                            const std::string &message = "") {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto feedback = std::make_shared<PumasNav::Feedback>();

    get_robot_position();
    float remaining_distance =
        std::sqrt(std::pow(global_goal_.position.x - robot_x_, 2) +
                  std::pow(global_goal_.position.y - robot_y_, 2));

    feedback->state_name = state_name;
    feedback->remaining_distance = remaining_distance;
    feedback->collision_risk = collision_risk_;
    feedback->near_goal_reached = near_goal_sent;
    feedback->message = message;

    active_goal_handle_->publish_feedback(feedback);
  }

  void set_map_augmenter_enable(bool enable) {
    std_msgs::msg::Bool msg;
    msg.data = enable;
    pub_map_aug_enable_->publish(msg);
  }

  void finish_action_success(const std::string &message) {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = true;
    result->near_goal_reached = near_goal_sent;
    result->message = message;

    active_goal_handle_->succeed(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  void finish_action_abort(const std::string &message) {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = false;
    result->near_goal_reached = near_goal_sent;
    result->message = message;

    active_goal_handle_->abort(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  void finish_action_cancel(const std::string &message) {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = false;
    result->near_goal_reached = near_goal_sent;
    result->message = message;

    active_goal_handle_->canceled(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  // ############
  // motion planner main processing
  void motion_planner_processing() {
    if (!services_ready_) {
      RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Services not ready.");
      return;
    }

    try {
      if (stop_) {
        stop_ = false;
        state = SM_INIT;

        if (action_active_) {
          if (cancel_requested_) {
            finish_action_cancel("Navigation task canceled");
          } else {
            finish_action_abort("Stop signal received. Task aborted");
          }
        }

        if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
          current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
                             "Stop signal received. Task cancelled");
        }
      }
      if (new_global_goal_) {
        state = SM_WAITING_FOR_TASK;

        if (arm_goal_received) {
          send_motion_synth_goal(global_goal_, target_arm_pose);
          arm_goal_received = false;
        }
      }

      switch (state) {
      case SM_INIT: {
        std::cout << "MotionPlanner.-> MVN PLN READY. Waiting for new goal. "
                  << std::endl;
        state = SM_WAITING_FOR_TASK;
        break;
      }

      case SM_WAITING_FOR_TASK: {
        publish_nav_feedback("SM_WAITING_FOR_TASK", "Waiting for task");
        if (new_global_goal_) {
          new_global_goal_ = false;
          set_map_augmenter_enable(true);
          state = SM_CALCULATE_PATH;
          if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE)
            current_status =
                publish_status(actionlib_msgs::msg::GoalStatus::ABORTED,
                               goal_id, "Cancelling current movement.");
          goal_id++;
          std::cout
              << "MotionPlanner.-> New goal received. Current task goal_id: "
              << goal_id << std::endl;
          current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
                             "Starting new movement task");
          near_goal_sent = false;
          goal_relocated_ = false;
          reset_recovery_state(); // clean slate per task (hygiene)
        }
        break;
      }

      case SM_CALCULATE_PATH: {
        publish_nav_feedback("CALCULATE_PATH", "Calculating path");
        get_robot_position();
        update_via_points();
        if (!via_points_.empty()) {
          get_plan_path_with_via(robot_x_, robot_y_, global_goal_.position.x,
                                 global_goal_.position.y);
        } else {
          get_plan_path_from_augmented_map(robot_x_, robot_y_,
                                           global_goal_.position.x,
                                           global_goal_.position.y);
        }

        state = SM_WAIT_FOR_PATH_RESPONSE;

        break;
      }

      case SM_WAIT_FOR_PATH_RESPONSE: {
        if (!is_path_response_) {
          // Still waiting, don't do anything yet
          break;
        }

        if (!is_path_) {
          std::cout << "MotionPlanner.-> Cannot calc path to "
                    << global_goal_.position.x << " " << global_goal_.position.y
                    << std::endl;
          pub_simple_move_stop_->publish(std_msgs::msg::Empty());

          if (memory_all_obstacles_) {
            if (!clt_clear_memory_all_obstacles_->wait_for_service(
                    std::chrono::seconds(1))) {
              RCLCPP_ERROR(this->get_logger(),
                           "MotionPlanner.-> clear_memory_all_obstacles "
                           "service not available; falling back.");
              state = patience_ ? SM_CHECK_IF_OBSTACLES
                                : SM_CHECK_IF_INSIDE_OBSTACLES;
            } else {
              request_clear_memory_all_obstacles();
              state = SM_WAIT_FOR_CLEAR_MEMORY_RESPONSE;
            }
          } else {
            state = patience_ ? SM_CHECK_IF_OBSTACLES
                              : SM_CHECK_IF_INSIDE_OBSTACLES;
          }
        } else {
          if (is_temporary_no_cloud_pot_fields_) {
            no_cloud_pot_fields_start_time_ = this->now(); // reset timer
            state = SM_START_MOVE_PATH;
          } else {
            state = SM_ENABLE_POT_FIELDS;
          }
        }
        break;
      }

      case SM_WAIT_FOR_CLEAR_MEMORY_RESPONSE: {
        if (!clear_memory_response_received_) {
          break;
        }

        if (clear_memory_failed_) {
          RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> clear_memory_all_obstacles call failed.");
        } else if (clear_memory_success_) {
          RCLCPP_WARN(this->get_logger(),
                      "MotionPlanner.-> Cleared memory obstacles due to path "
                      "planning failure; retrying path calculation.");
        } else {
          RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> clear_memory_all_obstacles returned false.");
        }

        state = SM_CALCULATE_PATH;
        break;
      }

      case SM_CHECK_IF_INSIDE_OBSTACLES: {
        std::cout
            << "MotionPlanner.-> Checking if robot is inside an obstacle..."
            << std::endl;
        is_in_obstacles_ = false;
        is_in_obstacles_response_ = false;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        clt_is_in_obstacles_->async_send_request(
            request,
            [this](
                rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                is_in_obstacles_ = future.get()->success;
              } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "MotionPlanner.-> Failed to get obstacles: %s",
                             e.what());
                is_in_obstacles_ = false;
              }
              is_in_obstacles_response_ = true;
            });

        state = SM_WAIT_FOR_INSIDE_OBSTACLES_RESPONSE;
        break;
      }

      case SM_WAIT_FOR_INSIDE_OBSTACLES_RESPONSE: {
        if (!is_in_obstacles_response_) {
          RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Waiting for obstacle check response...");
          break;
        }

        if (is_in_obstacles_) {
          std::cout << "MotionPlanner.-> Robot is inside an obstacle. Moving "
                       "backwards..."
                    << std::endl;
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;

          msg_goal_dist_angle.data.resize(2);
          msg_goal_dist_angle.data[0] = -0.15;
          msg_goal_dist_angle.data[1] = 0;
          pub_goal_dist_angle_->publish(msg_goal_dist_angle);
          state = SM_WAITING_FOR_MOVE_BACKWARDS;
        } else {
          // current_status =
          //     publish_status(actionlib_msgs::msg::GoalStatus::ABORTED,
          //     goal_id,
          //                    "Cannot calc path from start to goal");
          current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
              "Recalculating path because start pose is inside obstacle");
          // state = SM_INIT; //default impl
          state = SM_CALCULATE_PATH;
        }
        break;
      }

      case SM_WAITING_FOR_MOVE_BACKWARDS: {
        if (simple_move_goal_status_.status ==
                actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == -1) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout << "MotionPlanner.-> Moved backwards successfully."
                    << std::endl;
          state = SM_CALCULATE_PATH;
        } else if (simple_move_goal_status_.status ==
                   actionlib_msgs::msg::GoalStatus::ABORTED) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout << "MotionPlanner.-> Simple move reported move aborted."
                    << std::endl;
          state = SM_CALCULATE_PATH;
        } else {
          // while backwards
          break;
        }
        break;
      }

      case SM_CHECK_IF_OBSTACLES: {
        are_there_obs_ = false;
        is_check_obs_response_ = false;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        clt_are_there_obs_->async_send_request(
            request,
            [this](
                rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                are_there_obs_ = future.get()->success;
              } catch (const std::exception &e) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "MotionPlanner.-> Failed to get obstacle check: %s",
                    e.what());
                are_there_obs_ = false;
              }
              is_check_obs_response_ = true;
            });

        state = SM_WAIT_FOR_IF_OBSTACLES_RESPONSE;
        break;
      }

      case SM_WAIT_FOR_IF_OBSTACLES_RESPONSE: {
        if (!is_check_obs_response_) {
          RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Waiting for obstacle check response...");
          break;
        }

        if (!are_there_obs_) {
          std::cout << "MotionPlanner.->There are no temporal obstacles. "
                       "Announcing failure."
                    << std::endl;
          current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
                             "Cannot calculate path from start to goal point");
          finish_action_abort("Cannot calculate path from start to goal point");
          state = SM_INIT;
        } else {
          std::cout << "MotionPlanner.->Temporal obstacles detected. Waiting "
                       "for them to move."
                    << std::endl;
          current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
                             "Waiting for temporal obstacles to move");
          state = SM_WAIT_FOR_NO_OBSTACLES;
        }
        break;
      }

      case SM_WAIT_FOR_NO_OBSTACLES: {
        is_wait_obs_response_ = false;
        are_still_obs_ = true;
        wait_obs_failed_ = false;

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        clt_are_there_obs_->async_send_request(
            request,
            [this](
                rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                are_still_obs_ = future.get()->success;
              } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "MotionPlanner.-> Cannot call service for "
                             "checking temporal obstacles: %s",
                             e.what());
                wait_obs_failed_ = true;
              }
              is_wait_obs_response_ = true;
            });

        state = SM_WAIT_FOR_NO_OBSTACLES_RESPONSE;
        break;
      }

      case SM_WAIT_FOR_NO_OBSTACLES_RESPONSE: {
        if (!is_wait_obs_response_) {
          RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Waiting for temporal obstacles to clear...");
          break;
        }

        if (wait_obs_failed_) {
          RCLCPP_ERROR(this->get_logger(),
                       "MotionPlanner.-> Cannot call service for checking "
                       "temporal obstacles. Announcing failure.");

          current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ABORTED, this->goal_id,
              "Cannot calculate path from start to goal point");
          finish_action_abort("Cannot calculate path from start to goal point");
          state = SM_INIT;

        } else if (!are_still_obs_) {
          std::cout << "MotionPlanner.-> Temporal obstacles removed."
                    << std::endl;
          state = SM_CALCULATE_PATH;
        } else {
          rclcpp::sleep_for(std::chrono::seconds(1));
          state = SM_WAIT_FOR_NO_OBSTACLES;
        }

        break;
      }

      case SM_ENABLE_POT_FIELDS: {
        msg_bool.data = true;
        pub_pot_fields_enable_->publish(msg_bool);
        std::cout << "MotionPlanner.-> Potential fields enable flag sent. "
                     "Waiting for potential fields to be enabled..."
                  << std::endl;
        state = SM_WAIT_FOR_POT_FIELDS;
        break;
      }

      case SM_WAIT_FOR_POT_FIELDS: {
        if (!waiting_for_potential_fields_) {
          RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Waiting for potential fields message...");

          is_pot_fields_response_ = false;
          pot_fields_received_ = false;
          waiting_for_potential_fields_ = true;
          pot_fields_start_time_ = this->now();
        }

        if (is_pot_fields_response_) {
          waiting_for_potential_fields_ = false;

          if (pot_fields_received_) {
            RCLCPP_INFO(this->get_logger(),
                        "MotionPlanner.-> Potential fields is now available.");
            state = SM_START_MOVE_PATH;
          } else {
            RCLCPP_WARN(
                this->get_logger(),
                "MotionPlanner.-> Potential fields received but invalid.");
            waiting_for_potential_fields_ = false;
            finish_action_abort("Potential fields received but invalid");
            state = SM_INIT;
          }
        } else if ((this->now() - pot_fields_start_time_).seconds() > 10.0) {
          RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> Timeout waiting for potential fields message.");

          finish_action_abort("Timeout waiting for potential fields message");
          waiting_for_potential_fields_ = false;
          state = SM_INIT;
        }

        break;
      }

      case SM_START_MOVE_PATH: {
        std::cout << "MotionPlanner.-> Starting path following " << std::endl;
        collision_risk_ = false;
        collision_risk_latched_ = false;
        simple_move_sequencer++;

        path_.header.frame_id = "map";
        path_.header.stamp = this->now();
        pub_goal_path_->publish(path_);

        // modified by ry0hei-kobayashi, 2025/8/23
        // path_.header.frame_id = std::to_string(simple_move_sequencer);
        simple_move_sequencer = path_.header.stamp.sec;

        simple_move_goal_status_.status = 0;

        state = SM_WAIT_FOR_MOVE_FINISHED;
        break;
      }

      case SM_WAIT_FOR_MOVE_FINISHED: {

        publish_nav_feedback("WAIT_FOR_MOVE_FINISHED", "Following path");
        get_robot_position();
        update_via_points();
        error = sqrt(pow(global_goal_.position.x - robot_x_, 2) +
                     pow(global_goal_.position.y - robot_y_, 2));

        // Recovery escape: once we have been recovering, fully reset the
        // recovery state (counters + cloud box) as soon as the robot has made
        // real forward progress, i.e. closed recovery_progress_dist_ of the
        // distance to the goal it had when it first got stuck. Tied to escaping
        // recovery, NOT to the start->goal span. (A back-up increases `error`,
        // so it can never trigger this.)
        if (recovery_stuck_recorded_ &&
            error <= stuck_goal_dist_ - recovery_progress_dist_) {
          std::cout << "MotionPlanner.-> Escaped collision recovery (closed "
                    << (stuck_goal_dist_ - error)
                    << " m toward goal). Resetting recovery state."
                    << std::endl;
          reset_recovery_state();
        }

        // Stop-short-of-goal (server-side). When enabled, halt as soon as the
        // robot is within goal_distance_ of the goal. Fresh TF (30 ms) makes
        // `error` reliable; we actively stop simple_move so the robot really
        // halts (unlike the old client-side cancel), skip final-angle
        // correction (we are not at the goal), and report success. Placed
        // before the proximity/collision checks so it wins the cycle.
        if (goal_distance_ > 0.0f && error < goal_distance_) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(2)
              << "Stopped within goal_distance (" << error
              << " m from goal, threshold " << goal_distance_ << " m)";
          const std::string msg = oss.str();
          std::cout << "MotionPlanner.-> " << msg << std::endl;

          // 1) Actually halt the robot (authoritative stop).
          pub_simple_move_stop_->publish(std_msgs::msg::Empty());
          // 2) Disable potential fields (mirror the normal SUCCEEDED branch).
          msg_bool.data = false;
          pub_pot_fields_enable_->publish(msg_bool);
          // 3) Clear any collision-recovery state / restore cloud.
          reset_recovery_state();
          // 4) Mark near-goal and emit one final feedback BEFORE finishing so a
          //    client near_goal_callback (feedback-driven) still fires.
          near_goal_sent = true;
          publish_nav_feedback("STOP_SHORT", msg);
          // 5) Report success and return to idle (skip final-angle correction).
          current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::SUCCEEDED, goal_id, msg);
          finish_action_success(msg);
          state = SM_INIT;
          break;
        }

        if (error < proximity_criterion_ && !near_goal_sent) {
          near_goal_sent = true;
          std::cout << "MotionPlanner.->Error less than proximity criterion. "
                       "Sending near goal point status."
                    << std::endl;
          publish_nav_feedback("NEAR_GOAL", "Near goal point");
          publish_status(actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
                         "Near goal point");
        }
        if (simple_move_goal_status_.status ==
                actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == simple_move_sequencer) {
          simple_move_goal_status_.status = 0;
          std::cout << "MotionPlanner.-> Path followed succesfully. "
                    << std::endl;
          msg_bool.data = false;
          pub_pot_fields_enable_->publish(msg_bool);
          reset_recovery_state();
          state = SM_CORRECT_FINAL_ANGLE;
        } else if (simple_move_goal_status_.status ==
                   actionlib_msgs::msg::GoalStatus::ABORTED) {
          simple_move_goal_status_.status = 0;
          if (collision_risk_latched_) {
            // Collision-risk abort: count it. Below the trigger, just replan
            // (the obstacle may be transient). At/above it, escalate to active
            // recovery.
            if (!recovery_stuck_recorded_) {
              stuck_goal_dist_ = error;
              recovery_stuck_recorded_ = true;
            }
            collision_event_count_++;
            std::cout << "MotionPlanner.-> COLLISION RISK abort #"
                      << collision_event_count_ << " (recovery triggers at "
                      << collision_recovery_trigger_count_ << ")." << std::endl;
            if (collision_event_count_ < collision_recovery_trigger_count_) {
              state = SM_CALCULATE_PATH;
            } else {
              state = SM_COLLISION_RECOVERY;
            }
          } else {
            // Generic (timeout) abort: unchanged behaviour. //TODO
            std::cout << "MotionPlanner.-> Simple move reported path aborted. "
                         "Trying again..."
                      << std::endl;
            state = SM_CALCULATE_PATH;
          }
        }
        break;
      }

      case SM_COLLISION_RECOVERY: {
        get_robot_position();
        if (use_online_ || recovery_level_ == 0) {
          // Tier 0: risk-aware escape move. Fetch the augmented map first so
          // the escape direction (away from the obstacle) can be chosen +
          // clearance checked. ONLINE always lands here: the shrink / cloud-off
          // tiers below are OFFLINE-only (online must never drop the cloud and,
          // empirically, shrinking the cloud box does not clear a phantom that
          // sits inside the floored box). For online, SM_RECOVERY_WAIT_MAP falls
          // back to an in-place rotation when no translational escape is clear.
          std::cout << "MotionPlanner.-> Collision recovery tier0: escape move "
                       "(choosing direction away from obstacle)."
                    << std::endl;
          request_augmented_map();
          state = SM_RECOVERY_WAIT_MAP;
        } else if (recovery_level_ <= recovery_max_shrink_steps_) {
          // Tier 1..N: shrink the cloud detection box one notch, replan.
          detection_scale_ =
              std::max(recovery_min_detection_scale_,
                       detection_scale_ * recovery_shrink_factor_);
          publish_detection_scale(detection_scale_);
          std::cout << "MotionPlanner.-> Collision recovery tier"
                    << recovery_level_
                    << ": shrink cloud box, scale=" << detection_scale_
                    << std::endl;
          advance_recovery_level();
          state = SM_CALCULATE_PATH;
        } else {
          // Tier N+1 (last resort, OFFLINE only): even the floored cloud box
          // cannot get through. Temporarily disable the point cloud entirely
          // (lidar stays full as the collision_risk trigger), then replan.
          // Latched until reset_recovery_state() re-enables it on recovery
          // escape / goal reached / new task. Online navigation must NEVER drop
          // the cloud: advance_recovery_level() keeps recovery_level_ from ever
          // reaching this tier when use_online_ is true; the !use_online_ guard
          // below is a belt-and-suspenders backstop.
          if (!use_online_ && !recovery_cloud_disabled_) {
            std_msgs::msg::Bool m;
            m.data = false;
            pub_pot_fields_enable_cloud_->publish(m);
            pub_map_aug_enable_cloud_->publish(m);
            recovery_cloud_disabled_ = true;
            std::cout << "MotionPlanner.-> Collision recovery tier"
                      << recovery_level_
                      << ": last resort, point cloud DISABLED (lidar only)."
                      << std::endl;
          }
          advance_recovery_level();
          state = SM_CALCULATE_PATH;
        }
        break;
      }

      case SM_RECOVERY_WAIT_MAP: {
        if (!recovery_map_response_)
          break; // waiting for augmented map

        float rel_x = 0.0f, rel_y = 0.0f;
        if (choose_escape_move(rel_x, rel_y)) {
          std::cout << "MotionPlanner.-> Recovery escape move (away from "
                       "obstacle): x="
                    << rel_x << " y=" << rel_y << " m." << std::endl;
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std_msgs::msg::Float32MultiArray rp;
          rp.data = {rel_x, rel_y, 0.0f}; // [x fwd+, y left+, yaw] base frame
          pub_goal_rel_pose_->publish(rp);
          state = SM_RECOVERY_WAIT_BACKUP;
        } else if (use_online_) {
          // ONLINE: no translational escape is clear -> the robot is boxed in on
          // all sides, typically by a directional point-cloud phantom (lidar +
          // cloud both on). Shrinking the cloud box does not help once the
          // phantom sits inside the floored box, and forcing a translation would
          // risk a real collision. Instead rotate IN PLACE: the base center is
          // fixed (collision-safe) and the head camera reorients, which sheds a
          // directional phantom. Replan + retry after each rotation; if a full
          // turn still finds no way through, the robot is enclosed -> stop at the
          // nearest reachable point rather than looping forever.
          if (recovery_rotated_total_ < recovery_rotate_max_) {
            std::cout << "MotionPlanner.-> No clear translational escape; "
                         "rotating in place "
                      << recovery_rotate_step_
                      << " rad to shed point-cloud phantom (rotated so far="
                      << recovery_rotated_total_ << " rad)." << std::endl;
            simple_move_goal_status_.status = 0;
            simple_move_status_id_ = 0;
            std_msgs::msg::Float32MultiArray rp;
            rp.data = {0.0f, 0.0f,
                       recovery_rotate_step_}; // [x, y, yaw]; no translation
            pub_goal_rel_pose_->publish(rp);
            recovery_rotated_total_ += std::fabs(recovery_rotate_step_);
            state = SM_RECOVERY_WAIT_ROTATE;
          } else {
            std::cout << "MotionPlanner.-> Rotation scan exhausted; robot is "
                         "enclosed. Stopping at nearest reachable point."
                      << std::endl;
            std_msgs::msg::Bool m;
            m.data = false;
            pub_pot_fields_enable_->publish(m);
            reset_recovery_state();
            finish_action_success(
                "Blocked by obstacles; stopped at nearest reachable point");
            state = SM_INIT;
          }
        } else {
          std::cout << "MotionPlanner.-> No clear escape direction; shrinking "
                       "cloud box instead."
                    << std::endl;
          recovery_level_ = 1; // jump straight to the shrink tier
          state = SM_COLLISION_RECOVERY;
        }
        break;
      }

      case SM_RECOVERY_WAIT_BACKUP: {
        if (simple_move_goal_status_.status ==
                actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == -1) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout << "MotionPlanner.-> Recovery escape move finished."
                    << std::endl;
          advance_recovery_level();
          state = SM_CALCULATE_PATH;
        } else if (simple_move_goal_status_.status ==
                   actionlib_msgs::msg::GoalStatus::ABORTED) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout
              << "MotionPlanner.-> Recovery escape move aborted; continuing."
              << std::endl;
          advance_recovery_level();
          state = SM_CALCULATE_PATH;
        } else {
          break; // still backing up
        }
        break;
      }

      case SM_RECOVERY_WAIT_ROTATE: {
        // In-place recovery rotation (ONLINE phantom shedding). On completion,
        // stay in the escape/rotation tier (recovery_level_ = 0) and replan, so
        // the next collision re-enters the escape->rotate path and the rotation
        // scan keeps accumulating until the robot breaks free (forward progress
        // resets recovery) or a full turn is exhausted.
        if (simple_move_goal_status_.status ==
                actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == -1) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout << "MotionPlanner.-> Recovery rotation finished; replanning."
                    << std::endl;
          recovery_level_ = 0;
          state = SM_CALCULATE_PATH;
        } else if (simple_move_goal_status_.status ==
                   actionlib_msgs::msg::GoalStatus::ABORTED) {
          simple_move_goal_status_.status = 0;
          simple_move_status_id_ = 0;
          std::cout << "MotionPlanner.-> Recovery rotation aborted; replanning."
                    << std::endl;
          recovery_level_ = 0;
          state = SM_CALCULATE_PATH;
        } else {
          break; // still rotating
        }
        break;
      }

      case SM_WAIT_FOR_NOT_POT_FIELDS: {
        if (!waiting_for_potential_fields_) {
          RCLCPP_INFO(this->get_logger(),
                      "MotionPlanner.-> Waiting for potential fields with "
                      "point cloud to be disabled...");

          waiting_for_potential_fields_ = true;
          pot_fields_start_time_ = this->now();
          no_cloud_pot_fields_start_time_ = this->now();
        }
        if ((this->now() - pot_fields_start_time_).seconds() > 1.0) {
          pot_fields_start_time_ = this->now();
          if (is_pot_fields_response_) {
            RCLCPP_WARN(this->get_logger(),
                        "MotionPlanner.-> Potential fields with point cloud "
                        "have been disabled.");
            collision_risk_ = false;
            waiting_for_potential_fields_ = false;

            state = SM_CALCULATE_PATH;
          }
        } else if ((this->now() - no_cloud_pot_fields_start_time_).seconds() >
                   10.0) {
          RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> Timeout waiting for potential fields message.");
          collision_risk_ = false;
          waiting_for_potential_fields_ = false;

          state = SM_CALCULATE_PATH;
        }
        break;
      }

      case SM_CORRECT_FINAL_ANGLE: {
        std::cout << "MotionPlanner.-> Correcting final angle." << std::endl;
        get_robot_position();

        double goal_yaw = tf2::getYaw(global_goal_.orientation);
        double error = goal_yaw - static_cast<double>(robot_t_);

        if (error > M_PI)
          error -= 2 * M_PI;
        if (error <= -M_PI)
          error += 2 * M_PI;

        msg_goal_dist_angle.data.resize(2);
        msg_goal_dist_angle.data[0] = 0.0f;
        msg_goal_dist_angle.data[1] = static_cast<float>(error);
        pub_goal_dist_angle_->publish(msg_goal_dist_angle);

        state = SM_WAIT_FOR_ANGLE_CORRECTED;
        break;
      }

      case SM_WAIT_FOR_ANGLE_CORRECTED: {
        if (simple_move_goal_status_.status ==
                actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == -1) {
          simple_move_goal_status_.status = 0;
          std::cout << "MotionPlanner.-> Final angle corrected succesfully."
                    << std::endl;
          state = SM_FINAL;
        } else if (simple_move_goal_status_.status ==
                   actionlib_msgs::msg::GoalStatus::ABORTED) {
          simple_move_goal_status_.status = 0;
          std::cout << "MotionPlanner.-> Simple move reported move aborted. "
                       "Trying again..."
                    << std::endl;
          state = SM_CALCULATE_PATH;
        }
        break;
      }

      case SM_FINAL: {
        std::cout << "MotionPlanner.-> TASK FINISHED." << std::endl;
        std::string final_msg = "Global goal point reached";
        if (goal_relocated_) {
          // The requested goal was enclosed/unreachable; the planner stopped at
          // the nearest reachable point. Report it explicitly and make sure
          // near_goal_reached is set in the action result.
          near_goal_sent = true;
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(2)
              << "Goal enclosed/unreachable; stopped at nearest reachable "
                 "point ("
              << goal_relocated_dist_ << " m from requested goal)";
          final_msg = oss.str();
        }
        current_status = publish_status(
            actionlib_msgs::msg::GoalStatus::SUCCEEDED, goal_id, final_msg);
        finish_action_success(final_msg);
        state = SM_INIT;
        break;
      }

      default:
        std::cout << "MotionPlanner.-> A VERY STUPID PERSON PROGRAMMED THIS "
                     "SHIT. APOLOGIES FOR THE INCONVINIENCE :'("
                  << std::endl;
        return;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "MotionPlanner.-> Error in MotionPlanner processing: %s",
                   e.what());
      if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
        current_status =
            publish_status(actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
                           "Exception in motion_planner_processing core");
      }
      if (action_active_) {
        finish_action_abort("Exception in motion_planner_processing core");
      }
      state = SM_INIT;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlannerNode>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
