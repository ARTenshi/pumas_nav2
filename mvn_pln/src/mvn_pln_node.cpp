#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "actionlib_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "pumas_interfaces/srv/get_plan_with_via.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

// TF2 (Transform listener)
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // for doTransform

#include <tf2/utils.hpp>  // for getYaw
#include <tf2/time.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

// For motion_synthesizer action client // motion synthesis maintain by
// ry0hei-kobayashi
#include "rclcpp_action/rclcpp_action.hpp"

#include "pumas_interfaces/action/motion_synthesis.hpp"
#include "pumas_interfaces/action/pumas_nav.hpp"
#include "pumas_interfaces/msg/joints.hpp"
#include "pumas_interfaces/msg/start_and_end_joints.hpp"

// Standard
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <exception>
#include <iomanip>  // for std::hex and std::setw
#include <iostream>
#include <limits>
#include <sstream>  // for std::stringstream
#include <string>
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
// Non-blocking gap between two obstacle re-polls (replaces a sleep_for(1s)).
#define SM_WAIT_FOR_NO_OBSTACLES_DELAY 25
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
// Goal-convergence track. Numbered clear of the 40-43 collision-recovery block
// on purpose: the two ladders are independent and must not be confused.
#define SM_GOAL_CONVERGENCE_CHECK 50
#define SM_CONVERGENCE_WAIT_CLEAR_MEM 51
#define SM_CONVERGENCE_WAIT_FREE_MAP 52

#define SM_WAIT_FOR_PATH_RESPONSE 102
#define SM_WAIT_FOR_INSIDE_OBSTACLES_RESPONSE 119
#define SM_WAIT_FOR_IF_OBSTACLES_RESPONSE 121
#define SM_WAIT_FOR_NO_OBSTACLES_RESPONSE 122

// motionsynthesis client by r.kobayashi
using MotionSynthesis = pumas_interfaces::action::MotionSynthesis;
using PumasNav = pumas_interfaces::action::PumasNav;
using GoalHandleMotionSynthesis = rclcpp_action::ClientGoalHandle<MotionSynthesis>;
using GoalHandlePumasNav = rclcpp_action::ServerGoalHandle<PumasNav>;

class MotionPlannerNode : public rclcpp::Node
{
public:
  MotionPlannerNode()
  : Node("motion_planner_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // ############
    //  Declare parameters with default values
    this->declare_parameter("use_namespace", false);
    this->declare_parameter<bool>("patience", true);
    this->declare_parameter<float>("proximity_criterion", 2.0f);
    this->declare_parameter<std::string>("base_link_name", "base_footprint");
    this->declare_parameter<bool>("remember_all_obstacles", false);

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

    // Goal-convergence monitor. Independent of the collision-recovery ladder:
    // it answers "is the final goal converging at all?", which collision
    // recovery does not, because the planner can keep returning a perfectly
    // good path that the robot is simply never able to traverse.
    this->declare_parameter<double>("convergence_epsilon", 0.20);
    this->declare_parameter<double>("convergence_timeout", 45.0);
    this->declare_parameter<double>("replan_window_sec", 30.0);
    this->declare_parameter<int>("replan_window_max", 5);
    this->declare_parameter<double>("replan_spread_radius", 2.0);
    this->declare_parameter<int>("replan_max_per_task", 40);
    this->declare_parameter<double>("goal_relax_max", 1.5);
    this->declare_parameter<bool>("enable_goal_relax", true);
    this->declare_parameter<bool>("enable_stuck_abort", true);
    this->declare_parameter<double>("free_point_search_radius", 2.0);

    // Near-goal replan suppression. Once the robot is this close [m] to the
    // goal there is nothing useful left to plan: a replan can only return
    // another near-zero path, which is what makes a goal placed ON an obstacle
    // loop forever (the planner keeps relocating it a few centimetres away and
    // we keep re-driving those few centimetres). Same idea as goal_distance_,
    // except goal_distance_ is a per-goal request from the caller while this is
    // a safety net that only applies when a REPLAN is about to happen.
    // "This close" is a distance ALONG THE PLANNED PATH, not a straight line
    // (goal_distance_ is the straight-line one): the question here is how far
    // the robot must still drive, and a goal a metre away on the far side of a
    // wall is a long drive, not an arrival.
    // When the last plan came back relocated (the goal sits on an obstacle) the
    // relocation distance is added to this radius, so "near the goal" is
    // measured against what is actually reachable.
    // This is the node-wide default: a PumasNav goal may override it with its
    // own min_reach_goal_dist, and a goal value of 0 means "not specified" and
    // falls back here. Setting this parameter to 0 disables the gate entirely.
    this->declare_parameter<double>("min_reach_goal_dist", 0.0);
    // true  -> treat the current point as the goal (SUCCEEDED, near_goal set)
    // false -> abort the task instead
    this->declare_parameter<bool>("near_goal_accept_as_goal", true);
    // Require evidence that the goal itself is blocked before giving up near it.
    // true (default) -> only a RELOCATED goal (the planner moved the path
    //   endpoint off the requested point, i.e. the goal sits on an obstacle)
    //   ends the task. A replan caused by something merely in the way keeps
    //   planning, so the robot does not abandon a perfectly free goal because a
    //   person walked past it.
    // false -> legacy behaviour: any replan inside the radius ends the task.
    this->declare_parameter<bool>("near_goal_require_relocation", true);

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("patience", patience_);
    this->get_parameter("proximity_criterion", proximity_criterion_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("remember_all_obstacles", remember_all_obstacles_);
    this->get_parameter("goal_relocation_report_threshold", goal_relocation_report_threshold_);
    this->get_parameter("collision_recovery_trigger_count", collision_recovery_trigger_count_);
    this->get_parameter("use_online", use_online_);

    this->get_parameter("convergence_epsilon", convergence_epsilon_);
    this->get_parameter("convergence_timeout", convergence_timeout_);
    this->get_parameter("replan_window_sec", replan_window_sec_);
    this->get_parameter("replan_window_max", replan_window_max_);
    this->get_parameter("replan_spread_radius", replan_spread_radius_);
    this->get_parameter("replan_max_per_task", replan_max_per_task_);
    this->get_parameter("goal_relax_max", goal_relax_max_);
    this->get_parameter("enable_goal_relax", enable_goal_relax_);
    this->get_parameter("enable_stuck_abort", enable_stuck_abort_);
    this->get_parameter("free_point_search_radius", free_point_search_radius_);
    this->get_parameter("min_reach_goal_dist", min_reach_goal_dist_);
    this->get_parameter("near_goal_accept_as_goal", near_goal_accept_as_goal_);
    this->get_parameter("near_goal_require_relocation", near_goal_require_relocation_);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotionPlannerNode::on_parameter_change, this, std::placeholders::_1));

    // Initialize service clients (non-blocking)
    init_service_clients();

    // ############
    //  Publishers
    //
    pub_pot_fields_enable_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/navigation/potential_fields/enable"), rclcpp::QoS(10).transient_local());
    pub_pot_fields_enable_cloud_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/navigation/potential_fields/enable_cloud"), rclcpp::QoS(10).transient_local());
    pub_map_aug_enable_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/navigation/map_augmenter/enable"), rclcpp::QoS(10).transient_local());
    pub_map_aug_enable_cloud_ = this->create_publisher<std_msgs::msg::Bool>(
      make_name("/navigation/map_augmenter/enable_cloud"), rclcpp::QoS(10).transient_local());

    pub_goal_path_ = this->create_publisher<nav_msgs::msg::Path>(
      make_name("/simple_move/goal_path"), rclcpp::QoS(10).transient_local());
    pub_goal_dist_angle_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      make_name("/simple_move/goal_dist_angle"), rclcpp::QoS(10).transient_local());
    // Holonomic relative move [x, y, yaw] (base frame) for the recovery escape.
    pub_goal_rel_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      make_name("/simple_move/goal_rel_pose"), rclcpp::QoS(10).reliable());
    pub_status_ = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
      make_name("/navigation/status"), rclcpp::QoS(10).transient_local());
    pub_simple_move_stop_ = this->create_publisher<std_msgs::msg::Empty>(
      make_name("/simple_move/stop"), rclcpp::QoS(10).transient_local());
    pub_detection_scale_ = this->create_publisher<std_msgs::msg::Float32>(
      make_name("/navigation/potential_fields/detection_scale"), rclcpp::QoS(10).transient_local());

    // ############
    //  Subscribers
    //// stop
    sub_general_stop_ = this->create_subscription<std_msgs::msg::Empty>(
      make_name("/stop"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_general_stop, this, std::placeholders::_1));

    sub_nav_ctrl_stop_ = this->create_subscription<std_msgs::msg::Empty>(
      make_name("/navigation/stop"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_navigation_stop, this, std::placeholders::_1));

    sub_move_goal_status_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
      make_name("/simple_move/goal_reached"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_simple_move_goal_status, this, std::placeholders::_1));

    //// motion
    sub_set_patience_ = this->create_subscription<std_msgs::msg::Bool>(
      make_name("/navigation/set_patience"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_set_patience, this, std::placeholders::_1));

    sub_collision_risk_ = this->create_subscription<std_msgs::msg::Bool>(
      make_name("/navigation/potential_fields/collision_risk"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_collision_risk, this, std::placeholders::_1));

    //// motion synth
    sub_arm_goal_ = this->create_subscription<pumas_interfaces::msg::StartAndEndJoints>(
      make_name("/motion_synth/joint_pose"), rclcpp::SensorDataQoS(),
      std::bind(&MotionPlannerNode::callback_arm_joints, this, std::placeholders::_1));

    motion_synth_client_ = rclcpp_action::create_client<MotionSynthesis>(this, "/motion_synth");
    if (!motion_synth_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(
        this->get_logger(),
        "MotionPlanner.-> MotionSynth action server not available "
        "(expected if this robot has no arm/motion_synth_server "
        "running); navigation goals with use_arm will be rejected, "
        "everything else is unaffected.");
    } else {
      RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> MotionSynth action server is ready.");
    }

    // pumas nav action
    nav_action_server_ = rclcpp_action::create_server<PumasNav>(
      this, "/pumas_nav",
      std::bind(
        &MotionPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MotionPlannerNode::handle_accepted, this, std::placeholders::_1));

    // ############
    //  Wait for transforms
    wait_for_transforms("map", base_link_name_);

    // Motion planner main processing.
    //
    // Node-clock timer, not a wall timer: the convergence monitor and the
    // replan sliding window below measure their windows with this->now(), and
    // the state machine only advances on these ticks. create_wall_timer
    // ignores use_sim_time, so under a simulator slower than realtime the
    // 45 s convergence_timeout and the 30 s replan_window_sec would expire
    // after rtf times that many simulated seconds (21 s / 14 s at the rtf 0.47
    // measured on Isaac Sim), escalating a goal that was merely being
    // approached slowly. See the matching comment in simple_move_node.cpp.
    processing_timer_ = rclcpp::create_timer(
      this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / 30.0),
      std::bind(&MotionPlannerNode::motion_planner_processing, this));

    RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> MotionPlannerNode is ready.");
  }

private:
  // ############
  //  State variables
  bool use_namespace_ = false;
  bool stop_ = false;
  bool collision_risk_ = false;
  bool new_global_goal_ = false;
  // Goal id echoed back by simple_move on /simple_move/goal_reached. Kept as
  // the raw string so the full path timestamp survives the round trip; see
  // simple_move_sequencer. Empty means "no status pending", "-1" is the
  // sentinel simple_move uses for non-path goals.
  std::string simple_move_status_id_;

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
  std::string simple_move_sequencer;
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
  rclcpp::Duration no_cloud_pot_fields_duration_{2, 0};  // 2 seconds
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
  int collision_recovery_trigger_count_ = 3;  // ROS param
  bool use_online_ = false;                   // ROS param; gates the cloud-off recovery tier
  int collision_event_count_ = 0;
  int recovery_level_ = 0;
  bool collision_risk_latched_ = false;  // a collision-risk True seen while moving
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
  const float recovery_back_dist_ = 0.15f;            // back-up distance [m]
  const float recovery_progress_dist_ = 0.50f;        // advance to count as escaped [m]
  const float recovery_shrink_factor_ = 0.8f;         // detection scale mult / step
  const float recovery_min_detection_scale_ = 0.60f;  // detection scale floor
  const float recovery_rear_lateral_ = 0.15f;         // rear-check lateral offset [m]
  const int recovery_max_shrink_steps_ = 3;           // shrink steps before wrapping

  // ONLINE sensor-safe escape: when no translational escape is clear (the robot
  // is boxed in on all sides, typically by a directional point-cloud phantom),
  // rotate IN PLACE. The base center does not move, so the rotation cannot
  // drive the robot into an obstacle (collision-safe), and it reorients the
  // head camera, which sheds the directional phantom. Rotation is accumulated;
  // once a full scan still finds no way through, the robot is genuinely
  // enclosed, so we stop at the nearest reachable point instead of looping
  // forever.
  const float recovery_rotate_step_ = 0.7853982f;  // in-place rotation / step [rad] (~45 deg)
  const float recovery_rotate_max_ = 2.0f * M_PI;  // total rotation before giving up [rad]
  float recovery_rotated_total_ = 0.0f;

  // ############
  //  Goal-convergence monitor.
  //
  //  Deliberately separate from the collision-recovery ladder above: recovery
  //  answers "something is in the way right now", this answers "the final goal
  //  is not converging". The failure it catches is the one recovery cannot —
  //  the planner keeps handing back a full path to the goal, the robot keeps
  //  setting off down it, and the distance to the goal never improves. Its
  //  escalation runs on converge_level_ and never touches recovery_level_ or
  //  collision_event_count_.
  //
  //  Note this is NOT the pumas replan detector, which compared each replan
  //  start against only the previous one and reset whenever they were more than
  //  a metre apart — i.e. reset on essentially every real cycle, since a stuck
  //  robot still travels several metres down the path before being blocked.
  double convergence_epsilon_ = 0.20;  // ROS param [m]
  double convergence_timeout_ = 45.0;  // ROS param [s]
  double replan_window_sec_ = 30.0;    // ROS param [s]
  int replan_window_max_ = 5;          // ROS param
  double replan_spread_radius_ = 2.0;  // ROS param [m]
  int replan_max_per_task_ = 40;       // ROS param
  double goal_relax_max_ = 1.5;        // ROS param [m]
  bool enable_goal_relax_ = true;      // ROS param
  bool enable_stuck_abort_ = true;     // ROS param

  // Signal A: best distance to goal reached so far this task, and when it last
  // improved. The clock only advances in states where progress is expected.
  float best_goal_dist_ = std::numeric_limits<float>::max();
  rclcpp::Time last_progress_time_;
  bool convergence_clock_valid_ = false;
  // Signal B: replan starts inside a sliding time window. Compared as a whole
  // set, so an A<->B shuttle wider than one metre is still caught.
  struct ReplanEvent
  {
    rclcpp::Time t;
    float x;
    float y;
  };
  std::deque<ReplanEvent> replan_events_;
  // Signal C: absolute backstop, geometry ignored.
  int replan_total_ = 0;
  // Escalation tier of this track (0 clear memory, 1 relax goal, 2 abort).
  int converge_level_ = 0;
  std::string convergence_reason_;

  // Near-goal replan suppression (see the parameter declarations). Sits on the
  // replan path only: the first plan of a task is never suppressed, so a goal
  // requested from right next to the robot still gets planned and driven.
  double min_reach_goal_dist_ = 0.0;          // ROS param [m]; node-wide default
  bool near_goal_accept_as_goal_ = true;      // ROS param
  bool near_goal_require_relocation_ = true;  // ROS param
  bool near_goal_final_accepted_ = false;     // latched once the point is accepted
  // Per-goal override carried in the PumasNav goal, exactly like goal_distance_.
  // <= 0 means "not specified" and falls back to min_reach_goal_dist_.
  float goal_min_reach_dist_ = 0.0f;
  // Overrides the SM_FINAL report when the task ended somewhere other than the
  // requested goal. Cleared at each new task.
  std::string pending_final_message_;
  // Outcome that goes with pending_final_message_; only read when that message
  // is set, so SM_FINAL can report WHY it is finishing away from the goal.
  uint8_t pending_final_outcome_ = PumasNav::Result::OUTCOME_UNKNOWN;

  // Node-clock deadline for the next temporal-obstacle re-poll. Must be built
  // with RCL_ROS_TIME: a default-constructed rclcpp::Time is RCL_SYSTEM_TIME
  // and comparing it against this->now() throws on mismatched time sources.
  rclcpp::Time obstacle_recheck_at_{0, 0, RCL_ROS_TIME};

  // Nearest-free-point escape: a planning start substituted for the robot pose
  // when the robot itself sits on an occupied cell. One-shot per use.
  double free_point_search_radius_ = 2.0;  // ROS param [m]
  bool plan_start_valid_ = false;
  float plan_start_x_ = 0.0f;
  float plan_start_y_ = 0.0f;

  // Memory obstacle clear flow (used when remember_all_obstacles is enabled and
  // path planning has just failed — we wipe accumulated memory then retry)
  bool remember_all_obstacles_ = false;
  bool clear_memory_request_sent_ = false;
  bool clear_memory_response_received_ = false;
  bool clear_memory_success_ = false;
  bool clear_memory_failed_ = false;

  rclcpp::Time pot_fields_start_time_;

  // ############
  //  Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pot_fields_enable_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_pot_fields_enable_cloud_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_map_aug_enable_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_map_aug_enable_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_goal_path_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_goal_dist_angle_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_goal_rel_pose_;
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
  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr sub_move_goal_status_;

  //// motion
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_set_patience_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collision_risk_;

  //// motion synth
  rclcpp::Subscription<pumas_interfaces::msg::StartAndEndJoints>::SharedPtr sub_arm_goal_;
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
  rclcpp::Client<pumas_interfaces::srv::GetPlanWithVia>::SharedPtr clt_plan_path_with_via_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_aug_map_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_aug_costmap_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_are_there_obs_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_is_in_obstacles_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_clear_obstacle_memory_;

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
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PumasNav::Goal> goal)
  {
    (void)uuid;
    (void)goal;

    // A new goal always wins: it preempts whatever is running (handle_accepted
    // terminates the old handle). Rejecting instead used to make an RViz goal
    // sent mid-navigation silently do nothing, and made navlib's
    // cancel-then-send racy -- the cancel only takes effect on the next 30 Hz
    // tick, so the follow-up goal could still land while action_active_ was
    // true and be rejected.
    if (action_active_) {
      RCLCPP_WARN(
        this->get_logger(), "MotionPlanner.-> New goal will preempt the active navigation task.");
    }

    RCLCPP_INFO(this->get_logger(), "MotionPlanner.-> Navigation action goal accepted request.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePumasNav> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> Cancel request received.");
    cancel_requested_ = true;
    stop_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ############
  //  Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      if (param.get_name() == "use_namespace")
        use_namespace_ = param.as_bool();
      else if (param.get_name() == "patience")
        patience_ = param.as_bool();
      else if (param.get_name() == "proximity_criterion")
        proximity_criterion_ = param.as_double();
      else if (param.get_name() == "base_link_name")
        base_link_name_ = param.as_string();
      else if (param.get_name() == "remember_all_obstacles")
        remember_all_obstacles_ = param.as_bool();
      else if (param.get_name() == "goal_relocation_report_threshold")
        goal_relocation_report_threshold_ = param.as_double();
      else if (param.get_name() == "collision_recovery_trigger_count")
        collision_recovery_trigger_count_ = param.as_int();
      else if (param.get_name() == "use_online")
        use_online_ = param.as_bool();

      else if (param.get_name() == "convergence_epsilon")
        convergence_epsilon_ = param.as_double();
      else if (param.get_name() == "convergence_timeout")
        convergence_timeout_ = param.as_double();
      else if (param.get_name() == "replan_window_sec")
        replan_window_sec_ = param.as_double();
      else if (param.get_name() == "replan_window_max")
        replan_window_max_ = param.as_int();
      else if (param.get_name() == "replan_spread_radius")
        replan_spread_radius_ = param.as_double();
      else if (param.get_name() == "replan_max_per_task")
        replan_max_per_task_ = param.as_int();
      else if (param.get_name() == "goal_relax_max")
        goal_relax_max_ = param.as_double();
      else if (param.get_name() == "enable_goal_relax")
        enable_goal_relax_ = param.as_bool();
      else if (param.get_name() == "enable_stuck_abort")
        enable_stuck_abort_ = param.as_bool();
      else if (param.get_name() == "free_point_search_radius")
        free_point_search_radius_ = param.as_double();
      else if (param.get_name() == "min_reach_goal_dist")
        min_reach_goal_dist_ = param.as_double();
      else if (param.get_name() == "near_goal_accept_as_goal")
        near_goal_accept_as_goal_ = param.as_bool();
      else if (param.get_name() == "near_goal_require_relocation")
        near_goal_require_relocation_ = param.as_bool();

      else {
        result.successful = false;
        result.reason = "MotionPlanner.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(
          this->get_logger(), "MotionPlanner.-> Attempted to update unsupported parameter: %s",
          param.get_name().c_str());
        break;
      }
    }

    return result;
  }

  std::string make_name(const std::string & suffix) const
  {
    // Ensure suffix starts with "/"
    std::string sfx = suffix;
    if (!sfx.empty() && sfx.front() != '/') sfx = "/" + sfx;

    std::string name;

    if (use_namespace_) {
      // Use node namespace prefix
      name = this->get_namespace() + sfx;

      // Avoid accidental double slash (e.g., when namespace is "/")
      if (name.size() > 1 && name[0] == '/' && name[1] == '/') name.erase(0, 1);
    } else {
      // Use global namespace (no node namespace prefix)
      name = sfx;
    }

    return name;
  }

  // Wait for transforms
  void wait_for_transforms(const std::string & target_frame, const std::string & source_frame)
  {
    RCLCPP_INFO(
      this->get_logger(), "MotionPlanner.-> Waiting for transform from '%s' to '%s'...",
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

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time) < timeout) {
      try {
        tf_buffer_.lookupTransform(
          target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
        transform_ok = true;
        break;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "MotionPlanner.-> Still waiting for transform: %s", ex.what());
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (!transform_ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "MotionPlanner.-> Timeout while waiting for transform from "
        "'%s' to '%s'.",
        source_frame.c_str(), target_frame.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(), "MotionPlanner.-> Transform from '%s' to '%s' is now available.",
        source_frame.c_str(), target_frame.c_str());
    }
  }

  // ############
  //  Initialize service clients (non-blocking)
  void init_service_clients()
  {
    clt_plan_path_static_ =
      this->create_client<nav_msgs::srv::GetPlan>(make_name("/path_planner/plan_path_with_static"));
    clt_plan_path_augmented_ = this->create_client<nav_msgs::srv::GetPlan>(
      make_name("/path_planner/plan_path_with_augmented"));
    clt_plan_path_with_via_ = this->create_client<pumas_interfaces::srv::GetPlanWithVia>(
      make_name("/path_planner/plan_path_with_via"));
    clt_get_aug_map_ =
      this->create_client<nav_msgs::srv::GetMap>(make_name("/map_augmenter/get_augmented_map"));
    clt_get_aug_costmap_ = this->create_client<nav_msgs::srv::GetMap>(
      make_name("/map_augmenter/get_augmented_cost_map"));
    clt_are_there_obs_ =
      this->create_client<std_srvs::srv::Trigger>(make_name("/map_augmenter/are_there_obstacles"));
    clt_is_in_obstacles_ =
      this->create_client<std_srvs::srv::Trigger>(make_name("/map_augmenter/is_inside_obstacles"));
    clt_clear_obstacle_memory_ = this->create_client<std_srvs::srv::Trigger>(
      make_name("/map_augmenter/clear_obstacle_memory"));

    service_check_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      if (
        clt_plan_path_static_->wait_for_service(std::chrono::seconds(0)) &&
        clt_plan_path_augmented_->wait_for_service(std::chrono::seconds(0)) &&
        clt_get_aug_map_->wait_for_service(std::chrono::seconds(0)) &&
        clt_get_aug_costmap_->wait_for_service(std::chrono::seconds(0)) &&
        clt_are_there_obs_->wait_for_service(std::chrono::seconds(0)) &&
        clt_is_in_obstacles_->wait_for_service(std::chrono::seconds(0)) &&
        clt_clear_obstacle_memory_->wait_for_service(std::chrono::seconds(0))) {
        RCLCPP_INFO(
          this->get_logger(),
          "MotionPlanner.-> All motion planner clients are now "
          "available.");
        services_ready_ = true;
        service_check_timer_->cancel();
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "MotionPlanner.-> Waiting for motion planner clients "
          "to become available...");
      }
    });
  }

  // ############
  // Motion Planner callbacks
  void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg)
  {
    try {
      std::cout << "MotionPlanner.-> General Stop signal received" << std::endl;
      stop_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> Error processing callback_general_stop: %s",
        e.what());
    }
  }

  void callback_navigation_stop(const std_msgs::msg::Empty::SharedPtr msg)
  {
    try {
      std::cout << "MotionPlanner.-> Navigation Stop signal received" << std::endl;
      stop_ = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> Error processing callback_navigation_stop: %s",
        e.what());
    }
  }

  void callback_simple_move_goal_status(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
  {
    try {
      simple_move_goal_status_ = *msg;

      simple_move_status_id_ = msg->goal_id.id;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "MotionPlanner.-> Error processing "
        "callback_simple_move_goal_status: %s",
        e.what());
    }
  }

  void callback_set_patience(const std_msgs::msg::Bool::SharedPtr msg)
  {
    try {
      std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False") << std::endl;
      patience_ = msg->data;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> Error processing callback_set_patience: %s",
        e.what());
    }
  }

  void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg)
  {
    try {
      pot_fields_received_ = true;
      is_pot_fields_response_ = true;

      collision_risk_ = msg->data;
      // Latch a True so the collision-risk abort can be distinguished from a
      // generic (timeout) abort even after the robot stops and potential_fields
      // (speed-gated) goes back to False. Cleared at each path-follow start.
      if (msg->data) collision_risk_latched_ = true;
    } catch (const std::exception & e) {
      pot_fields_received_ = false;
      is_pot_fields_response_ = true;
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> Error processing callback_collision_risk: %s",
        e.what());
    }
  }

  void callback_arm_joints(const pumas_interfaces::msg::StartAndEndJoints::SharedPtr msg)
  {
    target_arm_pose = *msg;
    arm_goal_received = true;
  }

  // ############
  // Motion Planner additional functions

  // Compare the planned path endpoint with the requested goal. If they differ
  // by more than goal_relocation_report_threshold_, the planner relocated the
  // goal to the nearest reachable cell (the requested goal was enclosed /
  // unreachable). Records the flag + distance for SM_FINAL reporting.
  void detect_goal_relocation(float goal_x, float goal_y)
  {
    goal_relocated_ = false;
    goal_relocated_dist_ = 0.0;
    if (path_.poses.empty()) return;
    const auto & end = path_.poses.back().pose.position;
    double d = std::hypot(end.x - goal_x, end.y - goal_y);
    if (d > goal_relocation_report_threshold_) {
      goal_relocated_ = true;
      goal_relocated_dist_ = d;
      RCLCPP_WARN(
        this->get_logger(),
        "MotionPlanner.-> Goal appears relocated: path ends %.2f m "
        "from the requested goal (enclosed/unreachable goal).",
        d);
    }
  }

  void get_plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y)
  {
    threads_.push_back(
      std::thread(
        std::bind(
          &MotionPlannerNode::plan_path_from_augmented_map, this, robot_x, robot_y, goal_x,
          goal_y)));
  }

  void plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y)
  {
    is_path_ = false;
    is_path_response_ = false;
    goal_relocated_ = false;
    // Wait for service to be available
    if (!clt_plan_path_augmented_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "MotionPlanner.-> Service not available.");
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

      RCLCPP_INFO(
        this->get_logger(), "MotionPlanner.-> Path received successfully with size: %zu",
        path_.poses.size());
    } catch (const std::exception & e) {
      this->is_path_response_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "MotionPlanner.-> Failed to get path from "
        "/path_planner/plan_path_with_augmented: %s",
        e.what());
    }
  }

  void get_plan_path_with_via(float robot_x, float robot_y, float goal_x, float goal_y)
  {
    threads_.push_back(
      std::thread(
        std::bind(&MotionPlannerNode::plan_path_with_via, this, robot_x, robot_y, goal_x, goal_y)));
  }

  void plan_path_with_via(float robot_x, float robot_y, float goal_x, float goal_y)
  {
    is_path_ = false;
    is_path_response_ = false;
    goal_relocated_ = false;
    if (!clt_plan_path_with_via_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> plan_path_with_via service not available.");
      is_path_ = false;
      is_path_response_ = true;
      return;
    }

    auto request = std::make_shared<pumas_interfaces::srv::GetPlanWithVia::Request>();
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

    for (const auto & p : via_points_) {
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

      RCLCPP_INFO(
        this->get_logger(),
        "MotionPlanner.-> Path (via %zu) received successfully with "
        "size: %zu",
        request->via_points.size(), path_.poses.size());
    } catch (const std::exception & e) {
      this->is_path_response_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "MotionPlanner.-> Failed to get path from "
        "/path_planner/plan_path_with_via: %s",
        e.what());
    }
  }

  // Pop via points the robot has already reached (within threshold) so a
  // replan does not route back to them.
  void update_via_points()
  {
    const double thr = 0.5;  // [m]
    while (!via_points_.empty()) {
      double dx = via_points_.front().position.x - robot_x_;
      double dy = via_points_.front().position.y - robot_y_;
      if (std::sqrt(dx * dx + dy * dy) <= thr) {
        RCLCPP_INFO(
          this->get_logger(), "MotionPlanner.-> Reached via point, %zu remaining",
          via_points_.size() - 1);
        via_points_.erase(via_points_.begin());
      } else {
        break;
      }
    }
  }

  void get_robot_position()
  {
    try {
      geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

      robot_x_ = transformStamped.transform.translation.x;
      robot_y_ = transformStamped.transform.translation.y;

      tf2::Quaternion q(
        transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_t_ = static_cast<float>(yaw);

      return;
    } catch (const tf2::TransformException & ex) {
      // RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s",
      // ex.what());
      // robot_x_ = robot_y_ = robot_t_ = 0.0f;
      return;
    }
  }

  // ############
  //  Collision-recovery helpers
  void publish_detection_scale(float scale)
  {
    std_msgs::msg::Float32 m;
    m.data = scale;
    pub_detection_scale_->publish(m);
  }

  // Clear all recovery state and restore the cloud box. Called on recovery
  // escape (forward progress) and at goal boundaries.
  void reset_recovery_state()
  {
    // The convergence track is independent of the recovery ladder, but its
    // lifetime is the same: both are per-task. Riding along here means the
    // existing call sites (new goal, path succeeded, stop-short) already clear
    // it and no extra hooks are needed.
    reset_convergence_state();
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
      std::cout << "MotionPlanner.-> Recovery reset: point cloud RE-ENABLED." << std::endl;
    }
  }

  // Step the recovery tier. OFFLINE (use_online_ == false): after the shrink
  // steps comes the cloud-off tier (recovery_max_shrink_steps_ + 1); past that
  // the level wraps to tier 0 (re-attempt an escape move) while leaving the
  // scale at its floor and the cloud disabled -> never gives up. ONLINE: the
  // ladder stops at the last shrink step and wraps there, so recovery_level_
  // never reaches the cloud-off tier and the cloud is never dropped.
  void advance_recovery_level()
  {
    recovery_level_++;
    const int max_level = use_online_ ? recovery_max_shrink_steps_ : recovery_max_shrink_steps_ + 1;
    if (recovery_level_ > max_level) recovery_level_ = 0;
  }

  // ############
  //  Goal-convergence monitor (independent of the collision-recovery ladder)

  void reset_convergence_state()
  {
    best_goal_dist_ = std::numeric_limits<float>::max();
    convergence_clock_valid_ = false;
    replan_events_.clear();
    replan_total_ = 0;
    converge_level_ = 0;
    convergence_reason_.clear();
    plan_start_valid_ = false;
  }

  // Is this a state in which the robot is supposed to be closing on the goal?
  // The clock is frozen everywhere else, so a patience wait (a person standing
  // in the way) and the collision-recovery ladder cannot be mistaken for the
  // goal failing to converge.
  static bool convergence_clock_runs(int s)
  {
    return s == SM_CALCULATE_PATH || s == SM_WAIT_FOR_PATH_RESPONSE || s == SM_START_MOVE_PATH ||
           s == SM_WAIT_FOR_MOVE_FINISHED;
  }

  // Signal A. Fed from SM_WAIT_FOR_MOVE_FINISHED, which already computes the
  // distance to the goal every cycle.
  void note_goal_distance(float dist_to_goal)
  {
    if (!convergence_clock_valid_) {
      best_goal_dist_ = dist_to_goal;
      last_progress_time_ = this->now();
      convergence_clock_valid_ = true;
      return;
    }
    if (dist_to_goal < best_goal_dist_ - static_cast<float>(convergence_epsilon_)) {
      best_goal_dist_ = dist_to_goal;
      last_progress_time_ = this->now();
    }
  }

  // Signal B. Recorded on every entry into SM_CALCULATE_PATH.
  void note_replan(float x, float y)
  {
    const rclcpp::Time now = this->now();
    replan_events_.push_back(ReplanEvent{now, x, y});
    replan_total_++;
    while (!replan_events_.empty() &&
           (now - replan_events_.front().t).seconds() > replan_window_sec_) {
      replan_events_.pop_front();
    }
  }

  // Widest separation between any two replan starts still inside the window.
  // Using the whole set rather than consecutive pairs is the entire point: a
  // robot shuttling between two spots 1.2 m apart has a spread of 1.2 m and is
  // caught, where a previous-point comparison resets on every single hop.
  double replan_spread() const
  {
    double worst = 0.0;
    for (size_t i = 0; i < replan_events_.size(); ++i) {
      for (size_t j = i + 1; j < replan_events_.size(); ++j) {
        const double d = std::hypot(
          replan_events_[i].x - replan_events_[j].x, replan_events_[i].y - replan_events_[j].y);
        if (d > worst) worst = d;
      }
    }
    return worst;
  }

  // Any of the three signals. Sets convergence_reason_ so the operator can see
  // which one fired without reading the thresholds back.
  bool goal_not_converging()
  {
    if (
      convergence_clock_valid_ &&
      (this->now() - last_progress_time_).seconds() > convergence_timeout_) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << "no progress for "
          << (this->now() - last_progress_time_).seconds() << " s (best " << best_goal_dist_
          << " m)";
      convergence_reason_ = oss.str();
      return true;
    }

    if (static_cast<int>(replan_events_.size()) >= replan_window_max_) {
      const double spread = replan_spread();
      if (spread <= replan_spread_radius_) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << replan_events_.size() << " replans within "
            << replan_window_sec_ << " s, spread " << spread << " m";
        convergence_reason_ = oss.str();
        return true;
      }
    }

    if (replan_total_ > replan_max_per_task_) {
      std::ostringstream oss;
      oss << replan_total_ << " replans in this task";
      convergence_reason_ = oss.str();
      return true;
    }

    return false;
  }

  // Distance from the robot to the REQUESTED goal, measured ALONG the path the
  // last plan returned instead of straight through walls. Straight-line
  // distance is the wrong measure for the near-goal gate below: a goal one
  // metre away on the far side of a wall reads as "practically arrived", and
  // the task is ended while the robot still has the whole detour left to drive.
  //
  //   = robot -> its nearest point on the path   (lateral offset)
  //   + that point -> the path endpoint          (along the path)
  //   + the path endpoint -> the requested goal  (straight)
  //
  // The last term is the residual left when the planner moved the endpoint off
  // a blocked goal cell; it is the same quantity as goal_relocated_dist_, so it
  // cancels against the radius widening in suppress_replan_near_goal() rather
  // than double-counting there.
  //
  // Returns false when the last plan came back empty and there is no path to
  // measure along -- see the caller for why that must not end a task.
  bool goal_distance_along_path(double & out) const
  {
    const auto & poses = path_.poses;
    if (poses.size() < 2) return false;

    // Nearest point on the polyline: project the robot onto every segment,
    // clamped to the segment, and keep the closest.
    size_t best_i = 0;
    double best_t = 0.0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
      const auto & a = poses[i].pose.position;
      const auto & b = poses[i + 1].pose.position;
      const double vx = b.x - a.x, vy = b.y - a.y;
      const double len2 = vx * vx + vy * vy;
      double t = 0.0;
      if (len2 > 1e-12) {
        t = ((robot_x_ - a.x) * vx + (robot_y_ - a.y) * vy) / len2;
        t = std::max(0.0, std::min(1.0, t));
      }
      const double px = a.x + t * vx, py = a.y + t * vy;
      const double d2 = (robot_x_ - px) * (robot_x_ - px) + (robot_y_ - py) * (robot_y_ - py);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_i = i;
        best_t = t;
      }
    }

    // Getting back onto the path counts: potential fields push the robot off it.
    double d = std::sqrt(best_d2);

    // Remainder of the segment the robot sits on, then every segment after it.
    {
      const auto & a = poses[best_i].pose.position;
      const auto & b = poses[best_i + 1].pose.position;
      d += (1.0 - best_t) * std::hypot(b.x - a.x, b.y - a.y);
    }
    for (size_t i = best_i + 1; i + 1 < poses.size(); ++i) {
      const auto & a = poses[i].pose.position;
      const auto & b = poses[i + 1].pose.position;
      d += std::hypot(b.x - a.x, b.y - a.y);
    }

    // Endpoint -> requested goal; non-zero exactly when the goal was relocated.
    const auto & end = poses.back().pose.position;
    d += std::hypot(global_goal_.position.x - end.x, global_goal_.position.y - end.y);

    out = d;
    return true;
  }

  // Near-goal replan gate. Runs at the top of SM_CALCULATE_PATH for every plan
  // except the first of a task. Once the robot is within min_reach_goal_dist_
  // of the goal AND the planner has relocated that goal, a replan cannot improve
  // anything: the path is a few centimetres long and the planner just relocates
  // the endpoint slightly differently every cycle, so the robot shuffles around
  // the obstacle forever. Ends the task instead — accepting the current point as
  // the goal, or aborting, per near_goal_accept_as_goal_. Returns true when it
  // did, in which case the caller must not plan.
  //
  // Proximity by itself is deliberately NOT enough (near_goal_require_relocation_,
  // default true): a free goal with a person standing in front of it produces
  // exactly the same replan, and abandoning it there was the old failure mode.
  //
  // "Within min_reach_goal_dist_" is measured as a PATH LENGTH by
  // goal_distance_along_path(), not as a straight line: what decides whether a
  // replan can still achieve anything is how far the robot must still DRIVE,
  // and a goal a metre away through a wall is metres of driving away. When no
  // path is available to measure along, the gate does not fire at all.
  //
  // This is the replan-side counterpart of the caller-requested goal_distance_
  // stop-short in SM_WAIT_FOR_MOVE_FINISHED (which stays straight-line, being a
  // caller-facing tolerance), and uses the same shutdown sequence.
  bool suppress_replan_near_goal()
  {
    // Per-goal value wins; 0 (or unset) falls back to the node parameter, and
    // a node parameter of 0 disables the gate entirely.
    const double min_dist = (goal_min_reach_dist_ > 0.0f)
                              ? static_cast<double>(goal_min_reach_dist_)
                              : min_reach_goal_dist_;
    if (min_dist <= 0.0) return false;
    // First plan of a task is never suppressed, so a goal requested from right
    // next to the robot is still planned and driven. The latch keeps the gate
    // armed afterwards even though reset_recovery_state() zeroes replan_total_.
    if (replan_total_ <= 0 && !near_goal_final_accepted_) return false;

    // Measured along the path, so "near" cannot mean "near through a wall".
    double dist_to_goal = 0.0;
    if (!goal_distance_along_path(dist_to_goal)) {
      // The last plan came back empty (planning failure), so there is no path
      // to measure along. Never end a task on a distance we could not measure:
      // let SM_CALCULATE_PATH plan again. If that never converges, the
      // goal-convergence monitor is the backstop that terminates the task.
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "MotionPlanner.-> No path from the last plan; skipping the near-goal gate.");
      return false;
    }

    // A relocated goal is one sitting on an obstacle: the nearest reachable
    // point is goal_relocated_dist_ away from it, so that is as close as the
    // robot can ever get. Widen the radius by that much rather than demand a
    // proximity that is geometrically impossible. dist_to_goal carries the same
    // endpoint->goal residual, so the two cancel and the comparison is really
    // "path length to the reachable endpoint vs min_dist".
    double radius = min_dist;
    if (goal_relocated_) radius += goal_relocated_dist_;
    if (dist_to_goal > radius) return false;

    // Proximity alone is NOT evidence that the goal is unreachable. A replan
    // right next to a perfectly free goal is the ordinary case: something (a
    // person, a chair) got in the way and collision_risk aborted the move. The
    // only evidence that the requested point itself is blocked is the planner
    // RELOCATING it -- goal_relocated_, set by detect_goal_relocation() when the
    // path endpoint lands further than goal_relocation_report_threshold_ from
    // the request. Not relocated => the goal cell is free => keep planning and
    // let the robot finish the approach once the obstacle moves.
    //
    // Note the value read here belongs to the LAST COMPLETED plan: this runs at
    // the top of SM_CALCULATE_PATH, before the next planning call clears the
    // flag. That is intended -- it is the freshest verdict available, and for a
    // goal genuinely sitting on an obstacle the flag is already set by the plan
    // that brought the robot into the radius, so nothing is delayed.
    //
    // Giving up is no longer guaranteed here, so the terminator for the
    // never-converging case is the goal-convergence monitor: goal relax
    // (SUCCEEDED) or, failing that, the stuck abort.
    // near_goal_final_accepted_ bypasses the check: the gate already accepted
    // this point and only came back for the post-final-angle second pass. That
    // decision is not re-opened.
    if (near_goal_require_relocation_ && !near_goal_final_accepted_) {
      if (!goal_relocated_) {
        // The goal cell itself is free -- what is in the way is an obstacle,
        // not the goal. Keep planning and track the path all the way in.
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "MotionPlanner.-> Near goal (%.2f m along the path) but the goal was not "
          "relocated; the obstacle is in the way, not on the goal. Continuing to plan.",
          dist_to_goal);
        return false;
      }

      // The goal cell itself is blocked, so the robot can never converge on it.
      // Deliberately does NOT return here: fall through to the acceptance /
      // abort below, which is what stops the replan loop.
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "MotionPlanner.-> Near goal (%.2f m along the path) and the goal was relocated "
        "(%.2f m); the goal cell itself is blocked. Stopping replanning.",
        dist_to_goal, goal_relocated_dist_);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "Near goal (" << dist_to_goal
        << " m along the path, no-replan radius " << radius << " m)";
    if (goal_relocated_) oss << "; requested goal is on an obstacle";
    const std::string reason = oss.str();

    // Common shutdown: halt the base, drop potential fields, clear recovery.
    pub_simple_move_stop_->publish(std_msgs::msg::Empty());
    msg_bool.data = false;
    pub_pot_fields_enable_->publish(msg_bool);
    reset_recovery_state();

    if (!near_goal_accept_as_goal_) {
      const std::string msg = reason + "; aborted instead of replanning";
      std::cout << "MotionPlanner.-> " << msg << std::endl;
      publish_nav_feedback("NEAR_GOAL_ABORT", msg);
      current_status = publish_status(actionlib_msgs::msg::GoalStatus::ABORTED, goal_id, msg);
      finish_action_abort(msg, PumasNav::Result::OUTCOME_NEAR_GOAL_ABORT);
      state = SM_INIT;
      return true;
    }

    near_goal_sent = true;
    pending_final_message_ = reason + "; accepted the current point as the goal";
    pending_final_outcome_ = PumasNav::Result::OUTCOME_NEAR_GOAL_ACCEPTED;
    publish_nav_feedback("NEAR_GOAL_ACCEPTED", pending_final_message_);

    if (near_goal_final_accepted_) {
      // Second pass: the final-angle correction already ran and reported an
      // abort. Do not loop on it — finish here.
      std::cout << "MotionPlanner.-> " << pending_final_message_ << " (final angle skipped)."
                << std::endl;
      state = SM_FINAL;
      return true;
    }
    near_goal_final_accepted_ = true;
    std::cout << "MotionPlanner.-> " << pending_final_message_ << "; correcting final angle."
              << std::endl;
    state = SM_CORRECT_FINAL_ANGLE;
    return true;
  }

  // Strict free-cell test for choosing a planning start. Deliberately NOT
  // map_cell_blocked(), which is fail-open (unknown reads as navigable) because
  // it guards short escape moves; a planning start must be known free.
  static bool cell_strictly_free(const nav_msgs::msg::OccupancyGrid & map, float wx, float wy)
  {
    const auto & info = map.info;
    if (info.resolution <= 0.0f || map.data.empty()) return false;
    const int cx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    const int cy = static_cast<int>((wy - info.origin.position.y) / info.resolution);
    if (
      cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height))
      return false;
    const size_t idx = static_cast<size_t>(cy) * info.width + cx;
    return idx < map.data.size() && map.data[idx] == 0;
  }

  // Ring search outward from (cx,cy) for the nearest known-free cell, so a
  // robot sitting on an occupied cell can be planned out of it without moving
  // first. Only the boundary of each ring is scanned, so no cell is visited
  // twice and the first hit is the nearest in Chebyshev order.
  bool find_nearest_free_point(float cx, float cy, float & fx, float & fy) const
  {
    if (!recovery_map_ready_ || recovery_map_.info.resolution <= 0.0f || recovery_map_.data.empty())
      return false;

    const float res = recovery_map_.info.resolution;
    const int max_radius = static_cast<int>(free_point_search_radius_ / res);

    for (int radius = 0; radius <= max_radius; ++radius) {
      for (int i = -radius; i <= radius; ++i) {
        for (int j = -radius; j <= radius; ++j) {
          if (std::abs(i) != radius && std::abs(j) != radius)
            continue;  // interior already covered by a smaller radius
          const float tx = cx + i * res;
          const float ty = cy + j * res;
          if (cell_strictly_free(recovery_map_, tx, ty)) {
            fx = tx;
            fy = ty;
            RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Found free start %.2f m from the "
              "robot [%.2f, %.2f].",
              std::hypot(i * res, j * res), tx, ty);
            return true;
          }
        }
      }
    }
    return false;
  }

  // Asynchronously fetch the augmented occupancy map for the rear check.
  void request_augmented_map()
  {
    recovery_map_response_ = false;
    recovery_map_ready_ = false;
    if (!clt_get_aug_map_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> get_augmented_map service unavailable.");
      recovery_map_response_ = true;  // no map -> rear treated as unknown
      return;
    }
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_aug_map_->async_send_request(
      request, [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
        try {
          recovery_map_ = future.get()->map;
          recovery_map_ready_ = true;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(), "MotionPlanner.-> Failed to get augmented map: %s", e.what());
          recovery_map_ready_ = false;
        }
        recovery_map_response_ = true;
      });
  }

  // Is the augmented-map cell at world (px,py) KNOWN-occupied (>0)? Fail-OPEN:
  // missing map, off-map and unknown (-1) all return false (not blocked), so a
  // short escape move is never deadlocked by the map being incomplete.
  bool map_cell_blocked(float px, float py) const
  {
    const auto & info = recovery_map_.info;
    if (info.resolution <= 0.0f || recovery_map_.data.empty()) return false;
    const int cx = static_cast<int>((px - info.origin.position.x) / info.resolution);
    const int cy = static_cast<int>((py - info.origin.position.y) / info.resolution);
    if (
      cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height))
      return false;
    const int cell = cy * static_cast<int>(info.width) + cx;
    if (cell < 0 || cell >= static_cast<int>(recovery_map_.data.size())) return false;
    return recovery_map_.data[cell] > 0;
  }

  // Is a relative move (rel_x fwd+, rel_y left+, base frame) clear? Checks the
  // destination plus ± footprint offsets perpendicular to the motion.
  bool dir_clear(float rel_x, float rel_y) const
  {
    const float ct = cos(robot_t_), st = sin(robot_t_);
    const float len = std::hypot(rel_x, rel_y);
    const float ux = (len > 1e-3f) ? rel_x / len : 0.0f;
    const float uy = (len > 1e-3f) ? rel_y / len : 0.0f;
    const float perpx = -uy, perpy = ux;  // perpendicular to motion
    const float offs[3] = {-recovery_rear_lateral_, 0.0f, recovery_rear_lateral_};
    for (float o : offs) {
      const float bx = rel_x + perpx * o;
      const float by = rel_y + perpy * o;
      const float px = robot_x_ + bx * ct - by * st;
      const float py = robot_y_ + bx * st + by * ct;
      if (map_cell_blocked(px, py)) return false;
    }
    return true;
  }

  // Choose direction escape move (base frame [x fwd+, y left+])
  bool choose_escape_move(float & rel_x, float & rel_y)
  {
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

    const float D = recovery_back_dist_;  // 15 cm
    const float Dd = D * 0.70710678f;     // diagonal component (|move| = D)
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
    for (const auto & m : cands) {
      if (dir_clear(m[0], m[1])) {
        rel_x = m[0];
        rel_y = m[1];
        return true;
      }
    }
    return false;
  }

  int publish_status(int status, int id, const std::string & text)
  {
    actionlib_msgs::msg::GoalStatus msg;
    std::stringstream ss;
    ss << id;
    ss >> msg.goal_id.id;
    msg.status = status;
    msg.text = text;
    // Node clock, not a default-constructed rclcpp::Clock: that one is
    // RCL_SYSTEM_TIME regardless of use_sim_time, so under a simulator this
    // stamp alone would sit ~1.7e9 s ahead of every other message.
    msg.goal_id.stamp = this->now();

    pub_status_->publish(msg);

    return status;
  }

  void send_motion_synth_goal(
    const geometry_msgs::msg::Pose & nav_goal_pose,
    const pumas_interfaces::msg::StartAndEndJoints & arm_joints)
  {
    auto goal_msg = MotionSynthesis::Goal();
    goal_msg.goal_location.x = nav_goal_pose.position.x;
    goal_msg.goal_location.y = nav_goal_pose.position.y;

    tf2::Quaternion q(
      nav_goal_pose.orientation.x, nav_goal_pose.orientation.y, nav_goal_pose.orientation.z,
      nav_goal_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal_msg.goal_location.theta = static_cast<float>(yaw);

    goal_msg.apply_start_pose = arm_joints.has_arm_start_pose;
    goal_msg.apply_goal_pose = arm_joints.has_arm_end_pose;
    goal_msg.start_pose = arm_joints.start_pose;
    goal_msg.goal_pose = arm_joints.end_pose;
    goal_msg.motion_execution_time = arm_joints.motion_execution_time;

    auto send_goal_options = rclcpp_action::Client<MotionSynthesis>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleMotionSynthesis> goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(this->get_logger(), "motion_synth goal was rejected by server");
          motion_synth_goal_handle_.reset();
        } else {
          motion_synth_goal_handle_ = goal_handle;
        }
      };
    send_goal_options.result_callback =
      [this](const GoalHandleMotionSynthesis::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "motion_synth action SUCCEEDED");
        } else {
          RCLCPP_WARN(
            this->get_logger(), "motion_synth action failed or canceled (code: %d)",
            static_cast<int>(result.code));
        }
        motion_synth_goal_handle_.reset();
      };

    motion_synth_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void request_clear_obstacle_memory()
  {
    if (clear_memory_request_sent_) {
      return;
    }

    clear_memory_request_sent_ = true;
    clear_memory_response_received_ = false;
    clear_memory_success_ = false;
    clear_memory_failed_ = false;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    clt_clear_obstacle_memory_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          auto response = future.get();
          clear_memory_success_ = response->success;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "MotionPlanner.-> Failed to call "
            "clear_obstacle_memory: %s",
            e.what());
          clear_memory_failed_ = true;
        }

        clear_memory_response_received_ = true;
        clear_memory_request_sent_ = false;
      });
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePumasNav> goal_handle)
  {
    // Must run before anything below overwrites active_goal_handle_: an
    // accepted goal handle that is never terminated leaks inside the action
    // server and its client waits on a result that never arrives.
    //
    // No locking: the action server and the 30 Hz processing timer both live in
    // the node's default (mutually exclusive) callback group, so this cannot
    // interleave with motion_planner_processing even under the
    // MultiThreadedExecutor in main().
    preempt_active_goal();

    active_goal_handle_ = goal_handle;
    action_active_ = true;
    cancel_requested_ = false;

    const auto goal = goal_handle->get_goal();

    global_goal_ = goal->goal.pose;
    new_global_goal_ = true;

    // Via points for this goal (empty -> plain A* in SM_CALCULATE_PATH).
    via_points_ = goal->via_points.poses;
    RCLCPP_INFO(
      this->get_logger(), "MotionPlanner.-> Goal with %zu via points", via_points_.size());

    // Stop-short distance for this goal (<= 0 disables; full arrival).
    goal_distance_ = goal->goal_distance;

    // Per-goal near-goal replan radius (<= 0 -> use the node parameter).
    goal_min_reach_dist_ = goal->min_reach_goal_dist;
    if (goal_min_reach_dist_ > 0.0f) {
      RCLCPP_INFO(
        this->get_logger(), "MotionPlanner.-> Goal overrides min_reach_goal_dist: %.2f m",
        goal_min_reach_dist_);
    }

    if (goal->use_arm) {
      target_arm_pose = goal->arm_joints;
      arm_goal_received = true;
    } else {
      arm_goal_received = false;
    }

    near_goal_sent = false;
  }

  // Name and one-line description of every state, so the periodic feedback can
  // report where the state machine is without each case having to remember to
  // publish. Termination tags (STOP_SHORT, GOAL_RELAXED, ...) are NOT here:
  // those are events, published explicitly at the branch that raises them.
  static void state_info(int s, const char *& name, const char *& message)
  {
    switch (s) {
        // clang-format off
      case SM_INIT:                             name = "INIT";                       message = "Idle"; return;
      case SM_WAITING_FOR_TASK:                 name = "WAITING_FOR_TASK";           message = "Waiting for task"; return;
      case SM_CALCULATE_PATH:                   name = "CALCULATE_PATH";             message = "Calculating path"; return;
      case SM_WAIT_FOR_PATH_RESPONSE:           name = "WAIT_FOR_PATH_RESPONSE";     message = "Waiting for the path planner"; return;
      case SM_WAIT_FOR_CLEAR_MEMORY_RESPONSE:   name = "WAIT_FOR_CLEAR_MEMORY";      message = "Clearing remembered obstacles"; return;
      case SM_CHECK_IF_INSIDE_OBSTACLES:        name = "CHECK_INSIDE_OBSTACLES";     message = "Checking whether the robot sits on an obstacle"; return;
      case SM_WAIT_FOR_INSIDE_OBSTACLES_RESPONSE: name = "WAIT_INSIDE_OBSTACLES";    message = "Checking whether the robot sits on an obstacle"; return;
      case SM_WAITING_FOR_MOVE_BACKWARDS:       name = "MOVE_BACKWARDS";             message = "Backing out of an obstacle"; return;
      case SM_CHECK_IF_OBSTACLES:               name = "CHECK_OBSTACLES";            message = "Checking for temporal obstacles"; return;
      case SM_WAIT_FOR_IF_OBSTACLES_RESPONSE:   name = "WAIT_CHECK_OBSTACLES";       message = "Checking for temporal obstacles"; return;
      case SM_WAIT_FOR_NO_OBSTACLES:            name = "WAIT_FOR_NO_OBSTACLES";      message = "Waiting for temporal obstacles to move"; return;
      case SM_WAIT_FOR_NO_OBSTACLES_DELAY:      name = "WAIT_FOR_NO_OBSTACLES";      message = "Waiting for temporal obstacles to move"; return;
      case SM_WAIT_FOR_NO_OBSTACLES_RESPONSE:   name = "WAIT_FOR_NO_OBSTACLES";      message = "Waiting for temporal obstacles to move"; return;
      case SM_ENABLE_POT_FIELDS:                name = "ENABLE_POT_FIELDS";          message = "Enabling potential fields"; return;
      case SM_WAIT_FOR_POT_FIELDS:              name = "WAIT_FOR_POT_FIELDS";        message = "Waiting for potential fields"; return;
      case SM_WAIT_FOR_NOT_POT_FIELDS:          name = "WAIT_FOR_NOT_POT_FIELDS";    message = "Disabling potential fields"; return;
      case SM_START_MOVE_PATH:                  name = "START_MOVE_PATH";            message = "Starting to follow the path"; return;
      case SM_WAIT_FOR_MOVE_FINISHED:           name = "WAIT_FOR_MOVE_FINISHED";     message = "Following path"; return;
      case SM_COLLISION_RECOVERY:               name = "COLLISION_RECOVERY";         message = "Recovering from a collision risk"; return;
      case SM_RECOVERY_WAIT_MAP:                name = "RECOVERY_WAIT_MAP";          message = "Looking for an escape direction"; return;
      case SM_RECOVERY_WAIT_BACKUP:             name = "RECOVERY_WAIT_BACKUP";       message = "Backing away from the obstacle"; return;
      case SM_RECOVERY_WAIT_ROTATE:             name = "RECOVERY_WAIT_ROTATE";       message = "Rotating to find a way out"; return;
      case SM_GOAL_CONVERGENCE_CHECK:           name = "GOAL_CONVERGENCE_CHECK";     message = "Goal is not converging"; return;
      case SM_CONVERGENCE_WAIT_CLEAR_MEM:       name = "CONVERGENCE_CLEAR_MEMORY";   message = "Clearing remembered obstacles"; return;
      case SM_CONVERGENCE_WAIT_FREE_MAP:        name = "CONVERGENCE_WAIT_MAP";       message = "Looking for a free cell to plan from"; return;
      case SM_CORRECT_FINAL_ANGLE:              name = "CORRECT_FINAL_ANGLE";        message = "Correcting the final angle"; return;
      case SM_WAIT_FOR_ANGLE_CORRECTED:         name = "WAIT_FOR_ANGLE_CORRECTED";   message = "Correcting the final angle"; return;
      case SM_FINAL:                            name = "FINAL";                      message = "Finishing"; return;
      default:                                  name = "UNKNOWN";                    message = ""; return;
        // clang-format on
    }
  }

  // Called once per tick from motion_planner_processing() so that feedback
  // flows at the full loop rate in EVERY state. Before this, only three states
  // published anything and a client saw multi-second silences during obstacle
  // waits, potential-field waits and the whole collision-recovery ladder.
  void publish_periodic_feedback()
  {
    const char * name = nullptr;
    const char * message = nullptr;
    state_info(state, name, message);
    publish_nav_feedback(name, message);
  }

  void publish_nav_feedback(const std::string & state_name, const std::string & message = "")
  {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto feedback = std::make_shared<PumasNav::Feedback>();

    get_robot_position();
    float remaining_distance = std::sqrt(
      std::pow(global_goal_.position.x - robot_x_, 2) +
      std::pow(global_goal_.position.y - robot_y_, 2));

    feedback->state_name = state_name;
    feedback->remaining_distance = remaining_distance;
    feedback->collision_risk = collision_risk_;
    feedback->near_goal_reached = near_goal_sent;
    feedback->message = message;

    // Settings in force for this goal, so a client can interpret the numbers
    // above without reaching for our parameters. min_reach_goal_dist is the
    // RESOLVED value -- same expression as suppress_replan_near_goal().
    feedback->proximity_criterion = proximity_criterion_;
    feedback->min_reach_goal_dist = (goal_min_reach_dist_ > 0.0f)
                                      ? goal_min_reach_dist_
                                      : static_cast<float>(min_reach_goal_dist_);
    feedback->goal_distance = goal_distance_;

    active_goal_handle_->publish_feedback(feedback);
  }

  void set_map_augmenter_enable(bool enable)
  {
    std_msgs::msg::Bool msg;
    msg.data = enable;
    pub_map_aug_enable_->publish(msg);
  }

  // Terminate the running goal because a newer one is taking over. Called from
  // handle_accepted BEFORE the new goal's fields are installed.
  //
  // Deliberately different from the finish_action_* helpers: the map augmenter
  // stays enabled (SM_WAITING_FOR_TASK re-enables it one tick later anyway) and
  // via_points_ is left alone because handle_accepted overwrites it immediately.
  //
  // The base IS stopped here. simple_move still holds the old path, and if
  // planning for the new goal fails the state machine parks in a patience /
  // obstacle wait without ever republishing a stop -- the robot would keep
  // driving the abandoned path. One stop message costs a moment of motion and
  // removes that hazard.
  void preempt_active_goal()
  {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = false;
    result->near_goal_reached = near_goal_sent;
    result->message = "Preempted by a new goal";
    result->outcome = PumasNav::Result::OUTCOME_PREEMPTED;

    // A goal the client already asked to cancel must be terminated as CANCELED;
    // rclcpp_action rejects abort() on a handle in the CANCELING state.
    if (active_goal_handle_->is_canceling()) {
      active_goal_handle_->canceled(result);
    } else {
      active_goal_handle_->abort(result);
    }

    active_goal_handle_.reset();
    action_active_ = false;

    pub_simple_move_stop_->publish(std_msgs::msg::Empty());
    std_msgs::msg::Bool msg_bool;
    msg_bool.data = false;
    pub_pot_fields_enable_->publish(msg_bool);
    reset_recovery_state();

    // Drop a stop/cancel that was raised against the goal just terminated:
    // leaving it set would make the next 30 Hz tick abort the incoming goal.
    stop_ = false;
    cancel_requested_ = false;

    std::cout << "MotionPlanner.-> Active goal preempted by a new goal." << std::endl;
  }

  void finish_action_success(const std::string & message, uint8_t outcome)
  {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = true;
    result->near_goal_reached = near_goal_sent;
    result->message = message;
    result->outcome = outcome;

    active_goal_handle_->succeed(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  void finish_action_abort(const std::string & message, uint8_t outcome)
  {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = false;
    result->near_goal_reached = near_goal_sent;
    result->message = message;
    result->outcome = outcome;

    active_goal_handle_->abort(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  void finish_action_cancel(const std::string & message, uint8_t outcome)
  {
    if (!action_active_ || !active_goal_handle_) {
      return;
    }

    auto result = std::make_shared<PumasNav::Result>();
    result->success = false;
    result->near_goal_reached = near_goal_sent;
    result->message = message;
    result->outcome = outcome;

    active_goal_handle_->canceled(result);
    active_goal_handle_.reset();
    action_active_ = false;
    cancel_requested_ = false;
    via_points_.clear();
    set_map_augmenter_enable(false);
  }

  // ############
  // motion planner main processing
  void motion_planner_processing()
  {
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
            finish_action_cancel("Navigation task canceled", PumasNav::Result::OUTCOME_CANCELED);
          } else {
            finish_action_abort(
              "Stop signal received. Task aborted", PumasNav::Result::OUTCOME_STOP_SIGNAL);
          }
        }

        if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
          current_status = publish_status(
            actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
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

      // Goal-convergence gate. Evaluated only in states where the robot is
      // meant to be closing on the goal, so it never pre-empts the collision
      // recovery ladder or a deliberate patience wait.
      if (action_active_ && convergence_clock_runs(state) && goal_not_converging()) {
        RCLCPP_WARN(
          this->get_logger(),
          "MotionPlanner.-> Goal not converging (%s). Entering "
          "convergence escalation tier %d.",
          convergence_reason_.c_str(), converge_level_);
        state = SM_GOAL_CONVERGENCE_CHECK;
      }

      // One feedback per tick, whatever the state. A case below may publish
      // again with an event tag (STOP_SHORT, NEAR_GOAL, ...); that one lands
      // last and is what the client sees for this tick.
      publish_periodic_feedback();

      switch (state) {
        case SM_INIT: {
          std::cout << "MotionPlanner.-> MVN PLN READY. Waiting for new goal. " << std::endl;
          state = SM_WAITING_FOR_TASK;
          break;
        }

        case SM_WAITING_FOR_TASK: {
          if (new_global_goal_) {
            new_global_goal_ = false;
            set_map_augmenter_enable(true);
            state = SM_CALCULATE_PATH;
            if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE)
              current_status = publish_status(
                actionlib_msgs::msg::GoalStatus::ABORTED, goal_id, "Cancelling current movement.");
            goal_id++;
            std::cout << "MotionPlanner.-> New goal received. Current task goal_id: " << goal_id
                      << std::endl;
            current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id, "Starting new movement task");
            near_goal_sent = false;
            goal_relocated_ = false;
            near_goal_final_accepted_ = false;
            pending_final_message_.clear();
            pending_final_outcome_ = PumasNav::Result::OUTCOME_UNKNOWN;
            reset_recovery_state();  // clean slate per task (hygiene)
          }
          break;
        }

        case SM_CALCULATE_PATH: {
          get_robot_position();

          // Do not replan from right next to the goal: there is nothing left to
          // plan, and with the goal on an obstacle this is exactly the loop that
          // keeps relocating it a few centimetres away forever.
          if (suppress_replan_near_goal()) break;

          update_via_points();
          note_replan(robot_x_, robot_y_);

          // Normally the plan starts at the robot. It starts elsewhere only when
          // the robot was found sitting on an occupied cell and a nearby free
          // cell was substituted (see SM_CONVERGENCE_WAIT_FREE_MAP). One-shot.
          float start_x = robot_x_;
          float start_y = robot_y_;
          if (plan_start_valid_) {
            start_x = plan_start_x_;
            start_y = plan_start_y_;
            plan_start_valid_ = false;
            std::cout << "MotionPlanner.-> Planning from substituted free start [" << start_x
                      << ", " << start_y << "] instead of the robot [" << robot_x_ << ", "
                      << robot_y_ << "]." << std::endl;
          }

          if (!via_points_.empty()) {
            get_plan_path_with_via(
              start_x, start_y, global_goal_.position.x, global_goal_.position.y);
          } else {
            get_plan_path_from_augmented_map(
              start_x, start_y, global_goal_.position.x, global_goal_.position.y);
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
            std::cout << "MotionPlanner.-> Cannot calc path to " << global_goal_.position.x << " "
                      << global_goal_.position.y << std::endl;
            pub_simple_move_stop_->publish(std_msgs::msg::Empty());

            if (remember_all_obstacles_) {
              if (!clt_clear_obstacle_memory_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_ERROR(
                  this->get_logger(),
                  "MotionPlanner.-> clear_obstacle_memory "
                  "service not available; falling back.");
                state = patience_ ? SM_CHECK_IF_OBSTACLES : SM_CHECK_IF_INSIDE_OBSTACLES;
              } else {
                request_clear_obstacle_memory();
                state = SM_WAIT_FOR_CLEAR_MEMORY_RESPONSE;
              }
            } else {
              state = patience_ ? SM_CHECK_IF_OBSTACLES : SM_CHECK_IF_INSIDE_OBSTACLES;
            }
          } else {
            if (is_temporary_no_cloud_pot_fields_) {
              no_cloud_pot_fields_start_time_ = this->now();  // reset timer
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
            RCLCPP_WARN(this->get_logger(), "MotionPlanner.-> clear_obstacle_memory call failed.");
          } else if (clear_memory_success_) {
            RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> Cleared memory obstacles due to path "
              "planning failure; retrying path calculation.");
          } else {
            RCLCPP_WARN(
              this->get_logger(), "MotionPlanner.-> clear_obstacle_memory returned false.");
          }

          state = SM_CALCULATE_PATH;
          break;
        }

        // ####################################################################
        //  Goal-convergence escalation. A separate ladder from the collision
        //  recovery one: it does not touch recovery_level_ or
        //  collision_event_count_, and it is about the goal never converging
        //  rather than about an obstacle being in the way right now.
        // ####################################################################
        case SM_GOAL_CONVERGENCE_CHECK: {
          publish_nav_feedback("GOAL_NOT_CONVERGING", convergence_reason_);

          if (converge_level_ == 0) {
            // Tier 0: stale remembered obstacles are the most common reason a
            // reachable goal stops converging. Wipe them and give the planner a
            // clean run before doing anything the caller can observe.
            converge_level_ = 1;
            // Restart the clock so the retry gets a full budget of its own.
            convergence_clock_valid_ = false;
            replan_events_.clear();

            if (clt_clear_obstacle_memory_->wait_for_service(std::chrono::milliseconds(200))) {
              std::cout << "MotionPlanner.-> Convergence tier 0: clearing "
                           "obstacle memory and retrying."
                        << std::endl;
              request_clear_obstacle_memory();
              state = SM_CONVERGENCE_WAIT_CLEAR_MEM;
            } else {
              RCLCPP_WARN(
                this->get_logger(),
                "MotionPlanner.-> clear_obstacle_memory unavailable; "
                "skipping convergence tier 0.");
              state = SM_CALCULATE_PATH;
            }
            break;
          }

          if (
            converge_level_ == 1 && enable_goal_relax_ &&
            best_goal_dist_ <= static_cast<float>(goal_relax_max_)) {
            // Tier 1: the robot got close but cannot close the last stretch.
            // Converge the goal instead of looping: accept the best position
            // reached as arrival. Same shutdown sequence as the stop-short
            // branch in SM_WAIT_FOR_MOVE_FINISHED.
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << "Goal relaxed: stopped " << best_goal_dist_
                << " m from goal after failing to converge (" << convergence_reason_ << ")";
            const std::string msg = oss.str();
            std::cout << "MotionPlanner.-> " << msg << std::endl;

            pub_simple_move_stop_->publish(std_msgs::msg::Empty());
            msg_bool.data = false;
            pub_pot_fields_enable_->publish(msg_bool);
            reset_recovery_state();
            near_goal_sent = true;
            publish_nav_feedback("GOAL_RELAXED", msg);
            current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::SUCCEEDED, goal_id, msg);
            finish_action_success(msg, PumasNav::Result::OUTCOME_GOAL_RELAXED);
            state = SM_INIT;
            break;
          }

          // Tier 2: never got near the goal, or relaxation is disabled.
          if (enable_stuck_abort_) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "goal_not_converging: " << convergence_reason_ << ", best " << best_goal_dist_
                << " m, " << replan_total_ << " replans";
            const std::string msg = oss.str();
            std::cout << "MotionPlanner.-> " << msg << std::endl;

            pub_simple_move_stop_->publish(std_msgs::msg::Empty());
            msg_bool.data = false;
            pub_pot_fields_enable_->publish(msg_bool);
            reset_recovery_state();
            publish_nav_feedback("GOAL_NOT_CONVERGING", msg);
            current_status = publish_status(actionlib_msgs::msg::GoalStatus::ABORTED, goal_id, msg);
            finish_action_abort(msg, PumasNav::Result::OUTCOME_NOT_CONVERGING);
            state = SM_INIT;
            break;
          }

          // Warn-only operation: log, rearm and keep trying.
          RCLCPP_WARN(
            this->get_logger(),
            "MotionPlanner.-> Goal not converging (%s) but "
            "enable_stuck_abort is false; continuing.",
            convergence_reason_.c_str());
          converge_level_ = 0;
          convergence_clock_valid_ = false;
          replan_events_.clear();
          replan_total_ = 0;
          state = SM_CALCULATE_PATH;
          break;
        }

        case SM_CONVERGENCE_WAIT_CLEAR_MEM: {
          if (!clear_memory_response_received_) {
            break;
          }
          if (clear_memory_failed_) {
            RCLCPP_WARN(
              this->get_logger(),
              "MotionPlanner.-> Convergence tier 0: "
              "clear_obstacle_memory call failed.");
          }
          state = SM_CALCULATE_PATH;
          break;
        }

        case SM_CONVERGENCE_WAIT_FREE_MAP: {
          if (!recovery_map_response_) {
            break;
          }

          float free_x = 0.0f, free_y = 0.0f;
          if (find_nearest_free_point(robot_x_, robot_y_, free_x, free_y)) {
            // Plan out of the obstacle without moving the robot at all.
            plan_start_x_ = free_x;
            plan_start_y_ = free_y;
            plan_start_valid_ = true;
            current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
              "Robot inside obstacle; planning from nearest free cell");
            state = SM_CALCULATE_PATH;
            break;
          }

          // No known-free cell within reach (or no map): fall back to the
          // original blind reverse move.
          RCLCPP_WARN(
            this->get_logger(),
            "MotionPlanner.-> No free cell within %.2f m; falling back "
            "to moving backwards.",
            free_point_search_radius_);
          simple_move_goal_status_.status = 0;
          simple_move_status_id_.clear();
          msg_goal_dist_angle.data.resize(2);
          msg_goal_dist_angle.data[0] = -recovery_back_dist_;
          msg_goal_dist_angle.data[1] = 0;
          pub_goal_dist_angle_->publish(msg_goal_dist_angle);
          state = SM_WAITING_FOR_MOVE_BACKWARDS;
          break;
        }

        case SM_CHECK_IF_INSIDE_OBSTACLES: {
          std::cout << "MotionPlanner.-> Checking if robot is inside an obstacle..." << std::endl;
          is_in_obstacles_ = false;
          is_in_obstacles_response_ = false;

          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
          clt_is_in_obstacles_->async_send_request(
            request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                is_in_obstacles_ = future.get()->success;
              } catch (const std::exception & e) {
                RCLCPP_ERROR(
                  this->get_logger(), "MotionPlanner.-> Failed to get obstacles: %s", e.what());
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
              this->get_logger(), "MotionPlanner.-> Waiting for obstacle check response...");
            break;
          }

          if (is_in_obstacles_) {
            // Try to plan out of the obstacle before driving out of it: fetch
            // the augmented map and look for a known-free cell nearby to use as
            // the planning start. Reversing blindly is the fallback, not the
            // first move, because the space behind may be no better.
            std::cout << "MotionPlanner.-> Robot is inside an obstacle. Looking "
                         "for a free cell to plan from..."
                      << std::endl;
            get_robot_position();
            request_augmented_map();
            state = SM_CONVERGENCE_WAIT_FREE_MAP;
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
          if (
            simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == "-1") {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Moved backwards successfully." << std::endl;
            state = SM_CALCULATE_PATH;
          } else if (simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED) {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Simple move reported move aborted." << std::endl;
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
            request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                are_there_obs_ = future.get()->success;
              } catch (const std::exception & e) {
                RCLCPP_ERROR(
                  this->get_logger(), "MotionPlanner.-> Failed to get obstacle check: %s",
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
              this->get_logger(), "MotionPlanner.-> Waiting for obstacle check response...");
            break;
          }

          if (!are_there_obs_) {
            std::cout << "MotionPlanner.->There are no temporal obstacles. "
                         "Announcing failure."
                      << std::endl;
            current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
              "Cannot calculate path from start to goal point");
            finish_action_abort(
              "Cannot calculate path from start to goal point", PumasNav::Result::OUTCOME_NO_PATH);
            state = SM_INIT;
          } else {
            std::cout << "MotionPlanner.->Temporal obstacles detected. Waiting "
                         "for them to move."
                      << std::endl;
            current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id,
              "Waiting for temporal obstacles to move");
            state = SM_WAIT_FOR_NO_OBSTACLES;
          }
          break;
        }

        case SM_WAIT_FOR_NO_OBSTACLES_DELAY: {
          // Idle until the re-poll is due; ticks keep running so feedback keeps
          // flowing and goals/cancels stay responsive.
          if (this->now() >= obstacle_recheck_at_) state = SM_WAIT_FOR_NO_OBSTACLES;
          break;
        }

        case SM_WAIT_FOR_NO_OBSTACLES: {
          is_wait_obs_response_ = false;
          are_still_obs_ = true;
          wait_obs_failed_ = false;

          auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
          clt_are_there_obs_->async_send_request(
            request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
              try {
                are_still_obs_ = future.get()->success;
              } catch (const std::exception & e) {
                RCLCPP_ERROR(
                  this->get_logger(),
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
              this->get_logger(), "MotionPlanner.-> Waiting for temporal obstacles to clear...");
            break;
          }

          if (wait_obs_failed_) {
            RCLCPP_ERROR(
              this->get_logger(),
              "MotionPlanner.-> Cannot call service for checking "
              "temporal obstacles. Announcing failure.");

            current_status = publish_status(
              actionlib_msgs::msg::GoalStatus::ABORTED, this->goal_id,
              "Cannot calculate path from start to goal point");
            finish_action_abort(
              "Cannot calculate path from start to goal point", PumasNav::Result::OUTCOME_NO_PATH);
            state = SM_INIT;

          } else if (!are_still_obs_) {
            std::cout << "MotionPlanner.-> Temporal obstacles removed." << std::endl;
            state = SM_CALCULATE_PATH;
          } else {
            // Re-poll in 1 s WITHOUT blocking. This used to be a
            // rclcpp::sleep_for(1s) inside the timer callback, which stalls the
            // node's default (mutually exclusive) callback group: for that whole
            // second no feedback is published, and no goal or cancel request is
            // served either, so a new goal could not preempt this wait.
            obstacle_recheck_at_ = this->now() + rclcpp::Duration::from_seconds(1.0);
            state = SM_WAIT_FOR_NO_OBSTACLES_DELAY;
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
              this->get_logger(), "MotionPlanner.-> Waiting for potential fields message...");

            is_pot_fields_response_ = false;
            pot_fields_received_ = false;
            waiting_for_potential_fields_ = true;
            pot_fields_start_time_ = this->now();
          }

          if (is_pot_fields_response_) {
            waiting_for_potential_fields_ = false;

            if (pot_fields_received_) {
              RCLCPP_INFO(
                this->get_logger(), "MotionPlanner.-> Potential fields is now available.");
              state = SM_START_MOVE_PATH;
            } else {
              RCLCPP_WARN(
                this->get_logger(), "MotionPlanner.-> Potential fields received but invalid.");
              waiting_for_potential_fields_ = false;
              finish_action_abort(
                "Potential fields received but invalid",
                PumasNav::Result::OUTCOME_POT_FIELDS_INVALID);
              state = SM_INIT;
            }
          } else if ((this->now() - pot_fields_start_time_).seconds() > 10.0) {
            RCLCPP_WARN(
              this->get_logger(), "MotionPlanner.-> Timeout waiting for potential fields message.");

            finish_action_abort(
              "Timeout waiting for potential fields message",
              PumasNav::Result::OUTCOME_POT_FIELDS_TIMEOUT);
            waiting_for_potential_fields_ = false;
            state = SM_INIT;
          }

          break;
        }

        case SM_START_MOVE_PATH: {
          std::cout << "MotionPlanner.-> Starting path following " << std::endl;
          collision_risk_ = false;
          collision_risk_latched_ = false;

          path_.header.frame_id = "map";
          path_.header.stamp = this->now();
          // Hand the goal orientation over on the last path pose. During the
          // last-mile omni simple_move yaws onto it while sliding into the goal,
          // so SM_CORRECT_FINAL_ANGLE has little left to turn. The path planner
          // leaves every pose orientation zeroed, so this is the only one set and
          // simple_move can tell it apart from "no yaw requested".
          if (!path_.poses.empty()) path_.poses.back().pose.orientation = global_goal_.orientation;
          pub_goal_path_->publish(path_);

          // Goal id for the handshake with simple_move, which echoes it back on
          // /simple_move/goal_reached. It must key on the *whole* stamp: this
          // used to be header.stamp.sec alone, which is unique enough under
          // wall-clock epoch seconds but not under simulated time, where the
          // stamp starts at 0 and the replan loops here can republish several
          // paths within one simulated second. Two paths sharing an id let a
          // stale SUCCEEDED from the previous path be accepted as completion of
          // the current one. simple_move builds the same string in
          // simple_move_node.cpp (search for goal_id.id) - keep them in sync.
          simple_move_sequencer = std::to_string(path_.header.stamp.sec) + "." +
                                  std::to_string(path_.header.stamp.nanosec);

          simple_move_goal_status_.status = 0;

          state = SM_WAIT_FOR_MOVE_FINISHED;
          break;
        }

        case SM_WAIT_FOR_MOVE_FINISHED: {
          get_robot_position();
          update_via_points();
          error = sqrt(
            pow(global_goal_.position.x - robot_x_, 2) +
            pow(global_goal_.position.y - robot_y_, 2));
          note_goal_distance(error);

          // Recovery escape: once we have been recovering, fully reset the
          // recovery state (counters + cloud box) as soon as the robot has made
          // real forward progress, i.e. closed recovery_progress_dist_ of the
          // distance to the goal it had when it first got stuck. Tied to escaping
          // recovery, NOT to the start->goal span. (A back-up increases `error`,
          // so it can never trigger this.)
          if (recovery_stuck_recorded_ && error <= stuck_goal_dist_ - recovery_progress_dist_) {
            std::cout << "MotionPlanner.-> Escaped collision recovery (closed "
                      << (stuck_goal_dist_ - error) << " m toward goal). Resetting recovery state."
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
            oss << std::fixed << std::setprecision(2) << "Stopped within goal_distance (" << error
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
            current_status =
              publish_status(actionlib_msgs::msg::GoalStatus::SUCCEEDED, goal_id, msg);
            finish_action_success(msg, PumasNav::Result::OUTCOME_STOP_SHORT);
            state = SM_INIT;
            break;
          }

          if (error < proximity_criterion_ && !near_goal_sent) {
            near_goal_sent = true;
            std::cout << "MotionPlanner.->Error less than proximity criterion. "
                         "Sending near goal point status."
                      << std::endl;
            publish_nav_feedback("NEAR_GOAL", "Near goal point");
            publish_status(actionlib_msgs::msg::GoalStatus::ACTIVE, goal_id, "Near goal point");
          }
          if (
            simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            !simple_move_sequencer.empty() && simple_move_status_id_ == simple_move_sequencer) {
            simple_move_goal_status_.status = 0;
            std::cout << "MotionPlanner.-> Path followed succesfully. " << std::endl;
            msg_bool.data = false;
            pub_pot_fields_enable_->publish(msg_bool);
            reset_recovery_state();
            state = SM_CORRECT_FINAL_ANGLE;
          } else if (simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED) {
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
              std::cout << "MotionPlanner.-> COLLISION RISK abort #" << collision_event_count_
                        << " (recovery triggers at " << collision_recovery_trigger_count_ << ")."
                        << std::endl;
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
            // sits inside the floored box). For online, SM_RECOVERY_WAIT_MAP
            // falls back to an in-place rotation when no translational escape is
            // clear.
            std::cout << "MotionPlanner.-> Collision recovery tier0: escape move "
                         "(choosing direction away from obstacle)."
                      << std::endl;
            request_augmented_map();
            state = SM_RECOVERY_WAIT_MAP;
          } else if (recovery_level_ <= recovery_max_shrink_steps_) {
            // Tier 1..N: shrink the cloud detection box one notch, replan.
            detection_scale_ =
              std::max(recovery_min_detection_scale_, detection_scale_ * recovery_shrink_factor_);
            publish_detection_scale(detection_scale_);
            std::cout << "MotionPlanner.-> Collision recovery tier" << recovery_level_
                      << ": shrink cloud box, scale=" << detection_scale_ << std::endl;
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
              std::cout << "MotionPlanner.-> Collision recovery tier" << recovery_level_
                        << ": last resort, point cloud DISABLED (lidar only)." << std::endl;
            }
            advance_recovery_level();
            state = SM_CALCULATE_PATH;
          }
          break;
        }

        case SM_RECOVERY_WAIT_MAP: {
          if (!recovery_map_response_) break;  // waiting for augmented map

          float rel_x = 0.0f, rel_y = 0.0f;
          if (choose_escape_move(rel_x, rel_y)) {
            std::cout << "MotionPlanner.-> Recovery escape move (away from "
                         "obstacle): x="
                      << rel_x << " y=" << rel_y << " m." << std::endl;
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std_msgs::msg::Float32MultiArray rp;
            rp.data = {rel_x, rel_y, 0.0f};  // [x fwd+, y left+, yaw] base frame
            pub_goal_rel_pose_->publish(rp);
            state = SM_RECOVERY_WAIT_BACKUP;
          } else if (use_online_) {
            // ONLINE: no translational escape is clear -> the robot is boxed in
            // on all sides, typically by a directional point-cloud phantom (lidar
            // + cloud both on). Shrinking the cloud box does not help once the
            // phantom sits inside the floored box, and forcing a translation
            // would risk a real collision. Instead rotate IN PLACE: the base
            // center is fixed (collision-safe) and the head camera reorients,
            // which sheds a directional phantom. Replan + retry after each
            // rotation; if a full turn still finds no way through, the robot is
            // enclosed -> stop at the nearest reachable point rather than looping
            // forever.
            if (recovery_rotated_total_ < recovery_rotate_max_) {
              std::cout << "MotionPlanner.-> No clear translational escape; "
                           "rotating in place "
                        << recovery_rotate_step_
                        << " rad to shed point-cloud phantom (rotated so far="
                        << recovery_rotated_total_ << " rad)." << std::endl;
              simple_move_goal_status_.status = 0;
              simple_move_status_id_.clear();
              std_msgs::msg::Float32MultiArray rp;
              rp.data = {0.0f, 0.0f, recovery_rotate_step_};  // [x, y, yaw]; no translation
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
                "Blocked by obstacles; stopped at nearest reachable point",
                PumasNav::Result::OUTCOME_BLOCKED_STOPPED);
              state = SM_INIT;
            }
          } else {
            std::cout << "MotionPlanner.-> No clear escape direction; shrinking "
                         "cloud box instead."
                      << std::endl;
            recovery_level_ = 1;  // jump straight to the shrink tier
            state = SM_COLLISION_RECOVERY;
          }
          break;
        }

        case SM_RECOVERY_WAIT_BACKUP: {
          if (
            simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == "-1") {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Recovery escape move finished." << std::endl;
            advance_recovery_level();
            state = SM_CALCULATE_PATH;
          } else if (simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED) {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Recovery escape move aborted; continuing." << std::endl;
            advance_recovery_level();
            state = SM_CALCULATE_PATH;
          } else {
            break;  // still backing up
          }
          break;
        }

        case SM_RECOVERY_WAIT_ROTATE: {
          // In-place recovery rotation (ONLINE phantom shedding). On completion,
          // stay in the escape/rotation tier (recovery_level_ = 0) and replan, so
          // the next collision re-enters the escape->rotate path and the rotation
          // scan keeps accumulating until the robot breaks free (forward progress
          // resets recovery) or a full turn is exhausted.
          if (
            simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == "-1") {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Recovery rotation finished; replanning." << std::endl;
            recovery_level_ = 0;
            state = SM_CALCULATE_PATH;
          } else if (simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED) {
            simple_move_goal_status_.status = 0;
            simple_move_status_id_.clear();
            std::cout << "MotionPlanner.-> Recovery rotation aborted; replanning." << std::endl;
            recovery_level_ = 0;
            state = SM_CALCULATE_PATH;
          } else {
            break;  // still rotating
          }
          break;
        }

        case SM_WAIT_FOR_NOT_POT_FIELDS: {
          if (!waiting_for_potential_fields_) {
            RCLCPP_INFO(
              this->get_logger(),
              "MotionPlanner.-> Waiting for potential fields with "
              "point cloud to be disabled...");

            waiting_for_potential_fields_ = true;
            pot_fields_start_time_ = this->now();
            no_cloud_pot_fields_start_time_ = this->now();
          }
          if ((this->now() - pot_fields_start_time_).seconds() > 1.0) {
            pot_fields_start_time_ = this->now();
            if (is_pot_fields_response_) {
              RCLCPP_WARN(
                this->get_logger(),
                "MotionPlanner.-> Potential fields with point cloud "
                "have been disabled.");
              collision_risk_ = false;
              waiting_for_potential_fields_ = false;

              state = SM_CALCULATE_PATH;
            }
          } else if ((this->now() - no_cloud_pot_fields_start_time_).seconds() > 10.0) {
            RCLCPP_WARN(
              this->get_logger(), "MotionPlanner.-> Timeout waiting for potential fields message.");
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

          if (error > M_PI) error -= 2 * M_PI;
          if (error <= -M_PI) error += 2 * M_PI;

          msg_goal_dist_angle.data.resize(2);
          msg_goal_dist_angle.data[0] = 0.0f;
          msg_goal_dist_angle.data[1] = static_cast<float>(error);
          pub_goal_dist_angle_->publish(msg_goal_dist_angle);

          state = SM_WAIT_FOR_ANGLE_CORRECTED;
          break;
        }

        case SM_WAIT_FOR_ANGLE_CORRECTED: {
          if (
            simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::SUCCEEDED &&
            simple_move_status_id_ == "-1") {
            simple_move_goal_status_.status = 0;
            std::cout << "MotionPlanner.-> Final angle corrected succesfully." << std::endl;
            state = SM_FINAL;
          } else if (simple_move_goal_status_.status == actionlib_msgs::msg::GoalStatus::ABORTED) {
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
          uint8_t final_outcome = PumasNav::Result::OUTCOME_GOAL_REACHED;
          if (!pending_final_message_.empty()) {
            // The task ended somewhere other than the requested goal (near-goal
            // replan suppression). near_goal_sent is already set by the gate.
            final_msg = pending_final_message_;
            final_outcome = pending_final_outcome_;
          } else if (goal_relocated_) {
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
            final_outcome = PumasNav::Result::OUTCOME_GOAL_RELOCATED;
          }
          current_status =
            publish_status(actionlib_msgs::msg::GoalStatus::SUCCEEDED, goal_id, final_msg);
          finish_action_success(final_msg, final_outcome);
          state = SM_INIT;
          break;
        }

        default:
          std::cout << "MotionPlanner.-> A VERY STUPID PERSON PROGRAMMED THIS "
                       "SHIT. APOLOGIES FOR THE INCONVINIENCE :'("
                    << std::endl;
          return;
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "MotionPlanner.-> Error in MotionPlanner processing: %s", e.what());
      if (current_status == actionlib_msgs::msg::GoalStatus::ACTIVE) {
        current_status = publish_status(
          actionlib_msgs::msg::GoalStatus::ABORTED, goal_id,
          "Exception in motion_planner_processing core");
      }
      if (action_active_) {
        finish_action_abort(
          "Exception in motion_planner_processing core", PumasNav::Result::OUTCOME_INTERNAL_ERROR);
      }
      state = SM_INIT;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlannerNode>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
