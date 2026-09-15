// C++ port of motion_synth_server.py (originally maintained by ry0hei-kobayashi).
// Synchronizes arm/head motion with base navigation progress: freezes a
// trigger waypoint at ~75% of the first planned path and fires the arm/head
// goal pose once the robot nears it (or the goal, as a fallback), with
// self-collision-aware staging for risky goal poses.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <actionlib_msgs/msg/goal_status.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pumas_interfaces/action/motion_synthesis.hpp>
#include <pumas_interfaces/msg/motion_pose.hpp>
#include <pumas_interfaces/msg/joints.hpp>

#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class MotionSynthNode : public rclcpp::Node
{
public:
  using MotionSynthesis = pumas_interfaces::action::MotionSynthesis;
  using GoalHandleMotionSynthesis = rclcpp_action::ServerGoalHandle<MotionSynthesis>;
  using Point2 = std::array<double, 2>;

  MotionSynthNode()
  : Node("motion_synth_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Self-collision-aware staging: which Joints fields to read (so this
    // isn't hardcoded to one robot's specific joint names) and the
    // thresholds/staged value that classify a goal pose as risky. Defaults
    // match this robot's arm geometry.
    this->declare_parameter<std::string>("flex_joint_name", "arm_flex_joint");
    this->declare_parameter<std::string>("lift_joint_name", "arm_lift_joint");
    this->declare_parameter<double>("large_forward_flex_threshold", -1.0);
    this->declare_parameter<double>("self_collision_flex_threshold", -0.35);
    this->declare_parameter<double>("self_collision_lift_threshold", 0.15);
    // Matches self_collision_flex_threshold: the minimal move that escapes
    // the risky zone, rather than an arbitrary deeper fold with no other
    // precedent in this codebase's validated arm poses.
    this->declare_parameter<double>("staging_flex_pose", -0.35);

    this->get_parameter("flex_joint_name", flex_joint_name_);
    this->get_parameter("lift_joint_name", lift_joint_name_);
    this->get_parameter("large_forward_flex_threshold", large_forward_flex_threshold_);
    this->get_parameter("self_collision_flex_threshold", self_collision_flex_threshold_);
    this->get_parameter("self_collision_lift_threshold", self_collision_lift_threshold_);
    this->get_parameter("staging_flex_pose", staging_flex_pose_);

    // Which Joints.msg fields to carry over into the outgoing JointState (see
    // joints_to_state) — configurable so this isn't hardcoded to one robot's
    // specific joint set. Validated against the actual message shape since a
    // bad name would otherwise be silently dropped by get_joints_field.
    this->declare_parameter<std::vector<std::string>>("joints_fields", {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint",
        "head_pan_joint", "head_tilt_joint"});
    std::vector<std::string> configured_fields;
    this->get_parameter("joints_fields", configured_fields);
    pumas_interfaces::msg::Joints probe;
    for (const auto &f : configured_fields) {
      double dummy;
      if (get_joints_field(probe, f, dummy)) {
        joints_fields_.push_back(f);
      } else {
        RCLCPP_ERROR(this->get_logger(),
            "motion_synth -> joints_fields contains unknown Joints.msg field: '%s'; ignoring it.",
            f.c_str());
      }
    }

    motion_pose_pub_ = this->create_publisher<pumas_interfaces::msg::MotionPose>(
        "/hardware/motion_pose", 10);

    sub_nav_status_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
        "/navigation/status", 10,
        std::bind(&MotionSynthNode::navigation_status_callback, this, std::placeholders::_1));

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/simple_move/goal_path", 10,
        std::bind(&MotionSynthNode::path_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<MotionSynthesis>(
        this, "/motion_synth",
        std::bind(&MotionSynthNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionSynthNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&MotionSynthNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "motion_synth_server.-> is ready");
  }

private:
  // ---------- Parameters ----------
  std::string flex_joint_name_;
  std::string lift_joint_name_;
  double large_forward_flex_threshold_ = -1.0;
  double self_collision_flex_threshold_ = -0.35;
  double self_collision_lift_threshold_ = 0.15;
  double staging_flex_pose_ = -0.35;
  std::vector<std::string> joints_fields_;

  // ---------- Cross-thread state (written by subscription callbacks, read
  // from the detached execute() thread) — guarded by state_mtx_. ----------
  std::mutex state_mtx_;
  std::vector<Point2> path_points_;
  bool global_nav_goal_reached_ = false;

  // Only ever touched from navigation_status_callback (single-threaded).
  bool last_nav_status_valid_ = false;
  uint8_t last_nav_status_ = 0;

  // Only ever written/read from within a single execute() invocation.
  bool current_pose_valid_ = false;
  double current_pose_x_ = 0.0, current_pose_y_ = 0.0, current_pose_yaw_ = 0.0;

  // Per-goal motion execution time. Set in execute() before any send_pose call.
  double motion_execution_time_ = 0.5;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<pumas_interfaces::msg::MotionPose>::SharedPtr motion_pose_pub_;
  rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr sub_nav_status_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp_action::Server<MotionSynthesis>::SharedPtr action_server_;

  // ---------- Joints field access (the only place that needs to know
  // Joints.msg's fixed field names) ----------
  static bool get_joints_field(const pumas_interfaces::msg::Joints &j,
                              const std::string &name, double &out)
  {
    if (name == "arm_lift_joint")   { out = j.arm_lift_joint;   return true; }
    if (name == "arm_flex_joint")   { out = j.arm_flex_joint;   return true; }
    if (name == "arm_roll_joint")   { out = j.arm_roll_joint;   return true; }
    if (name == "wrist_flex_joint") { out = j.wrist_flex_joint; return true; }
    if (name == "wrist_roll_joint") { out = j.wrist_roll_joint; return true; }
    if (name == "head_pan_joint")   { out = j.head_pan_joint;   return true; }
    if (name == "head_tilt_joint")  { out = j.head_tilt_joint;  return true; }
    return false;
  }

  static bool set_joints_field(pumas_interfaces::msg::Joints &j,
                              const std::string &name, double value)
  {
    if (name == "arm_lift_joint")   { j.arm_lift_joint   = static_cast<float>(value); return true; }
    if (name == "arm_flex_joint")   { j.arm_flex_joint   = static_cast<float>(value); return true; }
    if (name == "arm_roll_joint")   { j.arm_roll_joint   = static_cast<float>(value); return true; }
    if (name == "wrist_flex_joint") { j.wrist_flex_joint = static_cast<float>(value); return true; }
    if (name == "wrist_roll_joint") { j.wrist_roll_joint = static_cast<float>(value); return true; }
    if (name == "head_pan_joint")   { j.head_pan_joint   = static_cast<float>(value); return true; }
    if (name == "head_tilt_joint")  { j.head_tilt_joint  = static_cast<float>(value); return true; }
    return false;
  }

  sensor_msgs::msg::JointState joints_to_state(const pumas_interfaces::msg::Joints &joints)
  {
    sensor_msgs::msg::JointState state;
    state.header = joints.header;
    state.name = joints_fields_;
    state.position.reserve(joints_fields_.size());
    for (const auto &f : joints_fields_) {
      double v = 0.0;
      get_joints_field(joints, f, v);
      state.position.push_back(v);
    }
    return state;
  }

  void send_pose(const pumas_interfaces::msg::Joints &joints)
  {
    pumas_interfaces::msg::MotionPose msg;
    msg.joints = joints_to_state(joints);
    msg.motion_execution_time = static_cast<float>(motion_execution_time_);
    motion_pose_pub_->publish(msg);
  }

  // ---------- Risk classification ----------
  enum class RiskTag { kNone, kLargeForwardFlex, kSelfCollisionZone };

  // Arm bent strongly forward, past large_forward_flex_threshold.
  //
  // Not a static self-collision, but we don't want the arm to swing all the
  // way forward in one shot during navigation — it adds inertia and the
  // wrist can swing into the base. Staged motion (partial fold first, full
  // fold once positioned) keeps the swing tame.
  bool is_large_forward_flex(const pumas_interfaces::msg::Joints &goal_pose)
  {
    double flex = 0.0;
    get_joints_field(goal_pose, flex_joint_name_, flex);
    return flex < large_forward_flex_threshold_;
  }

  // Arm near-vertical with the lift raised — true self-collision risk.
  //
  // Past self_collision_lift_threshold the shoulder is high enough that an
  // arm pointing near-straight-up (past self_collision_flex_threshold) hits
  // the head/torso structure. Staged motion folds the arm forward before
  // committing to this lift height.
  bool is_self_collision_zone(const pumas_interfaces::msg::Joints &goal_pose)
  {
    double flex = 0.0, lift = 0.0;
    get_joints_field(goal_pose, flex_joint_name_, flex);
    get_joints_field(goal_pose, lift_joint_name_, lift);
    return flex > self_collision_flex_threshold_ && lift > self_collision_lift_threshold_;
  }

  // Return the risk tag for the goal pose, or kNone if no staging needed.
  RiskTag classify_goal_pose_risk(const pumas_interfaces::msg::Joints &goal_pose)
  {
    if (is_large_forward_flex(goal_pose)) return RiskTag::kLargeForwardFlex;
    if (is_self_collision_zone(goal_pose)) return RiskTag::kSelfCollisionZone;
    return RiskTag::kNone;
  }

  pumas_interfaces::msg::Joints create_temporary_pose(const pumas_interfaces::msg::Joints &goal_pose)
  {
    pumas_interfaces::msg::Joints tmp = goal_pose;
    if (is_large_forward_flex(goal_pose) || is_self_collision_zone(goal_pose)) {
      set_joints_field(tmp, flex_joint_name_, staging_flex_pose_);
    }
    return tmp;
  }

  // TODO: Replace with real check
  bool joint_goal_reached(const pumas_interfaces::msg::Joints & /*goal_joints*/)
  {
    return true;
  }

  static double distance_xy(const Point2 &a, const Point2 &b)
  {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
  }

  static bool path_matches_goal(const std::vector<Point2> &points,
                               const geometry_msgs::msg::Pose2D &goal_location,
                               double tolerance = 1.0)
  {
    if (points.empty()) return false;
    const auto &last = points.back();
    double dx = last[0] - goal_location.x;
    double dy = last[1] - goal_location.y;
    return std::sqrt(dx * dx + dy * dy) < tolerance;
  }

  // ---------- Trigger configuration. Tune in-place if real-robot behavior
  // demands it. ----------
  static constexpr double kTriggerWaypointFraction = 0.75;  // snapshot waypoint at 75% along first path
  static constexpr double kTriggerWaypointRadius = 1.0;     // m, distance to frozen waypoint to fire
  static constexpr double kTriggerGoalSafetyRadius = 0.6;   // m, fallback when detour skipped waypoint
  static constexpr double kMinSettleSec = 1.5;              // min seconds after start_pose before trigger
  // Final goal pose is sent only after mvn_pln has finished the path and is
  // in its final-yaw correction phase. We detect that phase by requiring the
  // robot to be inside this radius around the goal location — during normal
  // navigation the yaw may transiently match goal_theta and we must NOT fire
  // on that false positive.
  static constexpr double kFinalPoseGoalRadius = 0.3;   // m
  static constexpr double kFinalPoseYawTolerance = 0.3; // rad
  // If the robot is already this close to the goal at execute time, the path
  // planner will return no/short path and the path-based trigger never fires.
  // Take the trivial-nav fast path instead.
  static constexpr double kTrivialNavDistance = 0.15; // m

  // ---------- Subscription callbacks ----------
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;
    std::vector<Point2> points;
    points.reserve(msg->poses.size());
    for (const auto &p : msg->poses) {
      points.push_back({p.pose.position.x, p.pose.position.y});
    }
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      path_points_ = std::move(points);
    }
    RCLCPP_INFO(this->get_logger(), "motion_synth -> Received Path Length: %zu", msg->poses.size());
  }

  void navigation_status_callback(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
  {
    // actionlib_msgs/GoalStatus uses SUCCEEDED=3 (not STATUS_SUCCEEDED=4 from
    // action_msgs). mvn_pln publishes actionlib_msgs on this topic. Log every
    // transition so we can see why global_nav_goal_reached latches (or
    // doesn't).
    if (!last_nav_status_valid_ || last_nav_status_ != msg->status) {
      RCLCPP_INFO(this->get_logger(),
          "motion_synth -> /navigation/status status=%d (SUCCEEDED=3, ACTIVE=1, ABORTED=4) text='%s'",
          static_cast<int>(msg->status), msg->text.c_str());
      last_nav_status_ = msg->status;
      last_nav_status_valid_ = true;
    }
    if (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED) {
      std::lock_guard<std::mutex> lock(state_mtx_);
      global_nav_goal_reached_ = true;
    }
  }

  bool get_global_pose_from_tf(double &x, double &y, double &yaw,
                              const std::string &target_frame = "map",
                              const std::string &source_frame = "base_footprint")
  {
    try {
      geometry_msgs::msg::TransformStamped t =
          tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "motion_synth -> TF lookup failed: %s -> %s: %s",
                  target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
    }
  }

  // ---------- Action server callbacks ----------
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MotionSynthesis::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMotionSynthesis>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // execute_callback runs synchronously and blocks (sleep_for). It runs on a
  // detached thread (spawned below) rather than an executor callback group,
  // so /navigation/status and /simple_move/goal_path keep arriving via the
  // MultiThreadedExecutor while this thread waits. The robot pose is pulled
  // on-demand from TF inside the loop (see get_global_pose_from_tf).
  void handle_accepted(const std::shared_ptr<GoalHandleMotionSynthesis> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  // ---------- Trivial-nav fast path: robot already at goal ----------
  void execute_trivial(const std::shared_ptr<const MotionSynthesis::Goal> &goal,
                      const std::shared_ptr<GoalHandleMotionSynthesis> &goal_handle,
                      const rclcpp::Time &start_pose_dispatch_time)
  {
    RiskTag risk = classify_goal_pose_risk(goal->goal_pose);
    bool needs_staging = risk != RiskTag::kNone;

    // Let start_pose play out before we layer the next command on top.
    double elapsed_sec = (this->now() - start_pose_dispatch_time).seconds();
    double settle_remaining = std::max(0.0, kMinSettleSec - elapsed_sec);
    if (settle_remaining > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(settle_remaining));
    }

    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<MotionSynthesis::Result>();
      result->result = false;
      goal_handle->canceled(result);
      return;
    }

    if (needs_staging) {
      RCLCPP_INFO(this->get_logger(), "motion_synth -> trivial nav: sending staging pose");
      send_pose(create_temporary_pose(goal->goal_pose));
      std::this_thread::sleep_for(std::chrono::duration<double>(kMinSettleSec));

      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<MotionSynthesis::Result>();
        result->result = false;
        goal_handle->canceled(result);
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "motion_synth -> trivial nav: sending final goal pose");
    send_pose(goal->goal_pose);
    // joint_goal_reached is a TODO stub that just returns true, so wait a
    // fixed settle period to give the hardware time to reach the pose.
    std::this_thread::sleep_for(std::chrono::duration<double>(kMinSettleSec));

    auto result = std::make_shared<MotionSynthesis::Result>();
    result->result = true;
    goal_handle->succeed(result);
  }

  // ---------- Main execute loop ----------
  void execute(const std::shared_ptr<GoalHandleMotionSynthesis> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing motion_synth goal...");
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MotionSynthesis::Feedback>();

    // motion_execution_time == 0.0 means "unset"; fall back to 0.5 s.
    motion_execution_time_ = (goal->motion_execution_time > 0.0f)
        ? static_cast<double>(goal->motion_execution_time) : 0.5;
    RCLCPP_INFO(this->get_logger(), "motion_synth -> motion_execution_time = %.3f s",
                motion_execution_time_);

    // Reset only the per-goal latching flag. Do NOT reset path_points_,
    // because mvn_pln publishes the goal_path only once per replan and the
    // message may have arrived either before or after this callback starts.
    // Stale paths from a previous goal are filtered by path_matches_goal.
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      global_nav_goal_reached_ = false;
    }

    rclcpp::Time start_pose_dispatch_time = this->now();
    if (goal->apply_start_pose) {
      send_pose(goal->start_pose);
      start_pose_dispatch_time = this->now();
    }

    // Trivial-nav fast path: when the robot is already at the goal, the
    // planner returns no/short path and the path-based trigger never fires,
    // so the goal_pose would be skipped. Detect this case and execute the
    // arm motion directly.
    if (current_pose_valid_) {
      Point2 cur_xy{current_pose_x_, current_pose_y_};
      Point2 goal_xy{goal->goal_location.x, goal->goal_location.y};
      double d_goal = distance_xy(cur_xy, goal_xy);
      if (d_goal < kTrivialNavDistance) {
        RCLCPP_INFO(this->get_logger(),
            "motion_synth -> trivial nav (d_goal=%.3f m < %.2f m); executing arm motion directly",
            d_goal, kTrivialNavDistance);
        execute_trivial(goal, goal_handle, start_pose_dispatch_time);
        return;
      }
    }

    // Wait until a path whose tail matches this goal_location arrives. 15 s
    // is longer than mvn_pln's 10 s potential-fields timeout so a slow
    // path-calculation chain still gets through.
    constexpr int kPathWaitIters = 150;
    bool have_path = false;
    for (int i = 0; i < kPathWaitIters; ++i) {
      {
        std::lock_guard<std::mutex> lock(state_mtx_);
        have_path = path_matches_goal(path_points_, goal->goal_location);
      }
      if (have_path) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!have_path) {
      RCLCPP_WARN(this->get_logger(), "No path matching goal_location received, aborting.");
      auto result = std::make_shared<MotionSynthesis::Result>();
      result->result = false;
      goal_handle->abort(result);
      return;
    }

    // Snapshot the FIRST matching path and freeze a waypoint at 75% of it.
    // mvn_pln republishes a new goal_path on every replan/detour, which would
    // drift an index-on-latest-path approach; freezing the waypoint in map
    // frame makes the trigger geographically stable.
    std::vector<Point2> first_path;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      first_path = path_points_;
    }
    if (first_path.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Frozen path is too short to choose a trigger waypoint, aborting.");
      auto result = std::make_shared<MotionSynthesis::Result>();
      result->result = false;
      goal_handle->abort(result);
      return;
    }

    size_t trigger_idx = std::min(
        first_path.size() - 1,
        std::max<size_t>(1, static_cast<size_t>(first_path.size() * kTriggerWaypointFraction)));
    Point2 arm_trigger_xy = first_path[trigger_idx];
    Point2 goal_xy{goal->goal_location.x, goal->goal_location.y};
    RCLCPP_INFO(this->get_logger(),
        "motion_synth -> frozen trigger waypoint = (%.2f, %.2f) (idx %zu/%zu), "
        "goal safety radius = %.2f m, settle delay = %.2f s",
        arm_trigger_xy[0], arm_trigger_xy[1], trigger_idx, first_path.size(),
        kTriggerGoalSafetyRadius, kMinSettleSec);

    bool triggered = false;
    pumas_interfaces::msg::Joints temporary_pose;
    bool temporary_pose_sent = false;
    bool final_pose_sent = false;
    RiskTag risk = classify_goal_pose_risk(goal->goal_pose);
    bool needs_staging = risk != RiskTag::kNone;
    if (risk == RiskTag::kLargeForwardFlex) {
      double flex = 0.0;
      get_joints_field(goal->goal_pose, flex_joint_name_, flex);
      RCLCPP_INFO(this->get_logger(),
          "motion_synth -> large arm flex detected (%s=%.2f rad). Staging motion via temporary pose.",
          flex_joint_name_.c_str(), flex);
      temporary_pose = create_temporary_pose(goal->goal_pose);
    } else if (risk == RiskTag::kSelfCollisionZone) {
      double flex = 0.0, lift = 0.0;
      get_joints_field(goal->goal_pose, flex_joint_name_, flex);
      get_joints_field(goal->goal_pose, lift_joint_name_, lift);
      RCLCPP_WARN(this->get_logger(),
          "motion_synth -> self-collision zone detected (%s=%.2f rad, %s=%.2f m). Using temporary pose.",
          flex_joint_name_.c_str(), flex, lift_joint_name_.c_str(), lift);
      temporary_pose = create_temporary_pose(goal->goal_pose);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Log loop state every kLogPeriod iterations (~1 s at 0.05 s sleep).
    constexpr int kLogPeriod = 20;
    int loop_iter = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        auto result = std::make_shared<MotionSynthesis::Result>();
        result->result = false;
        goal_handle->canceled(result);
        return;
      }

      double x, y, yaw;
      if (get_global_pose_from_tf(x, y, yaw)) {
        current_pose_x_ = x;
        current_pose_y_ = y;
        current_pose_yaw_ = yaw;
        current_pose_valid_ = true;
      }

      if (!triggered && current_pose_valid_) {
        double elapsed_sec = (this->now() - start_pose_dispatch_time).seconds();
        Point2 cur_xy{current_pose_x_, current_pose_y_};
        double dist_to_waypoint = distance_xy(cur_xy, arm_trigger_xy);
        double dist_to_goal = distance_xy(cur_xy, goal_xy);
        if (loop_iter % kLogPeriod == 0) {
          RCLCPP_INFO(this->get_logger(),
              "motion_synth -> waiting trigger: elapsed=%.2f/%.2f s, d_wp=%.2f/%.2f m, d_goal=%.2f/%.2f m",
              elapsed_sec, kMinSettleSec, dist_to_waypoint, kTriggerWaypointRadius,
              dist_to_goal, kTriggerGoalSafetyRadius);
        }
        if (elapsed_sec >= kMinSettleSec) {
          bool fire = false;
          std::string reason;
          if (dist_to_waypoint < kTriggerWaypointRadius) {
            fire = true;
            reason = "dist_to_waypoint=" + std::to_string(dist_to_waypoint) + " m < " +
                     std::to_string(kTriggerWaypointRadius) + " m";
          } else if (dist_to_goal < kTriggerGoalSafetyRadius) {
            fire = true;
            reason = "safety: dist_to_goal=" + std::to_string(dist_to_goal) + " m < " +
                     std::to_string(kTriggerGoalSafetyRadius) + " m";
          }

          if (fire) {
            RCLCPP_INFO(this->get_logger(), "Triggering arm motion (%s)", reason.c_str());
            if (needs_staging) {
              RCLCPP_INFO(this->get_logger(),
                  "motion_synth -> sending temporary pose, will wait for yaw alignment or nav SUCCEEDED");
              send_pose(temporary_pose);
              temporary_pose_sent = true;
            } else {
              RCLCPP_INFO(this->get_logger(),
                  "motion_synth -> no staging needed, sending final goal pose directly");
              send_pose(goal->goal_pose);
              final_pose_sent = true;
            }
            triggered = true;
          }
        }
      }

      if (triggered) {
        if (temporary_pose_sent && !final_pose_sent) {
          double yaw_error = std::fabs(current_pose_yaw_ - goal->goal_location.theta);
          if (yaw_error > M_PI) yaw_error = 2 * M_PI - yaw_error;
          Point2 cur_xy{current_pose_x_, current_pose_y_};
          double dist_to_goal = distance_xy(cur_xy, goal_xy);
          bool yaw_aligned_at_goal = yaw_error < kFinalPoseYawTolerance &&
                                     dist_to_goal < kFinalPoseGoalRadius;

          bool nav_reached;
          {
            std::lock_guard<std::mutex> lock(state_mtx_);
            nav_reached = global_nav_goal_reached_;
          }

          if (loop_iter % kLogPeriod == 0) {
            RCLCPP_INFO(this->get_logger(),
                "motion_synth -> waiting final: d_goal=%.2f/%.2f m, yaw_error=%.2f/%.2f rad "
                "(both required), nav_goal_reached=%s",
                dist_to_goal, kFinalPoseGoalRadius, yaw_error, kFinalPoseYawTolerance,
                nav_reached ? "true" : "false");
          }
          if (yaw_aligned_at_goal || nav_reached) {
            const char *fire_reason = yaw_aligned_at_goal ? "yaw aligned at goal" : "nav SUCCEEDED";
            RCLCPP_INFO(this->get_logger(),
                "motion_synth -> sending final goal pose (%s, d_goal=%.2f m, yaw_error=%.2f rad, "
                "nav_goal_reached=%s)",
                fire_reason, dist_to_goal, yaw_error, nav_reached ? "true" : "false");
            send_pose(goal->goal_pose);
            final_pose_sent = true;
            std::lock_guard<std::mutex> lock(state_mtx_);
            global_nav_goal_reached_ = false;
          }
        }
      }

      if (triggered && final_pose_sent) {
        if (joint_goal_reached(goal->goal_pose)) {
          RCLCPP_INFO(this->get_logger(), "Final arm pose reached.");
          auto result = std::make_shared<MotionSynthesis::Result>();
          result->result = true;
          goal_handle->succeed(result);
          return;
        }
      }

      goal_handle->publish_feedback(feedback);
      loop_iter++;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionSynthNode>();
  // MultiThreadedExecutor is required because execute() blocks on
  // sleep_for on its own detached thread; subscriptions are served
  // concurrently by the executor while it waits.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
