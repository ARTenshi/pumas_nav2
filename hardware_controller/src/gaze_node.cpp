#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <algorithm>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pumas_interfaces/action/gaze_head.hpp>

#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>


class GazeController : public rclcpp::Node
{
public:
  using GazeHead       = pumas_interfaces::action::GazeHead;
  using GoalHandleGaze = rclcpp_action::ServerGoalHandle<GazeHead>;

  GazeController()
  : Node("gaze_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<bool>(        "use_namespace",  false);
    this->declare_parameter<std::string>( "gaze_tf_name",   "gaze_point");
    this->declare_parameter<std::string>( "base_link_name", "base_footprint");
    this->declare_parameter<double>(      "head_height",    1.1);
    // Slew-rate smoothing: cap how fast the head goal can change per tick so the
    // head ramps toward a (possibly jumping) gaze target instead of snapping.
    this->declare_parameter<double>(      "max_pan_vel",    1.0);   // [rad/s]
    this->declare_parameter<double>(      "max_tilt_vel",   0.7);   // [rad/s]
    this->declare_parameter<int>(         "gaze_period_ms", 100);   // loop period

    this->get_parameter("use_namespace",  use_namespace_);
    this->get_parameter("gaze_tf_name",   gaze_tf_name_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("head_height",    head_height_);
    this->get_parameter("max_pan_vel",    max_pan_vel_);
    this->get_parameter("max_tilt_vel",   max_tilt_vel_);
    this->get_parameter("gaze_period_ms", gaze_period_ms_);

    pub_head_goal_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      make_name("/hardware/head/goal_pose"),
      rclcpp::QoS(10).reliable());

    // Track the actual head pose so the smoother can seed cmd_* at goal start
    // (prevents an initial snap from a stale 0,0).
    sub_head_current_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      make_name("/hardware/head/current_pose"),
      rclcpp::QoS(10).reliable(),
      std::bind(&GazeController::head_current_pose_callback, this,
                std::placeholders::_1));

    // Action server on /gaze_head.
    // execute() runs in a detached thread so it can block without starving
    // the TF listener subscription that the MultiThreadedExecutor serves.
    action_server_ = rclcpp_action::create_server<GazeHead>(
      this,
      "gaze_head",
      std::bind(&GazeController::handle_goal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&GazeController::handle_cancel,   this, std::placeholders::_1),
      std::bind(&GazeController::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Action server ready on /gaze_head");
  }

private:
  bool        use_namespace_ = false;
  std::string gaze_tf_name_;
  std::string base_link_name_;
  double      head_height_;
  double      max_pan_vel_    = 1.0;
  double      max_tilt_vel_   = 0.7;
  int         gaze_period_ms_ = 100;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_head_goal_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_head_current_pose_;
  rclcpp_action::Server<GazeHead>::SharedPtr action_server_;

  // Latest measured head pose (pan, tilt). Updated by head_current_pose_callback.
  std::mutex         head_pose_mtx_;
  float              cur_pan_  = 0.0f;
  float              cur_tilt_ = 0.0f;
  bool               head_pose_valid_ = false;

  // Generation counter for goal preemption: a newly accepted goal increments
  // this, and any older execute() loop exits as soon as it notices the change.
  std::atomic<uint64_t> active_gen_{0};

  void head_current_pose_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2)
      return;
    std::lock_guard<std::mutex> lock(head_pose_mtx_);
    cur_pan_  = msg->data[0];
    cur_tilt_ = msg->data[1];
    head_pose_valid_ = true;
  }

  // Shortest-path normalization to (-pi, pi].
  static float normalize_angle(float a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  std::string make_name(const std::string &suffix) const
  {
    std::string sfx = suffix;
    if (!sfx.empty() && sfx.front() != '/')
      sfx = "/" + sfx;

    std::string name;
    if (use_namespace_) {
      name = this->get_namespace() + sfx;
      if (name.size() > 1 && name[0] == '/' && name[1] == '/')
        name.erase(0, 1);
    } else {
      name = sfx;
    }
    return name;
  }

  // Compute [pan, tilt] toward gaze_tf_name in the map frame.
  // Returns false (and leaves msg unchanged) when any TF lookup fails.
  bool gaze_point_to_head_angles(
    std_msgs::msg::Float32MultiArray &msg,
    const std::string &gaze_tf_name)
  {
    try {
      geometry_msgs::msg::TransformStamped robot_tf =
        tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

      float robot_x = static_cast<float>(robot_tf.transform.translation.x);
      float robot_y = static_cast<float>(robot_tf.transform.translation.y);

      tf2::Quaternion q(
        robot_tf.transform.rotation.x,
        robot_tf.transform.rotation.y,
        robot_tf.transform.rotation.z,
        robot_tf.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      float robot_t = static_cast<float>(yaw);

      geometry_msgs::msg::TransformStamped gaze_tf =
        tf_buffer_.lookupTransform("map", gaze_tf_name, tf2::TimePointZero);

      float gaze_x = static_cast<float>(gaze_tf.transform.translation.x);
      float gaze_y = static_cast<float>(gaze_tf.transform.translation.y);
      float gaze_z = static_cast<float>(gaze_tf.transform.translation.z);

      float pan = atan2(gaze_y - robot_y, gaze_x - robot_x) - robot_t;
      if (pan >  M_PI) pan -= 2.0f * M_PI;
      if (pan <= -M_PI) pan += 2.0f * M_PI;

      float dx     = gaze_x - robot_x;
      float dy     = gaze_y - robot_y;
      float h_dist = std::sqrt(dx * dx + dy * dy);
      float tilt   = std::atan2(gaze_z - static_cast<float>(head_height_), h_dist);

      msg.data = {pan, tilt};
      return true;
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "GazeController.-> TF lookup failed: %s", ex.what());
      return false;
    }
  }

  // ---------- Action server callbacks ----------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GazeHead::Goal> goal)
  {
    const std::string tf_name =
      goal->gaze_tf_name.empty() ? gaze_tf_name_ : goal->gaze_tf_name;
    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Goal received. TF target: '%s'", tf_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGaze>)
  {
    RCLCPP_INFO(this->get_logger(), "GazeController.-> Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGaze> goal_handle)
  {
    // Preempt any previous gaze loop: bump the generation so the older
    // execute() exits and only this goal owns the head publisher.
    const uint64_t my_gen = ++active_gen_;
    std::thread([this, goal_handle, my_gen]() { execute(goal_handle, my_gen); }).detach();
  }

  // Main gaze loop. Runs in a detached thread; publishes slew-limited head
  // angle commands every gaze_period_ms until canceled, preempted, or shutdown.
  void execute(const std::shared_ptr<GoalHandleGaze> goal_handle,
               uint64_t my_gen)
  {
    const auto goal = goal_handle->get_goal();
    const std::string tf_name =
      goal->gaze_tf_name.empty() ? gaze_tf_name_ : goal->gaze_tf_name;

    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Gaze started. Target TF: '%s'", tf_name.c_str());

    auto feedback = std::make_shared<GazeHead::Feedback>();
    auto result   = std::make_shared<GazeHead::Result>();

    const double dt = std::max(1, gaze_period_ms_) / 1000.0;
    const float  max_pan_step  = static_cast<float>(max_pan_vel_  * dt);
    const float  max_tilt_step = static_cast<float>(max_tilt_vel_ * dt);

    // Seed the smoothed command from the current head pose so the first tick
    // does not snap from a stale value. Fall back to 0 if not yet received.
    float cmd_pan = 0.0f, cmd_tilt = 0.0f;
    {
      std::lock_guard<std::mutex> lock(head_pose_mtx_);
      if (head_pose_valid_) {
        cmd_pan  = cur_pan_;
        cmd_tilt = cur_tilt_;
      }
    }

    while (rclcpp::ok()) {
      // Preempted by a newer goal: stop quietly so the new loop owns the head.
      if (my_gen != active_gen_.load()) {
        RCLCPP_INFO(this->get_logger(),
          "GazeController.-> Gaze preempted by a newer goal.");
        result->success = false;
        goal_handle->abort(result);
        return;
      }

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "GazeController.-> Gaze canceled.");
        // Final head command (after the gaze loop has stopped publishing), so
        // the head settles at the requested restore pose, not the gaze target.
        publish_restore_head(goal);
        return;
      }

      std_msgs::msg::Float32MultiArray target;
      if (gaze_point_to_head_angles(target, tf_name)) {
        // Slew-rate limit: step cmd_* toward the target by at most max_*_step.
        // Normalize the pan error to the shortest direction first.
        const float pan_err  = normalize_angle(target.data[0] - cmd_pan);
        const float tilt_err = target.data[1] - cmd_tilt;

        cmd_pan  += std::clamp(pan_err,  -max_pan_step,  max_pan_step);
        cmd_tilt += std::clamp(tilt_err, -max_tilt_step, max_tilt_step);
        cmd_pan   = normalize_angle(cmd_pan);

        std_msgs::msg::Float32MultiArray msg;
        msg.data = {cmd_pan, cmd_tilt};
        pub_head_goal_pose_->publish(msg);

        feedback->pan  = cmd_pan;
        feedback->tilt = cmd_tilt;
        goal_handle->publish_feedback(feedback);
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(1, gaze_period_ms_)));
    }

    result->success = false;
    goal_handle->abort(result);
    publish_restore_head(goal);
  }

  // Publish the restore head once as the final head command when gaze ends.
  void publish_restore_head(const std::shared_ptr<const GazeHead::Goal> &goal)
  {
    if (!goal->restore_head)
      return;
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {goal->restore_pan, goal->restore_tilt};
    pub_head_goal_pose_->publish(msg);
    RCLCPP_INFO(this->get_logger(),
      "GazeController.-> Restored head to [%.3f, %.3f] on gaze end.",
      goal->restore_pan, goal->restore_tilt);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
