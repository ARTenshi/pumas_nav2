#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

// Message types
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"

#include "nav_msgs/srv/get_map.hpp"
#include "std_srvs/srv/trigger.hpp"

// TF2 (Transform listener)
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "Eigen/Geometry"
#include "tf2/utils.hpp"
#include "tf2/time.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <utility>

class MapAugmenterNode : public rclcpp::Node {
public:
  MapAugmenterNode()
      : Node("map_augmenter_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // ############
    //  Declare parameters with default values
    this->declare_parameter("use_namespace", false);
    this->declare_parameter("use_lidar", false);
    this->declare_parameter("use_point_cloud", false);
    this->declare_parameter("use_point_cloud2", false);
    this->declare_parameter("use_online", false);
    this->declare_parameter("laser_min_x", -10.0);
    this->declare_parameter("laser_max_x", 10.0);
    this->declare_parameter("laser_min_y", -10.0);
    this->declare_parameter("laser_max_y", 10.0);
    this->declare_parameter("laser_min_z", -1.0);
    this->declare_parameter("laser_max_z", 2.0);
    this->declare_parameter("cloud_min_x", -10.0);
    this->declare_parameter("cloud_max_x", 10.0);
    this->declare_parameter("cloud_min_y", -10.0);
    this->declare_parameter("cloud_max_y", 10.0);
    this->declare_parameter("cloud_min_z", -1.0);
    this->declare_parameter("cloud_max_z", 2.0);
    this->declare_parameter("decay_factor", 10);
    this->declare_parameter("inflation_radius", 0.25);
    this->declare_parameter("cost_radius", 0.25);
    this->declare_parameter("cloud_downsampling", 1);
    this->declare_parameter("cloud_downsampling2", 1);
    this->declare_parameter("lidar_downsampling", 1);
    this->declare_parameter("point_cloud_topic", "/point_cloud");
    this->declare_parameter("point_cloud_topic2", "/point_cloud2");
    this->declare_parameter("laser_scan_topic", "/scan");
    this->declare_parameter("static_map_server", "/static_map_server/map");
    this->declare_parameter("prohibition_map_server",
                            "/prohibition_map_server/map");
    this->declare_parameter("base_link_name", "base_footprint");
    this->declare_parameter("remember_all_obstacles", false);
    this->declare_parameter("cloud_wait_timeout", 0.5);

    // Initialize internal variables from declared parameters
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("use_lidar", use_lidar_);
    this->get_parameter("use_point_cloud", use_cloud_);
    this->get_parameter("use_point_cloud2", use_cloud2_);
    this->get_parameter("use_online", use_online_);

    this->get_parameter("laser_min_x", laser_min_x_);
    this->get_parameter("laser_max_x", laser_max_x_);
    this->get_parameter("laser_min_y", laser_min_y_);
    this->get_parameter("laser_max_y", laser_max_y_);
    this->get_parameter("laser_min_z", laser_min_z_);
    this->get_parameter("laser_max_z", laser_max_z_);

    this->get_parameter("cloud_min_x", cloud_min_x_);
    this->get_parameter("cloud_max_x", cloud_max_x_);
    this->get_parameter("cloud_min_y", cloud_min_y_);
    this->get_parameter("cloud_max_y", cloud_max_y_);
    this->get_parameter("cloud_min_z", cloud_min_z_);
    this->get_parameter("cloud_max_z", cloud_max_z_);

    this->get_parameter("decay_factor", decay_factor_);
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("cost_radius", cost_radius_);
    this->get_parameter("cloud_downsampling", cloud_downsampling_);
    this->get_parameter("cloud_downsampling2", cloud_downsampling2_);
    this->get_parameter("lidar_downsampling", lidar_downsampling_);
    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("point_cloud_topic2", point_cloud_topic2_);
    this->get_parameter("laser_scan_topic", laser_scan_topic_);
    this->get_parameter("static_map_server", static_map_server_);
    this->get_parameter("prohibition_map_server", prohibition_map_server_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("remember_all_obstacles", remember_all_obstacles_);
    this->get_parameter("cloud_wait_timeout", cloud_wait_timeout_);

    cloud_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Setup parameter change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &MapAugmenterNode::on_parameter_change, this, std::placeholders::_1));

    // Initialize service clients (non-blocking)
    init_service_clients();

    // ############
    //  Publishers
    pub_augmented_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        make_name("/augmented_map"), rclcpp::QoS(10).transient_local());
    pub_augmented_cost_map_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            make_name("/augmented_cost_map"),
            rclcpp::QoS(10).transient_local());

    // ############
    //  Subscribers
    sub_clicked_point_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            make_name("/point_obstacle"), rclcpp::SensorDataQoS(),
            std::bind(&MapAugmenterNode::callback_point_obstacle, this,
                      std::placeholders::_1));

    // Cloud subscriptions are created on demand
    sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
        make_name("/navigation/map_augmenter/enable"),
        rclcpp::QoS(1).transient_local(),
        std::bind(&MapAugmenterNode::callback_enable, this,
                  std::placeholders::_1));

    sub_enable_cloud_ = this->create_subscription<std_msgs::msg::Bool>(
        make_name("/navigation/map_augmenter/enable_cloud"),
        rclcpp::QoS(1).transient_local(),
        std::bind(&MapAugmenterNode::callback_enable_cloud, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "MapAugmenter.-> Point-cloud gating enabled; subscribing to "
                "the cloud only while navigating.");

    sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MapAugmenterNode::callback_laser_scan, this,
                  std::placeholders::_1));

    // ############
    //  Advertise augmenter services
    srv_static_map_ = this->create_service<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_static_map"),
        std::bind(&MapAugmenterNode::callback_static_map, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_static_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_static_cost_map"),
        std::bind(&MapAugmenterNode::callback_static_cost_map, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_augmented_map_ = this->create_service<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_augmented_map"),
        std::bind(&MapAugmenterNode::callback_augmented_map, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_augmented_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
        make_name("/map_augmenter/get_augmented_cost_map"),
        std::bind(&MapAugmenterNode::callback_augmented_cost_map, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_are_there_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
        make_name("/map_augmenter/are_there_obstacles"),
        std::bind(&MapAugmenterNode::callback_are_there_obstacles, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_is_inside_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
        make_name("/map_augmenter/is_inside_obstacles"),
        std::bind(&MapAugmenterNode::callback_is_inside_obstacles, this,
                  std::placeholders::_1, std::placeholders::_2));

    srv_clear_obstacle_memory_ =
        this->create_service<std_srvs::srv::Trigger>(
            make_name("/map_augmenter/clear_obstacle_memory"),
            std::bind(&MapAugmenterNode::callback_clear_obstacle_memory,
                      this, std::placeholders::_1, std::placeholders::_2));

    // ############
    //  Map Augmenter main processing
    // The first map fetch is driven by the bootstrap timer in
    // init_service_clients(), not from here -- see the comment there.

    // Node-clock timer, matching simple_move / mvn_pln / potential_fields.
    // decay_factor counts iterations of this loop, so on a wall timer the
    // obstacle memory decayed in wall time while the controller consuming the
    // map advanced in simulated time - the two disagreed by the real-time
    // factor. On the node clock one iteration is 30 ms of simulated time for
    // every node in the pipeline.
    processing_timer_ = rclcpp::create_timer(
        this, this->get_clock(), rclcpp::Duration::from_seconds(0.03), // 30 ms
        std::bind(&MapAugmenterNode::map_augmenter_processing, this));

    RCLCPP_INFO(this->get_logger(),
                "MapAugmenter.-> MapAugmenterNode is ready.");
  }

private:
  // ############
  //  State variables
  int counter_ = 0;
  bool are_there_obstacles_;

  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid prohibition_map_;
  nav_msgs::msg::OccupancyGrid static_cost_map_;
  nav_msgs::msg::OccupancyGrid obstacles_map_;
  nav_msgs::msg::OccupancyGrid obstacles_inflated_map_;
  nav_msgs::msg::OccupancyGrid augmented_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Internal parameter values
  bool use_namespace_;
  bool use_lidar_;
  bool use_cloud_;
  bool use_cloud2_;
  bool use_online_;

  float laser_min_x_, laser_max_x_, laser_min_y_, laser_max_y_, laser_min_z_,
      laser_max_z_;
  float cloud_min_x_, cloud_max_x_, cloud_min_y_, cloud_max_y_, cloud_min_z_,
      cloud_max_z_;

  int decay_factor_;
  int cloud_downsampling_;
  int cloud_downsampling2_;
  int lidar_downsampling_;

  double inflation_radius_;
  double cost_radius_;
  double cloud_wait_timeout_;

  std::string point_cloud_topic_;
  std::string point_cloud_topic2_;
  std::string laser_scan_topic_;
  std::string static_map_server_;
  std::string prohibition_map_server_;

  std::string base_link_name_;

  // Persistent "memory" of every cell ever observed as obstacle.
  // /map_augmenter/clear_obstacle_memory.
  //
  bool remember_all_obstacles_ = false;
  std::set<std::pair<int, int>> memory_cells_;
  std::mutex memory_mutex_;

  // ############
  //  Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_augmented_map_;
  // Visualization only. The planner takes the cost map through
  // get_augmented_cost_map; without this topic a change of cost_radius is
  // invisible in RViz because nothing ever publishes a cost map.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      pub_augmented_cost_map_;

  // ############
  //  Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      sub_clicked_point_;

  // Episode-level gate for the on-demand point-cloud subscriptions.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  // Cloud-only override (recovery), independent of the episode-level enable.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_cloud_;

  rclcpp::CallbackGroup::SharedPtr cloud_cb_group_;
  std::mutex cloud_mutex_;

  std::atomic<bool> cloud_wait_expired_{false};

  // PointCloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_point_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;
  std::atomic<bool> point_cloud_new_{false};
  std::atomic<bool> point_cloud_ready_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_point_cloud2_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud2_;
  std::atomic<bool> point_cloud2_new_{false};
  std::atomic<bool> point_cloud2_ready_{false};

  // LaserScan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_laser_scan_;
  std::atomic<bool> laser_scan_new_{false};

  rclcpp::Time laser_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool laser_scan_ever_ = false;

  // ############
  //  Service clients
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_prohibition_map_;

  // Service servers
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_cost_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_cost_map_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_are_there_obstacles_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_is_inside_obstacles_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      srv_clear_obstacle_memory_;

  // ############
  //  Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Timer and readiness flag
  rclcpp::TimerBase::SharedPtr service_check_timer_;
  bool services_ready_ = false;
  bool is_static_map_ = false;
  bool is_prohibition_map_ = false;

  // Bootstrap retry state (see init_service_clients()). After
  // kProhibitionGraceAttempts seconds of the prohibition server answering with
  // an empty grid, come up without a prohibition layer instead of hanging.
  static constexpr int kProhibitionGraceAttempts = 10;
  int bootstrap_attempts_ = 0;
  bool prohibition_grace_over_ = false;

  // static_map_ merged with the prohibition layer but NOT yet inflated. Kept so
  // rebuild_static_maps() can re-inflate from clean geometry when
  // inflation_radius / cost_radius change at runtime.
  nav_msgs::msg::OccupancyGrid static_map_merged_;
  bool have_static_map_merged_ = false;

  // Main processing loop
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // Retries get_first_maps() when a map service responds before the map
  // server has actually loaded its map (empty 0x0 response).
  rclcpp::TimerBase::SharedPtr maps_retry_timer_;

  // ############
  //  Runtime parameter update callback
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // The static map is inflated once, when it arrives. With use_online=false
    // it is never re-fetched, so a new radius has to trigger the recompute
    // itself or it would only ever affect sensed obstacles.
    bool rebuild_static = false;

    for (const auto &param : params) {
      if (param.get_name() == "use_namespace")
        use_namespace_ = param.as_bool();
      else if (param.get_name() == "use_lidar")
        use_lidar_ = param.as_bool();
      else if (param.get_name() == "use_point_cloud")
        use_cloud_ = param.as_bool();
      else if (param.get_name() == "use_point_cloud2")
        use_cloud2_ = param.as_bool();
      else if (param.get_name() == "use_online")
        use_online_ = param.as_bool();

      else if (param.get_name() == "laser_min_x")
        laser_min_x_ = param.as_double();
      else if (param.get_name() == "laser_max_x")
        laser_max_x_ = param.as_double();
      else if (param.get_name() == "laser_min_y")
        laser_min_y_ = param.as_double();
      else if (param.get_name() == "laser_max_y")
        laser_max_y_ = param.as_double();
      else if (param.get_name() == "laser_min_z")
        laser_min_z_ = param.as_double();
      else if (param.get_name() == "laser_max_z")
        laser_max_z_ = param.as_double();

      else if (param.get_name() == "cloud_min_x")
        cloud_min_x_ = param.as_double();
      else if (param.get_name() == "cloud_max_x")
        cloud_max_x_ = param.as_double();
      else if (param.get_name() == "cloud_min_y")
        cloud_min_y_ = param.as_double();
      else if (param.get_name() == "cloud_max_y")
        cloud_max_y_ = param.as_double();
      else if (param.get_name() == "cloud_min_z")
        cloud_min_z_ = param.as_double();
      else if (param.get_name() == "cloud_max_z")
        cloud_max_z_ = param.as_double();

      else if (param.get_name() == "decay_factor")
        decay_factor_ = param.as_int();
      else if (param.get_name() == "cloud_downsampling")
        cloud_downsampling_ = param.as_int();
      else if (param.get_name() == "cloud_downsampling2")
        cloud_downsampling2_ = param.as_int();
      else if (param.get_name() == "lidar_downsampling")
        lidar_downsampling_ = param.as_int();
      else if (param.get_name() == "inflation_radius") {
        inflation_radius_ = param.as_double();
        rebuild_static = true;
      } else if (param.get_name() == "cost_radius") {
        cost_radius_ = param.as_double();
        rebuild_static = true;
      }

      else if (param.get_name() == "point_cloud_topic")
        point_cloud_topic_ = param.as_string();
      else if (param.get_name() == "point_cloud_topic2")
        point_cloud_topic2_ = param.as_string();
      else if (param.get_name() == "laser_scan_topic")
        laser_scan_topic_ = param.as_string();
      else if (param.get_name() == "static_map_server")
        static_map_server_ = param.as_string();
      else if (param.get_name() == "prohibition_map_server")
        prohibition_map_server_ = param.as_string();

      else if (param.get_name() == "base_link_name")
        base_link_name_ = param.as_string();

      else if (param.get_name() == "cloud_wait_timeout")
        cloud_wait_timeout_ = param.as_double();

      else if (param.get_name() == "remember_all_obstacles") {
        remember_all_obstacles_ = param.as_bool();
        if (!remember_all_obstacles_) {
          std::lock_guard<std::mutex> lock(memory_mutex_);
          memory_cells_.clear();
        }
      }

      else {
        result.successful = false;
        result.reason =
            "MapAugmenter.-> Unsupported parameter: " + param.get_name();
        RCLCPP_WARN(
            this->get_logger(),
            "MapAugmenter.-> Attempted to update unsupported parameter: %s",
            param.get_name().c_str());
        break;
      }
    }

    if (result.successful && rebuild_static) {
      rebuild_static_maps();
      // The obstacle layer is sized against static_map_, which just changed
      // shape only if the radius altered its extents; re-fitting is cheap and
      // keeps the two layers consistent.
      reproject_obstacles_map();
      RCLCPP_INFO(this->get_logger(),
                  "MapAugmenter.-> Static map re-inflated: inflation_radius=%.3f "
                  "cost_radius=%.3f",
                  inflation_radius_, cost_radius_);
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

  // ############
  //  Initialize service clients (non-blocking)
  void init_service_clients() {
    clt_get_static_map_ =
        this->create_client<nav_msgs::srv::GetMap>(static_map_server_);
    clt_get_prohibition_map_ =
        this->create_client<nav_msgs::srv::GetMap>(prohibition_map_server_);

    // Bootstrap loop. get_first_maps() used to run exactly once, from the
    // constructor, after a blind 1 s sleep -- so whether the map servers
    // happened to be up by then decided whether this node ever produced a map
    // at all. async_send_request() to a service with no server never resolves,
    // so is_static_map_ stayed false, process_maps() never ran,
    // obstacles_map_ stayed empty and map_augmenter_processing() bailed out
    // forever: /augmented_map silently never published, and the launch was a
    // coin flip. Keep re-requesting until the merged static map really exists.
    service_check_timer_ =
        this->create_wall_timer(std::chrono::seconds(1), [this]() {
          if (have_static_map_merged_) {
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> Bootstrap complete after %d attempt(s).",
                        bootstrap_attempts_);
            service_check_timer_->cancel();
            return;
          }

          bool is_static_map =
              clt_get_static_map_->wait_for_service(std::chrono::seconds(0));
          bool is_prohibition_map = clt_get_prohibition_map_->wait_for_service(
              std::chrono::seconds(0));
          if (is_static_map && is_prohibition_map) {
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> All map services are now available.");
            services_ready_ = true;
            service_check_timer_->cancel();
            get_first_maps();
          } else {
            if (!is_static_map)
              RCLCPP_WARN(this->get_logger(),
                          "MapAugmenter.-> Waiting for static_map service to "
                          "become available...");

            if (!is_prohibition_map)
              RCLCPP_WARN(this->get_logger(),
                          "MapAugmenter.-> Waiting for prohibition_map service "
                          "to become available...");
            return;
          }

          if (!services_ready_) {
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> All map services are now available.");
            services_ready_ = true;
          }

          // A map server can answer before it has finished loading its YAML and
          // hand back a 0 x 0 grid (seen on the prohibition layer), so one
          // successful round-trip is not proof of a usable map. After the grace
          // period stop insisting on the prohibition layer -- a deployment
          // without prohibition zones must still come up.
          ++bootstrap_attempts_;
          prohibition_grace_over_ =
              bootstrap_attempts_ > kProhibitionGraceAttempts;
          get_first_maps();
        });
  }

  // Wait for transforms
  void wait_for_transforms(const std::string &target_frame,
                           const std::string &source_frame) {
    RCLCPP_INFO(this->get_logger(),
                "MapAugmenter.-> Waiting for transform from '%s' to '%s'...",
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
                             "MapAugmenter.-> Still waiting for transform: %s",
                             ex.what());
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (!transform_ok) {
      RCLCPP_WARN(this->get_logger(),
                  "MapAugmenter.-> Timeout while waiting for transform from "
                  "'%s' to '%s'.",
                  source_frame.c_str(), target_frame.c_str());
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "MapAugmenter.-> Transform from '%s' to '%s' is now available.",
          source_frame.c_str(), target_frame.c_str());
    }
  }

  // Schedules a single retry of get_first_maps(). The map services can be
  // advertised (wait_for_service succeeds) before the underlying map-server
  // lifecycle node has actually loaded its map, so a GetMap call right after
  // service discovery may return a valid but empty (0x0) response.
  void schedule_maps_retry() {
    if (maps_retry_timer_)
      return;

    maps_retry_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      maps_retry_timer_->cancel();
      maps_retry_timer_.reset();
      get_first_maps();
    });
  }

  void get_first_maps() {
    auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_static_map_->async_send_request(
        static_map_req,
        [this](
            rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future_static) {
          try {
            auto map = future_static.get()->map;
            if (map.info.width == 0 || map.info.height == 0) {
              RCLCPP_WARN(this->get_logger(),
                          "MapAugmenter.-> Static map service returned an "
                          "empty map (0x0); the map server may still be "
                          "loading. Retrying...");
              schedule_maps_retry();
              return;
            }
            this->static_map_ = map;
            is_static_map_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> Got static map with size %d x %d",
                        static_map_.info.width, static_map_.info.height);
            process_maps();
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "MapAugmenter.-> Failed to get static map: %s",
                         e.what());
          }
        });

    auto prohibition_map_req =
        std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_prohibition_map_->async_send_request(
        prohibition_map_req,
        [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture
                   future_ptohibition) {
          try {
            auto map = future_ptohibition.get()->map;
            if (map.info.width == 0 || map.info.height == 0) {
              RCLCPP_WARN(this->get_logger(),
                          "MapAugmenter.-> Prohibition map service returned "
                          "an empty map (0x0); the map server may still be "
                          "loading. Retrying...");
              schedule_maps_retry();
              return;
            }
            this->prohibition_map_ = map;
            is_prohibition_map_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "MapAugmenter.-> Got prohibition map with size %d x %d",
                        prohibition_map_.info.width,
                        prohibition_map_.info.height);
            process_maps();
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "MapAugmenter.-> Failed to get prohibition map: %s",
                         e.what());
          }
        });
  }

  void process_maps() {
    if (is_static_map_ && is_prohibition_map_) {
      RCLCPP_INFO(this->get_logger(),
                  "MapAugmenter.-> Updating static map with prohibition layer "
                  "and static cost map...");

      // Keep the merged-but-NOT-inflated map so the inflation can be redone
      // later with a different radius. inflate_map() below overwrites
      // static_map_, so without this copy the pristine geometry is gone and
      // re-inflating would compound on top of the previous inflation.
      static_map_merged_ = merge_maps(static_map_, prohibition_map_);
      have_static_map_merged_ = true;

      rebuild_static_maps();
      reproject_obstacles_map();

      is_static_map_ = false;
      is_prohibition_map_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "MapAugmenter.-> Statics maps have been updated.");
    }
  }

  // Recompute the inflated static map and the static cost map from the
  // untouched merged map. Split out of process_maps() so a runtime change of
  // inflation_radius / cost_radius can take effect without waiting for a fresh
  // static map -- with use_online=false the map is fetched exactly once at
  // startup, so before this the new radius applied only to sensed obstacles and
  // the walls kept their startup inflation forever.
  void rebuild_static_maps() {
    if (!have_static_map_merged_) return;

    static_map_ = inflate_map(static_map_merged_, inflation_radius_);
    static_cost_map_ = get_cost_map(static_map_, cost_radius_);
  }

  // Re-fit obstacles_map_ to the current static-map geometry while KEEPING the
  // sensed obstacles. This used to zero the whole layer, which under
  // use_online=true wiped every point-cloud obstacle a few ms after each
  // get_augmented_map request (the static map is re-fetched asynchronously on
  // every request, and its response callback lands here).
  //
  // Under online SLAM the grid grows and its origin shifts, so surviving cells
  // are re-projected through world coordinates instead of copied index-wise.
  void reproject_obstacles_map() {
    const auto &new_info = static_map_.info;
    const auto &old_info = obstacles_map_.info;

    const bool same_geometry =
        !obstacles_map_.data.empty() && old_info.width == new_info.width &&
        old_info.height == new_info.height &&
        old_info.resolution == new_info.resolution &&
        old_info.origin.position.x == new_info.origin.position.x &&
        old_info.origin.position.y == new_info.origin.position.y;

    if (same_geometry) {
      obstacles_map_.header = static_map_.header;
      return;
    }

    nav_msgs::msg::OccupancyGrid new_map = static_map_;
    std::fill(new_map.data.begin(), new_map.data.end(), 0);

    if (!obstacles_map_.data.empty() && old_info.resolution > 0.0) {
      const int old_w = static_cast<int>(old_info.width);
      const int old_h = static_cast<int>(old_info.height);
      for (int y = 0; y < old_h; ++y) {
        for (int x = 0; x < old_w; ++x) {
          const int8_t v =
              obstacles_map_.data[static_cast<size_t>(y) * old_w + x];
          if (v <= 0)
            continue;
          const double wx =
              old_info.origin.position.x + (x + 0.5) * old_info.resolution;
          const double wy =
              old_info.origin.position.y + (y + 0.5) * old_info.resolution;
          int cell = 0;
          if (world_to_cell(new_map, wx, wy, cell) && new_map.data[cell] < v)
            new_map.data[cell] = v;
        }
      }
    }

    obstacles_map_ = std::move(new_map);
  }

  // ############
  //  On-demand point-cloud subscription management
  void create_cloud_subscriptions() {
    rclcpp::SubscriptionOptions options;
    options.callback_group = cloud_cb_group_;
    cloud_wait_expired_ = false;

    if (use_cloud_ && !sub_point_cloud_) {
      point_cloud_new_ = false;
      point_cloud_ready_ = false;
      sub_point_cloud_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              point_cloud_topic_, rclcpp::SensorDataQoS(),
              std::bind(&MapAugmenterNode::callback_point_cloud, this,
                        std::placeholders::_1),
              options);
    }
    if (use_cloud2_ && !sub_point_cloud2_) {
      point_cloud2_new_ = false;
      point_cloud2_ready_ = false;
      sub_point_cloud2_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              point_cloud_topic2_, rclcpp::SensorDataQoS(),
              std::bind(&MapAugmenterNode::callback_point_cloud2, this,
                        std::placeholders::_1),
              options);
    }
  }

  void destroy_cloud_subscriptions() {
    cloud_wait_expired_ = false;
    if (sub_point_cloud_) {
      sub_point_cloud_.reset();
      point_cloud_new_ = false;
      point_cloud_ready_ = false;
    }
    if (sub_point_cloud2_) {
      sub_point_cloud2_.reset();
      point_cloud2_new_ = false;
      point_cloud2_ready_ = false;
    }
  }

  void wait_for_first_cloud() {
    if (cloud_wait_expired_)
      return;

    const bool wait_cloud =
        use_cloud_ && sub_point_cloud_ && !point_cloud_ready_;
    const bool wait_cloud2 =
        use_cloud2_ && sub_point_cloud2_ && !point_cloud2_ready_;
    if (!wait_cloud && !wait_cloud2)
      return;

    const rclcpp::Time start = this->now();
    const rclcpp::Duration timeout =
        rclcpp::Duration::from_seconds(cloud_wait_timeout_);

    while (rclcpp::ok() && (this->now() - start) < timeout) {
      if ((!wait_cloud || point_cloud_ready_) &&
          (!wait_cloud2 || point_cloud2_ready_))
        return;
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    cloud_wait_expired_ = true;
    RCLCPP_WARN(this->get_logger(),
                "MapAugmenter.-> Timed out after %.2f s waiting for the first "
                "point cloud; augmenting without it for this episode.",
                cloud_wait_timeout_);
  }

  void callback_enable(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      if ((use_cloud_ && !sub_point_cloud_) ||
          (use_cloud2_ && !sub_point_cloud2_)) {
        RCLCPP_INFO(this->get_logger(),
                    "MapAugmenter.-> Navigation active; subscribing to point "
                    "cloud.");
        create_cloud_subscriptions();
      }
    } else {
      if (sub_point_cloud_ || sub_point_cloud2_) {
        RCLCPP_INFO(this->get_logger(),
                    "MapAugmenter.-> Navigation idle; unsubscribing from point "
                    "cloud to save bandwidth.");
        destroy_cloud_subscriptions();
      }
    }
  }

  void callback_enable_cloud(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      if ((use_cloud_ && !sub_point_cloud_) ||
          (use_cloud2_ && !sub_point_cloud2_)) {
        RCLCPP_INFO(this->get_logger(),
                    "MapAugmenter.-> Cloud re-enabled; subscribing to point "
                    "cloud.");
        create_cloud_subscriptions();
      }
    } else {
      if (sub_point_cloud_ || sub_point_cloud2_) {
        RCLCPP_INFO(this->get_logger(),
                    "MapAugmenter.-> Cloud disabled (recovery); unsubscribing "
                    "from point cloud.");
        destroy_cloud_subscriptions();
      }
    }
  }

  // Sensor callbacks
  void
  callback_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!use_cloud_) {
      point_cloud_new_ = false;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_point_cloud_ = msg;
    }
    point_cloud_new_ = true;
    point_cloud_ready_ = true;
  }

  void
  callback_point_cloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // See callback_point_cloud(): use_point_cloud2 is the single source of
    // truth.
    if (!use_cloud2_) {
      point_cloud2_new_ = false;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_point_cloud2_ = msg;
    }
    point_cloud2_new_ = true;
    point_cloud2_ready_ = true;
  }

  void callback_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_laser_scan_ = msg;
    laser_scan_new_ = true;
    laser_scan_stamp_ = this->now();
    laser_scan_ever_ = true;
  }

  // ############
  // Map Augmenter additional functions
  // get_robot_position() = get_relative_position("map", base_link_name_)
  // get_camera_position() = get_relative_position(base_link_name_,
  // point_cloud_frame_) get_camera_position2() =
  // get_relative_position(base_link_name_, point_cloud_frame2_)
  // get_lidar_position() =  get_relative_position(base_link_name_,
  // laser_scan_frame_)
  Eigen::Affine3d get_relative_position(const std::string &target_frame,
                                        const std::string &source_frame) {
    try {
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(target_frame, source_frame,
                                     tf2::TimePointZero);

      // Convert to Eigen
      Eigen::Affine3d e = tf2::transformToEigen(transform.transform);
      return e;
    } catch (const tf2::TransformException &ex) {
      // RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> TF Exception: %s",
      // ex.what());
      return Eigen::Affine3d::Identity();
    }
  }

  // get_robot_position(x, y, t) = get_relative_position("map", base_link_name_,
  // x, y, t)
  void get_relative_position(const std::string &target_frame,
                             const std::string &source_frame, float &robot_x,
                             float &robot_y, float &robot_t) {
    try {
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(target_frame, source_frame,
                                     tf2::TimePointZero);

      robot_x = static_cast<float>(transform.transform.translation.x);
      robot_y = static_cast<float>(transform.transform.translation.y);

      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      robot_t = static_cast<float>(tf2::getYaw(q));
    } catch (const tf2::TransformException &ex) {
      robot_x = 0.0f;
      robot_y = 0.0f;
      robot_t = 0.0f;
    }
  }

  // Map-frame world coords -> flat cell index. Both axes are range-checked
  // BEFORE the index is formed: validating only the flat index lets a column
  // outside [0, width) wrap into the neighbouring row, stamping the obstacle at
  // the wrong place (or on the opposite edge of the map).
  static bool world_to_cell(const nav_msgs::msg::OccupancyGrid &map, double wx,
                            double wy, int &cell) {
    const double res = map.info.resolution;
    if (res <= 0.0)
      return false;

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const int cx =
        static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
    const int cy =
        static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));

    if (cx < 0 || cy < 0 || cx >= width || cy >= height)
      return false;

    cell = cy * width + cx;
    return true;
  }

  nav_msgs::msg::OccupancyGrid
  merge_maps(const nav_msgs::msg::OccupancyGrid &a,
             const nav_msgs::msg::OccupancyGrid &b) {
    // Overlay semantics: the overlay map `b` (prohibition / sensor obstacles)
    // contributes ONLY obstacle (positive) cells; everywhere else keep the base
    // map `a`. This preserves unknown (-1) in the base map instead of turning
    // -1 into 0 (free) via max(-1, 0). Preserving -1 lets the path planner
    // decide via use_online whether unknown space is navigable
    // (use_online=true: -1 reachable for SLAM; use_online=false: -1 blocked
    // because entering unknown space is dangerous in a known environment).

    // Fast path: identical geometry -> direct index-wise overlay.
    if (a.info.width == b.info.width && a.info.height == b.info.height &&
        a.info.resolution == b.info.resolution &&
        a.info.origin.position.x == b.info.origin.position.x &&
        a.info.origin.position.y == b.info.origin.position.y) {
      nav_msgs::msg::OccupancyGrid c = a;
      for (size_t i = 0; i < c.data.size(); ++i) {
        if (b.data[i] > 0)
          c.data[i] = std::max(a.data[i], b.data[i]);
      }
      return c;
    }

    // fixed map size is different by ry0hei-kobayashi
    if (a.info.resolution <= 0.0 || b.info.resolution <= 0.0 || b.data.empty())
      return a;

    nav_msgs::msg::OccupancyGrid c = a;
    const double ax0 = a.info.origin.position.x;
    const double ay0 = a.info.origin.position.y;
    const double bx0 = b.info.origin.position.x;
    const double by0 = b.info.origin.position.y;
    const double ares = a.info.resolution;
    const double bres = b.info.resolution;
    const int aw = static_cast<int>(a.info.width);
    const int ah = static_cast<int>(a.info.height);
    const int bw = static_cast<int>(b.info.width);
    const int bh = static_cast<int>(b.info.height);
    for (int by = 0; by < bh; ++by) {
      for (int bx = 0; bx < bw; ++bx) {
        const int8_t v = b.data[static_cast<size_t>(by) * bw + bx];
        if (v <= 0)
          continue;
        // Overlay cell center -> world -> base cell index.
        const double wx = bx0 + (bx + 0.5) * bres;
        const double wy = by0 + (by + 0.5) * bres;
        const int ax = static_cast<int>((wx - ax0) / ares);
        const int ay = static_cast<int>((wy - ay0) / ares);
        if (ax < 0 || ay < 0 || ax >= aw || ay >= ah)
          continue;
        const size_t idx = static_cast<size_t>(ay) * aw + ax;
        if (c.data[idx] < v)
          c.data[idx] = v;
      }
    }
    return c;
  }

  nav_msgs::msg::OccupancyGrid
  inflate_map(const nav_msgs::msg::OccupancyGrid &map, float inflation) {
    /*
     * WARNING!!! It is assumed that map borders (borders with at least
     * 'inflation' thickness) are occupied or unkwnon. Map must be big enough to
     * fulfill this assumption.
     */
    if (inflation <= 0)
      return map;

    nav_msgs::msg::OccupancyGrid new_map = map;
    int n = static_cast<int>(inflation / map.info.resolution);
    int lower_limit = n * map.info.width + n;
    int upper_limit =
        static_cast<int>(map.data.size()) - n * map.info.width - n;

    for (int k = lower_limit; k < upper_limit; ++k) {
      if (map.data[k] > 0) {
        int col = k % static_cast<int>(map.info.width);
        for (int i = -n; i <= n; ++i) {
          if (col + i < 0 || col + i >= static_cast<int>(map.info.width))
            continue; // skip horizontal wrap_around path
          for (int j = -n; j <= n; ++j) {
            int idx = k + j * map.info.width + i;
            if (idx >= 0 && idx < static_cast<int>(new_map.data.size())) {
              new_map.data[idx] = map.data[k];
            }
          }
        }
      }
    }

    return new_map;
  }

  nav_msgs::msg::OccupancyGrid
  get_cost_map(const nav_msgs::msg::OccupancyGrid &map, float cost_radius) {
    if (cost_radius < 0)
      return map;

    nav_msgs::msg::OccupancyGrid cost_map = map;
    int steps = static_cast<int>(cost_radius / map.info.resolution);

    if (steps < 1)
      return cost_map; // cost radius (black inflation) smaller than one cell;
                       // no inflation cost

    const int NearnessToObstacle = 6;
    // add by ry0hei-kobayashi 2026/6/5, original impl made by Marco Negrete.
    // This function calculates the "nearness to obstacles", e.g., for the
    // following grid:
    /*
      0 0 0 0 0 0 0 0 0 0 0 0 0 0
      0 0 x x x 0 0 0 0 0 0 0 0 0
      0 0 x x 0 0 0 0 0 0 0 0 0 0
      0 0 x x 0 0 0 0 0 0 0 0 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0

      // the resulting nearness values would be:

      2 3 3 3 3 3 2 1 0 1 1 1 1 1
      2 3 x x x 3 2 1 0 1 2 2 2 2
      2 3 x x 3 3 2 1 0 1 2 3 3 3
      2 3 x x 3 2 2 1 0 1 2 3 x x
      2 3 3 3 3 2 1 1 0 1 2 3 x x
      2 2 2 2 2 2 1 0 0 1 2 3 3 3

      Max nearness value will depend on the distance of influence.
     */

    int box_size = (steps * 2 + 1) * (steps * 2 + 1);
    std::vector<int> cell_costs(box_size);
    std::vector<int> neighbors(box_size);
    int counter = 0;
    for (int i = -steps; i <= steps; ++i) {
      for (int j = -steps; j <= steps; ++j) {
        neighbors[counter] = i * map.info.width + j;

        int d = std::max(std::abs(i), std::abs(j)); // Chebyshev distance [cell]
        cell_costs[counter] = NearnessToObstacle * (steps - d) / steps;
        // cell_costs[counter] =
        //     (steps - std::max(std::abs(i), std::abs(j)) + 1) * 2; // old impl
        ++counter;
      }
    }

    int start_idx = steps * map.info.width + steps;
    int end_idx =
        static_cast<int>(map.data.size()) - steps * map.info.width - steps;

    for (int i = start_idx; i < end_idx; ++i) {
      if (map.data[i] > 0) {
        for (int j = 0; j < box_size; ++j) {
          int neighbor_idx = i + neighbors[j];
          if (neighbor_idx >= static_cast<int>(cost_map.data.size()))
            continue; // skip horizontal wrap_around path, fix by r.k
          // if (neighbor_idx >= 0 &&
          //     neighbor_idx < static_cast<int>(cost_map.data.size())) {
          if (neighbor_idx < 0 ||
              neighbor_idx >= static_cast<int>(cost_map.data.size()))
            continue;
          if (std::abs((neighbor_idx % static_cast<int>(map.info.width)) -
                       (i % static_cast<int>(map.info.width))) > steps)
            continue;
          if (cost_map.data[neighbor_idx] < cell_costs[j]) {
            cost_map.data[neighbor_idx] = static_cast<int8_t>(cell_costs[j]);
          }
        }
      }
    }

    return cost_map;
  }

  // Insert one cell into the persistent memory set. The point is quantized to a
  // map-frame METRIC grid key anchored at the world origin, so the key is
  // independent of the current map's width/origin (which drift under online
  // SLAM). The corresponding array index is recomputed at re-apply time.
  void add_memory_obstacle(const Eigen::Vector3d &point) {
    const double res = static_map_.info.resolution;
    if (res <= 0.0)
      return;

    int mx = static_cast<int>(std::floor(point.x() / res));
    int my = static_cast<int>(std::floor(point.y() / res));

    std::lock_guard<std::mutex> lock(memory_mutex_);
    memory_cells_.insert(std::make_pair(mx, my));
  }

  void apply_memory_obstacles() {
    if (!remember_all_obstacles_)
      return;
    std::lock_guard<std::mutex> lock(memory_mutex_);
    const double res = obstacles_map_.info.resolution;
    if (res <= 0.0)
      return;
    for (const auto &key : memory_cells_) {
      const double wx = (key.first + 0.5) * res;
      const double wy = (key.second + 0.5) * res;
      int cell = 0;
      if (world_to_cell(obstacles_map_, wx, wy, cell))
        obstacles_map_.data[cell] = 100;
    }
  }

  void callback_clear_obstacle_memory(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    std::lock_guard<std::mutex> lock(memory_mutex_);
    memory_cells_.clear();
    response->success = true;
    response->message = "MapAugmenter.-> Memory obstacles have been cleared.";
  }

  // Stamp one PointCloud2 into obstacles_map_. Shared by both cloud inputs.
  void stamp_cloud(const sensor_msgs::msg::PointCloud2 &cloud,
                   int downsampling) {
    const unsigned char *p = cloud.data.data();

    Eigen::Affine3d cam_to_robot =
        get_relative_position(base_link_name_, cloud.header.frame_id);
    Eigen::Affine3d robot_to_map =
        get_relative_position("map", base_link_name_);

    // Head-follow: rotate the cloud box to the camera's horizontal viewing yaw.
    const Eigen::Vector3d view =
        cam_to_robot.linear() * Eigen::Vector3d::UnitZ();
    const double cam_yaw = (std::hypot(view.x(), view.y()) > 0.1)
                               ? std::atan2(view.y(), view.x())
                               : 0.0;
    const double cyaw = std::cos(cam_yaw), syaw = std::sin(cam_yaw);

    for (size_t i = 0; i < cloud.width * cloud.height; i += downsampling) {
      Eigen::Vector3d v(*reinterpret_cast<const float *>(p),
                        *reinterpret_cast<const float *>(p + 4),
                        *reinterpret_cast<const float *>(p + 8));

      v = cam_to_robot * v;

      const double xr = cyaw * v.x() + syaw * v.y();
      const double yr = -syaw * v.x() + cyaw * v.y();
      if (xr > cloud_min_x_ && xr < cloud_max_x_ && yr > cloud_min_y_ &&
          yr < cloud_max_y_ && v.z() > cloud_min_z_ && v.z() < cloud_max_z_) {

        v = robot_to_map * v;
        if (remember_all_obstacles_)
          add_memory_obstacle(v);

        int cell = 0;
        if (world_to_cell(obstacles_map_, v.x(), v.y(), cell)) {
          obstacles_map_.data[cell] = 100;
          are_there_obstacles_ = true;
        }
      }

      p += static_cast<std::size_t>(downsampling * cloud.point_step);
    }
  }

  void obstacles_map_with_cloud() {
    // Consume the new-frame flag: a frame is stamped exactly once. Without the
    // exchange a stalled sensor would have its last cloud re-stamped every
    // cycle, outrunning the decay and pinning stale obstacles forever.
    if (!point_cloud_new_.exchange(false))
      return;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_point_cloud_;
    }
    if (!cloud)
      return;

    stamp_cloud(*cloud, cloud_downsampling_);
  }

  void obstacles_map_with_cloud2() {
    if (!point_cloud2_new_.exchange(false))
      return;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_point_cloud2_;
    }
    if (!cloud)
      return;

    stamp_cloud(*cloud, cloud_downsampling2_);
  }

  void obstacles_map_with_lidar() {
    if (!laser_scan_new_.exchange(false)) {

      if (!laser_scan_ever_ ||
          (this->now() - laser_scan_stamp_).seconds() > 1.0)
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "MapAugmenter.-> No new LaserScan available.");
      return;
    }

    Eigen::Affine3d lidar_to_robot = get_relative_position(
        base_link_name_, latest_laser_scan_->header.frame_id);
    Eigen::Affine3d robot_to_map =
        get_relative_position("map", base_link_name_);

    for (size_t i = 0; i < latest_laser_scan_->ranges.size();
         i += lidar_downsampling_) {
      float range = latest_laser_scan_->ranges[i];
      float angle = latest_laser_scan_->angle_min +
                    i * latest_laser_scan_->angle_increment;
      Eigen::Vector3d v(range * std::cos(angle), range * std::sin(angle), 0.0);
      v = lidar_to_robot * v;

      if (v.x() > laser_min_x_ && v.x() < laser_max_x_ &&
          v.y() > laser_min_y_ && v.y() < laser_max_y_ &&
          v.z() > laser_min_z_ && v.z() < laser_max_z_) {
        v = robot_to_map * v;
        if (remember_all_obstacles_)
          add_memory_obstacle(v);

        int cell = 0;
        if (world_to_cell(obstacles_map_, v.x(), v.y(), cell)) {
          obstacles_map_.data[cell] = 100;
          are_there_obstacles_ = true;
        }
      }
    }
  }

  void obstacles_map_with_sensors() {
    if (use_lidar_)
      obstacles_map_with_lidar();
    if (use_cloud_)
      obstacles_map_with_cloud();
    if (use_cloud2_)
      obstacles_map_with_cloud2();
  }

  bool decay_map_and_check_if_obstacles(nav_msgs::msg::OccupancyGrid &map,
                                        int decay_factor) {
    bool obstacles = false;
    for (size_t i = 0; i < map.data.size(); i++) {
      map.data[i] -= decay_factor;
      if (map.data[i] < 0)
        map.data[i] = 0;
      obstacles |= map.data[i] > 0;
    }
    return obstacles;
  }

  // ############
  // Map Augmenter subscribers callbacks
  void callback_point_obstacle(
      const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    (void)msg;
    RCLCPP_INFO(this->get_logger(),
                "MapAugmenter.-> new PointStamped received, re-fetching maps...");
    get_first_maps();
  }

  // ############
  // Map Augmenter services callbacks
  void callback_static_map(
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
    (void)request; // If unused
    response->map = static_map_;
  }

  void callback_static_cost_map(
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
    (void)request; // unused
    response->map = static_cost_map_;
  }

  void callback_augmented_map(
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
    if (!services_ready_) {
      RCLCPP_ERROR(this->get_logger(),
                   "MapAugmenter.-> Services not ready. Cannot handle "
                   "augmented map request.");
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "MapAugmenter.->Augmenting map using:
    // %s%s%s",
    //             use_lidar_ ? "lidar " : "",
    //             use_cloud_ ? "point_cloud " : "",
    //             use_cloud2_ ? "point_cloud2" : "");

    if (use_online_) {
      auto static_map_req = std::make_shared<nav_msgs::srv::GetMap::Request>();
      clt_get_static_map_->async_send_request(
          static_map_req,
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture
                     future_static) {
            try {
              this->static_map_ = future_static.get()->map;
              is_static_map_ = true;
              // RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got static map
              // with size %d x %d",
              //             static_map_.info.width, static_map_.info.height);
              process_maps();
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(),
                           "MapAugmenter.-> Failed to get static map: %s",
                           e.what());
            }
          });

      auto prohibition_map_req =
          std::make_shared<nav_msgs::srv::GetMap::Request>();
      clt_get_prohibition_map_->async_send_request(
          prohibition_map_req,
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture
                     future_ptohibition) {
            try {
              this->prohibition_map_ = future_ptohibition.get()->map;
              is_prohibition_map_ = true;
              // RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Got
              // prohibition map with size %d x %d",
              //             prohibition_map_.info.width,
              //             prohibition_map_.info.height);
              process_maps();
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(),
                           "MapAugmenter.-> Failed to get prohibition map: %s",
                           e.what());
            }
          });
    }

    // Wait for the first cloud frame of the episode, then fold in whatever the
    // sensors have delivered since the last processing cycle.
    wait_for_first_cloud();
    obstacles_map_with_sensors();

    // Include remembered obstacles in the map handed to the path planner
    // (no-op unless remember_all_obstacles is enabled).
    apply_memory_obstacles();

    obstacles_inflated_map_ = inflate_map(obstacles_map_, inflation_radius_);
    augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
    response->map = augmented_map_;

    // RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> Augmented map response
    // has been sent with size %d x %d",
    //             static_map_.info.width, static_map_.info.height);
  }

  void callback_augmented_cost_map(
      const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
      std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
    (void)request; // unused

    auto obs_cost_map = get_cost_map(obstacles_inflated_map_, cost_radius_);
    response->map = merge_maps(static_cost_map_, obs_cost_map);
  }

  void callback_are_there_obstacles(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // unused
    response->success = are_there_obstacles_;
  }

  void callback_is_inside_obstacles(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // unused

    float robot_x, robot_y, robot_a;
    get_relative_position("map", base_link_name_, robot_x, robot_y, robot_a);

    int cell = 0;
    response->success = world_to_cell(augmented_map_, robot_x, robot_y, cell) &&
                        augmented_map_.data[cell] > 0;
  }

  // ############
  // Map Augmenter main processing
  void map_augmenter_processing() {
    // Normal for the first second or two of a launch, and this loop runs at
    // ~33 Hz, so throttle instead of spamming one line per tick.
    if (!services_ready_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "MapAugmenter.-> Waiting for the map services "
                           "before augmenting.");
      return;
    }

    // Nothing to augment until process_maps() has sized the obstacle layer.
    if (obstacles_map_.data.empty())
      return;

    static int counter = 0;

    if (++counter > 10) {
      counter = 0;

      are_there_obstacles_ =
          decay_map_and_check_if_obstacles(obstacles_map_, decay_factor_);

      obstacles_map_with_sensors();

      apply_memory_obstacles();
      if (remember_all_obstacles_) {
        std::lock_guard<std::mutex> lock(memory_mutex_);
        are_there_obstacles_ = are_there_obstacles_ || !memory_cells_.empty();
      }

      obstacles_inflated_map_ = inflate_map(obstacles_map_, inflation_radius_);
      augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
      pub_augmented_map_->publish(augmented_map_);

      // Visualization copy of what callback_augmented_cost_map() hands the
      // planner. get_cost_map() is O(occupied cells * steps^2), so only pay for
      // it while something is actually looking. The static half is already
      // cached in static_cost_map_ (rebuilt by rebuild_static_maps() whenever
      // inflation_radius / cost_radius change), so the per-cycle work is just
      // the obstacle layer.
      if (pub_augmented_cost_map_->get_subscription_count() > 0) {
        auto obs_cost_map = get_cost_map(obstacles_inflated_map_, cost_radius_);
        pub_augmented_cost_map_->publish(
            merge_maps(static_cost_map_, obs_cost_map));
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapAugmenterNode>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
