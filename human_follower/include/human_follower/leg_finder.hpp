#ifndef LEG_FINDER_NODE_HPP_
#define LEG_FINDER_NODE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"

#define FILTER_THRESHOLD  .081
#define FLANK_THRESHOLD  .04
#define HORIZON_THRESHOLD  9
#define MAX_FLOAT  57295779500
#define LEG_THIN  0.00341
#define LEG_THICK  0.0567
#define TWO_LEGS_THIN  0.056644
#define TWO_LEGS_THICK  0.25
#define TWO_LEGS_NEAR  0.0022201
#define TWO_LEGS_FAR  0.25
#define IS_LEG_THRESHOLD 0.5
#define IN_FRONT_MIN_X  0.25
#define IN_FRONT_MAX_X  1.5
#define IN_FRONT_MIN_Y -0.5
#define IN_FRONT_MAX_Y  0.5
#define BFA0X 1.0
#define BFA1X -1.760041880343169
#define BFA2X 1.182893262037831
#define BFA3X -0.278059917634546
#define BFB0X 0.018098933007514
#define BFB1X 0.054296799022543
#define BFB2X 0.054296799022543
#define BFB3X 0.018098933007514
#define BFA0Y 1.0
#define BFA1Y -1.760041880343169
#define BFA2Y 1.182893262037831
#define BFA3Y -0.278059917634546
#define BFB0Y 0.018098933007514
#define BFB1Y 0.054296799022543
#define BFB2Y 0.054296799022543
#define BFB3Y 0.018098933007514

class LegFinderNode : public rclcpp::Node
{
public:
    LegFinderNode();

private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_stop;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLaserScan;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_legs_hypothesis;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_legs_pose;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_legs_found;

    // TF
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    

    // Variables
    bool legs_found = false;
    bool stop_robot = false;
    int legs_in_front_cnt = 0;
    int legs_lost_counter = 0;
    float last_legs_pose_x = 0;
    float last_legs_pose_y = 0;
    std::vector<float> legs_x_filter_input;
    std::vector<float> legs_x_filter_output;
    std::vector<float> legs_y_filter_input;
    std::vector<float> legs_y_filter_output;
    std::string frame_id = "base_link";
    std::string laser_scan_topic = "/scan";
    std::string laser_scan_frame = "laser";
    bool show_hypothesis = true;
    int scan_downsampling = 1;

    // Callbacks
    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_stop(const std_msgs::msg::Empty::SharedPtr msg);

    // Helper functions
    std::vector<float> downsample_scan(const std::vector<float>& ranges, int downsampling);
    std::vector<float> filter_laser_ranges(const std::vector<float>& laser_ranges);
    bool is_leg(float x1, float y1, float x2, float y2);
    bool obst_in_front(const sensor_msgs::msg::LaserScan& laser, float xmin, float xmax, float ymin, float ymax, float thr);
    Eigen::Affine3d get_lidar_position();
    void find_leg_hypothesis(const sensor_msgs::msg::LaserScan& laser, std::vector<float>& legs_x, std::vector<float>& legs_y);
    visualization_msgs::msg::Marker get_hypothesis_marker(const std::vector<float>& legs_x, const std::vector<float>& legs_y);
    bool get_nearest_legs_in_front(const std::vector<float>& legs_x, const std::vector<float>& legs_y, float& nearest_x, float& nearest_y);
    bool get_nearest_legs_to_last_legs(const std::vector<float>& legs_x, const std::vector<float>& legs_y,
                                        float& nearest_x, float& nearest_y, float last_x, float last_y);
};

#endif // LEG_FINDER_NODE_HPP_
