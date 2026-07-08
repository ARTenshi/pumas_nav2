#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"

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
//Constants to check if there are legs in front of the robot
#define IN_FRONT_MIN_X  0.25
#define IN_FRONT_MAX_X  1.5
#define IN_FRONT_MIN_Y -0.5
#define IN_FRONT_MAX_Y  0.5

//BUTTERWORTH FILTER A Ó B EN X O Y
//cutoff frequency X: 0.7
//                 Y: 0.2
//lowpass filter
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
    LegFinderNode() : Node("leg_finder_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) 
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<bool>("show_hypothesis",            true);

        this->declare_parameter<std::string>("laser_scan_topic",    "/scan");
        this->declare_parameter<std::string>("laser_scan_frame",    "base_range_sensor_link");
        this->declare_parameter<std::string>("base_link_frame",     "base_footprint");

        this->declare_parameter<int>("scan_downsampling",           1);

        // Initialize internal variables from declared parameters
        this->get_parameter("show_hypothesis",      show_hypothesis_);

        this->get_parameter("laser_scan_topic",     laser_scan_topic_);
        this->get_parameter("laser_scan_frame",     laser_scan_frame_);
        this->get_parameter("base_link_frame",      base_link_frame_);
        
        this->get_parameter("scan_downsampling",    scan_downsampling_);

        // Subscribers
        sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hri/leg_finder/enable", 1, std::bind(&LegFinderNode::callback_enable, this, std::placeholders::_1));
        sub_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/stop", 1, std::bind(&LegFinderNode::callback_stop, this, std::placeholders::_1));

        // Publishers
        pub_legs_hypothesis_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/hri/leg_finder/hypothesis", rclcpp::SensorDataQoS());
        pub_legs_pose_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/hri/leg_finder/leg_pose", rclcpp::SensorDataQoS());
        pub_legs_found_ = this->create_publisher<std_msgs::msg::Bool>(
            "/hri/leg_finder/legs_found", rclcpp::SensorDataQoS());

        // Initialize filter input/output vectors
        legs_x_filter_input_.resize(4, 0);
        legs_x_filter_output_.resize(4, 0);
        legs_y_filter_input_.resize(4, 0);
        legs_y_filter_output_.resize(4, 0);

        //############
        // Wait for transforms
        wait_for_transforms(base_link_frame_, laser_scan_frame_);

        // Initialization message
        RCLCPP_INFO(this->get_logger(), "LegFinder.-> LegFinderNode is ready.");
    }

private:
    // State variables
    bool use_namespace_ = false;
    bool enable_ = false;

    bool show_hypothesis_;
    int scan_downsampling_;
    std::string laser_scan_topic_;
    std::string laser_scan_frame_;
    std::string base_link_frame_;

    // Internal parameter values
    bool legs_found_ = false;
    bool stop_robot_ = false;

    int legs_in_front_cnt_ = 0;
    int legs_lost_counter_ = 0;
    float last_legs_pose_x_ = 0;
    float last_legs_pose_y_ = 0;

    std::vector<float> legs_x_filter_input_;
    std::vector<float> legs_x_filter_output_;
    std::vector<float> legs_y_filter_input_;
    std::vector<float> legs_y_filter_output_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    //############
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   pub_legs_hypothesis_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pub_legs_pose_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               pub_legs_found_;

    //############
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            sub_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr           sub_stop_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    sub_laser_scan_;

    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name()      == "use_namespace")         use_namespace_           = param.as_bool();
            else if (param.get_name() == "show_hypothesis")       show_hypothesis_         = param.as_bool();

            else if (param.get_name() == "laser_scan_topic")      laser_scan_topic_        = param.as_string();
            else if (param.get_name() == "laser_scan_frame")      laser_scan_frame_        = param.as_string();
            else if (param.get_name() == "base_link_frame")       base_link_frame_         = param.as_string();

            else if (param.get_name() == "scan_downsampling")     scan_downsampling_       = param.as_int();

            else {
                result.successful = false;
                result.reason = "LegFinder.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "LegFinder.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
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

    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "LegFinder.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

        rclcpp::Time start_time = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0); 

        bool transform_ok = false;

        while (rclcpp::ok() && (this->now() - start_time) < timeout) {
            try {
                tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1)
                );
                transform_ok = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "LegFinder.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "LegFinder.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "LegFinder.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }

    //############
    //Leg Finder callbacks
    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // std::cout << "Callback scan" << std::endl;
        if(scan_downsampling_ > 1)
            msg->ranges = downsample_scan(msg->ranges, scan_downsampling_);
        msg->ranges = filter_laser_ranges(msg->ranges);
        std::vector<float> legs_x, legs_y;
        find_leg_hypothesis(*msg, legs_x, legs_y);
        if(show_hypothesis_)
        {
            RCLCPP_INFO(this->get_logger(), "LegFinder.-> Num of Found legs: %zu  %zu", legs_x.size(), legs_y.size());
            pub_legs_hypothesis_->publish(get_hypothesis_marker(legs_x, legs_y));
        }

        float nearest_x, nearest_y;
        if(!legs_found_ && !stop_robot_)
        {
            if(get_nearest_legs_in_front(legs_x, legs_y, nearest_x, nearest_y))
                legs_in_front_cnt_++;
            if(legs_in_front_cnt_ > 20)
            {
                legs_found_ = true;
                legs_lost_counter_ = 0;
                last_legs_pose_x_ = nearest_x;
                last_legs_pose_y_ = nearest_y;
                for(int i=0; i < 4; i++)
                {
                    legs_x_filter_input_[i]  = nearest_x;
                    legs_x_filter_output_[i] = nearest_x;
                    legs_y_filter_input_[i]  = nearest_y;
                    legs_y_filter_output_[i] = nearest_y;
                }
            }
        }
        else if(legs_found_){
            geometry_msgs::msg::PointStamped filtered_legs;
            filtered_legs.header.frame_id = base_link_frame_;
            filtered_legs.point.z = 0.3;

            bool fobst_in_front = false;

            if(!fobst_in_front){

                //float diff = sqrt((nearest_x - last_legs_pose_x_)*(nearest_x - last_legs_pose_x_) +
                //		  (nearest_y - last_legs_pose_y_)*(nearest_y - last_legs_pose_y_));
                bool publish_legs = false;
                if(get_nearest_legs_to_last_legs(legs_x, legs_y, nearest_x, nearest_y, last_legs_pose_x_, last_legs_pose_y_))
                {
                    last_legs_pose_x_ = nearest_x;
                    last_legs_pose_y_ = nearest_y;
                    legs_x_filter_input_.insert(legs_x_filter_input_.begin(), nearest_x);
                    legs_y_filter_input_.insert(legs_y_filter_input_.begin(), nearest_y);
                    legs_lost_counter_ = 0;
                    publish_legs = true;
                }
                else
                {
                    legs_x_filter_input_.insert(legs_x_filter_input_.begin(), last_legs_pose_x_);
                    legs_y_filter_input_.insert(legs_y_filter_input_.begin(), last_legs_pose_y_);
                    if(++legs_lost_counter_ > 20)
                    {
                        legs_found_ = false;
                        legs_in_front_cnt_ = 0;
                    }
                }
                legs_x_filter_input_.pop_back();
                legs_y_filter_input_.pop_back();
                legs_x_filter_output_.pop_back();
                legs_y_filter_output_.pop_back();
                legs_x_filter_output_.insert(legs_x_filter_output_.begin(), 0);
                legs_y_filter_output_.insert(legs_y_filter_output_.begin(), 0);

                legs_x_filter_output_[0]  = BFB0X*legs_x_filter_input_[0] + BFB1X*legs_x_filter_input_[1] +
                BFB2X*legs_x_filter_input_[2] + BFB3X*legs_x_filter_input_[3];
                legs_x_filter_output_[0] -= BFA1X*legs_x_filter_output_[1] + BFA2X*legs_x_filter_output_[2] + BFA3X*legs_x_filter_output_[3];

                legs_y_filter_output_[0]  = BFB0Y*legs_y_filter_input_[0] + BFB1Y*legs_y_filter_input_[1] +
                    BFB2Y*legs_y_filter_input_[2] + BFB3Y*legs_y_filter_input_[3];
                legs_y_filter_output_[0] -= BFA1Y*legs_y_filter_output_[1] + BFA2Y*legs_y_filter_output_[2] + BFA3Y*legs_y_filter_output_[3];

                filtered_legs.point.x = legs_x_filter_output_[0];
                filtered_legs.point.y = legs_y_filter_output_[0];
                
                stop_robot_ = false;
            
                if(publish_legs)
                    pub_legs_pose_->publish(filtered_legs);
            }
            else
                stop_robot_ = true;
        }
        std_msgs::msg::Bool msg_found;
        msg_found.data = legs_found_;
        pub_legs_found_->publish(msg_found);
    }

    void callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        try 
        {
            enable_ = msg->data;
            if(enable_) {
                if(!sub_laser_scan_){
                    RCLCPP_INFO(this->get_logger(), "LegFinder.->LegFinder enabled...");

                    sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                        laser_scan_topic_, rclcpp::SensorDataQoS(), 
                        std::bind(&LegFinderNode::callback_scan, this, std::placeholders::_1));
                    legs_found_ = true;
                }
            } else
            {
                if (sub_laser_scan_) {
                    sub_laser_scan_.reset();
                    RCLCPP_INFO(this->get_logger(), "LegFinder.->LegFinder disabled...");
                }

                legs_found_ = false;
                legs_in_front_cnt_ = 0;
                }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "LegFinder.-> Error processing callback_enable: %s", e.what());
        }
    }

    void callback_stop(const std_msgs::msg::Empty::SharedPtr )
    {
        if(sub_laser_scan_){
            sub_laser_scan_.reset();
            RCLCPP_INFO(this->get_logger(), "LegFinder.->LegFinder stopped...");
        }

        enable_ = false;
        legs_found_ = false;
        legs_in_front_cnt_ = 0;
    }

    //############
    //Leg Finder additional functions
    std::vector<float> downsample_scan(const std::vector<float>& ranges, int downsampling)
    {
        std::vector<float> new_scans;
        new_scans.resize(ranges.size()/downsampling);
        for(int i=0; i < static_cast<int>(ranges.size()); i+=downsampling)
            new_scans[i/downsampling] = ranges[i];

        return new_scans;
    }

    std::vector<float> filter_laser_ranges(const std::vector<float>& laser_ranges)
    {
        std::vector<float> filtered_ranges;
        filtered_ranges.resize(laser_ranges.size());
        filtered_ranges[0] = 0;
        int i = 1;
        int max_idx = laser_ranges.size() - 1;

        while(++i < max_idx)
            if(laser_ranges[i] < 0.4)
                filtered_ranges[i] = 0;

            else if(fabs(laser_ranges[i-1] - laser_ranges[i]) < FILTER_THRESHOLD &&
                    fabs(laser_ranges[i] - laser_ranges[i+1]) < FILTER_THRESHOLD)
                filtered_ranges[i] = (laser_ranges[i-1] + laser_ranges[i] + laser_ranges[i+1])/3.0;

            else if(fabs(laser_ranges[i-1] - laser_ranges[i]) < FILTER_THRESHOLD)
                filtered_ranges[i] = (laser_ranges[i-1] + laser_ranges[i])/2.0;

            else if(fabs(laser_ranges[i] - laser_ranges[i+1]) < FILTER_THRESHOLD)
                filtered_ranges[i] = (laser_ranges[i] + laser_ranges[i+1])/2.0;
            else
                filtered_ranges[i] = 0;

        filtered_ranges[i] = 0;
        return filtered_ranges;
    }

    bool is_leg(float x1, float y1, float x2, float y2)
    {
        bool result = false;
        float m1, m2, px, py, angle;
        if(x1 != x2) m1 = (y1 - y2)/(x1 - x2);
        else m1 = MAX_FLOAT;

        px = (x1 + x2) / 2;
        py = (y1 + y2) / 2;
        if((px*px + py*py) < HORIZON_THRESHOLD)
        {
            if(px != 0)
                m2 = py / px;
            else
                m2 = MAX_FLOAT;
            angle = fabs((m2 - m1) / (1 + (m2*m1)));
            if(angle > IS_LEG_THRESHOLD)
                result = true;
        }
        return result;
    }

    bool obst_in_front(const sensor_msgs::msg::LaserScan& laser, float xmin, float xmax, float ymin, float ymax, float thr)
    {
        float theta = laser.angle_min;
        float quantize = 0.0;
        for(size_t i=0; i < static_cast<int>(laser.ranges.size()); i++)
        {
            float x, y;
            theta = laser.angle_min + i*laser.angle_increment;
            x = laser.ranges[i] * cos(theta);
            y = laser.ranges[i] * sin(theta);
            if(x >= xmin && x <= xmax && y >= ymin && y <= ymax)
                quantize += laser.ranges[i];
        }
        //std::cout << "LegFinder.-> quantize : " << quantize << std::endl;
        if(quantize >= thr)
            return true;
        else   
            return false;
    }

    Eigen::Affine3d get_lidar_position()
    {
        // Get transform to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                base_link_frame_,  // target frame
                laser_scan_frame_,  // source frame
                tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "LegFinder.-> TF lookup failed: %s", ex.what());
            return Eigen::Affine3d::Identity();
        }
        Eigen::Affine3d e = tf2::transformToEigen(transform_stamped.transform);

        return e;
    }

    void find_leg_hypothesis(const sensor_msgs::msg::LaserScan& laser, std::vector<float>& legs_x, std::vector<float>& legs_y)
    {
        std::vector<float> laser_x;
        std::vector<float> laser_y;
        laser_x.resize(laser.ranges.size());
        laser_y.resize(laser.ranges.size());
        Eigen::Affine3d lidar_to_robot = get_lidar_position();
        float theta = laser.angle_min;
        for(size_t i=0; i < static_cast<int>(laser.ranges.size()); i++)
        {
            theta = laser.angle_min + i*laser.angle_increment*scan_downsampling_;
            Eigen::Vector3d v(laser.ranges[i] * cos(theta), laser.ranges[i] * sin(theta), 0);
            v = lidar_to_robot * v;
            laser_x[i] = v.x();
            laser_y[i] = v.y();
        }

        std::vector<float> flank_x;
        std::vector<float> flank_y;
        std::vector<bool>  flank_id;
        int ant2 = 0;
        float px, py, sum_x, sum_y, cua_x, cua_y;

        legs_x.clear();
        legs_y.clear();
        for(int i=1; i < static_cast<int>(laser.ranges.size()); i++)
        {
            int ant = ant2;
            if(fabs(laser.ranges[i] - laser.ranges[i-1]) > FLANK_THRESHOLD) ant2 = i;
            if(fabs(laser.ranges[i] - laser.ranges[i-1]) > FLANK_THRESHOLD &&
            (is_leg(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) || 
                    is_leg(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2])))
            {
                if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > LEG_THIN &&
                        (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < LEG_THICK)
                {
                    sum_x = 0;
                    sum_y = 0;
                    for(int j= ant; j < i; j++)
                    {
                        sum_x += laser_x[j];
                        sum_y += laser_y[j];
                    }
                    flank_x.push_back(sum_x / (float)(i - ant));
                    flank_y.push_back(sum_y / (float)(i - ant));
                    flank_id.push_back(false);
                }
                else if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > TWO_LEGS_THIN &&
                        (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < TWO_LEGS_THICK)
                {
                    sum_x = 0;
                    sum_y = 0;
                    for(int j= ant; j < i; j++)
                    {
                        sum_x += laser_x[j];
                        sum_y += laser_y[j];
                    }
                    cua_x = sum_x / (float)(i - ant);
                    cua_y = sum_y / (float)(i - ant);
                    legs_x.push_back(cua_x);
                    legs_y.push_back(cua_y);
                }
            }
        }

        for(int i=0; i < (int)(flank_x.size())-2; i++)
            for(int j=1; j < 3; j++)
                if((pow(flank_x[i] - flank_x[i+j], 2) + pow(flank_y[i] - flank_y[i+j], 2)) > TWO_LEGS_NEAR &&
                        (pow(flank_x[i] - flank_x[i+j], 2) + pow(flank_y[i] - flank_y[i+j], 2)) < TWO_LEGS_FAR)
                {
                    px = (flank_x[i] + flank_x[i + j])/2;
                    py = (flank_y[i] + flank_y[i + j])/2;
                    if((px*px + py*py) < HORIZON_THRESHOLD)
                    {
                        cua_x = px;
                        cua_y = py;
                        legs_x.push_back(cua_x);
                        legs_y.push_back(cua_y);
                        flank_id[i] = true;
                        flank_id[i+j] = true;
                    }
                }
    /*
        if(flank_y.size() > 1 &&
                (pow(flank_x[flank_x.size()-2] - flank_x[flank_x.size()-1], 2) +
                pow(flank_y[flank_y.size()-2] - flank_y[flank_y.size()-1], 2)) > TWO_LEGS_NEAR &&
                (pow(flank_x[flank_x.size()-2] - flank_x[flank_x.size()-1], 2) +
                pow(flank_y[flank_y.size()-2] - flank_y[flank_y.size()-1], 2)) < TWO_LEGS_FAR)
        {
            px = (flank_x[flank_x.size()-2] + flank_x[flank_x.size()-1])/2.0;
            py = (flank_y[flank_y.size()-2] + flank_y[flank_y.size()-1])/2.0;
            if((px*px + py*py) < HORIZON_THRESHOLD)
            {
                cua_x = px;
                cua_y = py;
                legs_x.push_back(cua_x);
                legs_y.push_back(cua_y);
                flank_id[flank_y.size() - 2] = true;
                flank_id[flank_y.size() - 1] = true;
            }
        }
    */
        for(int i=0; i < flank_y.size(); i++)
            if(!flank_id[i])
            {
                float cua_x, cua_y;
                cua_x = flank_x[i];
                cua_y = flank_y[i];
                legs_x.push_back(cua_x);
                legs_y.push_back(cua_y);
                }

            // std::cout << "LegFinder.->Found " << legs_x.size() << " leg hypothesis" << std::endl;
    }

    visualization_msgs::msg::Marker get_hypothesis_marker(const std::vector<float>& legs_x, const std::vector<float>& legs_y)
    {
        visualization_msgs::msg::Marker marker_legs;
        marker_legs.header.stamp = get_clock()->now();
        marker_legs.header.frame_id = base_link_frame_;
        marker_legs.ns = "leg_finder";
        marker_legs.id = 0;
        marker_legs.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker_legs.action = visualization_msgs::msg::Marker::ADD;
        marker_legs.scale.x = 0.07;
        marker_legs.scale.y = 0.07;
        marker_legs.scale.z = 0.07;
        marker_legs.color.a = 1.0;
        marker_legs.color.r = 0;
        marker_legs.color.g = 0.5;
        marker_legs.color.b = 0;
        marker_legs.points.resize(legs_y.size());
        marker_legs.lifetime = rclcpp::Duration::from_seconds(1.0);
        for(int i=0; i < legs_y.size(); i++)
        {
            marker_legs.points[i].x = legs_x[i];
            marker_legs.points[i].y = legs_y[i];
            marker_legs.points[i].z = 0.3;
        }
        return marker_legs;
    }

    bool get_nearest_legs_in_front(const std::vector<float>& legs_x, const std::vector<float>& legs_y, float& nearest_x, float& nearest_y)
    {
        nearest_x = MAX_FLOAT;
        nearest_y = MAX_FLOAT;
        float min_dist = MAX_FLOAT;
        for(int i=0; i < legs_x.size(); i++)
        {
            if(!(legs_x[i] > IN_FRONT_MIN_X && legs_x[i] < IN_FRONT_MAX_X && legs_y[i] > IN_FRONT_MIN_Y && legs_y[i] < IN_FRONT_MAX_Y))
                continue;
            float dist = sqrt(legs_x[i]*legs_x[i] + legs_y[i]*legs_y[i]);
            if(dist < min_dist)
            {
                min_dist = dist;
                nearest_x = legs_x[i];
                nearest_y = legs_y[i];
            }
        }
        return nearest_x > IN_FRONT_MIN_X && nearest_x < IN_FRONT_MAX_X && nearest_y > IN_FRONT_MIN_Y && nearest_y < IN_FRONT_MAX_Y;
    }

    bool get_nearest_legs_to_last_legs(const std::vector<float>& legs_x, const std::vector<float>& legs_y,
        float& nearest_x, float& nearest_y, float last_x, float last_y)
    {
        nearest_x = MAX_FLOAT;
        nearest_y = MAX_FLOAT;
        float min_dist = MAX_FLOAT;
        for(int i=0; i < legs_x.size(); i++)
        {
            float dist = sqrt((legs_x[i] - last_x)*(legs_x[i] - last_x) + (legs_y[i] - last_y)*(legs_y[i] - last_y));
            if(dist < min_dist)
            {
                min_dist = dist;
                nearest_x = legs_x[i];
                nearest_y = legs_y[i];
            }
        }
        return min_dist < 0.33;
        /*
        if(min_dist > 0.5)
        {
        nearest_x = last_x;
        nearest_y = last_y;
        return false;
        }
        return true;*/
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegFinderNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
