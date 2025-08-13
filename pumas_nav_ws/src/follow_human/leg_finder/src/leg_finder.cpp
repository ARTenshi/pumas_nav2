#include "follow_human_pkg/leg_finder.hpp"

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

LegFinderNode::LegFinderNode() : Node("leg_finder_node"), tf_buffer(this->get_clock(), tf2::durationFromSec(10.0)), tf_listener(tf_buffer) 
{

    this->declare_parameter("show_hypothesis", true);
    this->declare_parameter("laser_scan_topic", "/scan");
    this->declare_parameter("laser_scan_frame", "laser");
    this->declare_parameter("scan_downsampling", 1);

    show_hypothesis = this->get_parameter("show_hypothesis").as_bool();
    laser_scan_topic = this->get_parameter("laser_scan_topic").as_string();
    laser_scan_frame = this->get_parameter("laser_scan_frame").as_string();
    scan_downsampling = this->get_parameter("scan_downsampling").as_int();

    // Subscribers
    sub_enable = this->create_subscription<std_msgs::msg::Bool>(
        "/hri/leg_finder/enable", 1, std::bind(&LegFinderNode::callback_enable, this, std::placeholders::_1));
    sub_stop = this->create_subscription<std_msgs::msg::Empty>(
        "/stop", 1, std::bind(&LegFinderNode::callback_stop, this, std::placeholders::_1));

    // Publishers
    pub_legs_hypothesis = this->create_publisher<visualization_msgs::msg::Marker>(
        "/hri/leg_finder/hypothesis", 1);
    pub_legs_pose = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/hri/leg_finder/leg_pose", 1);
    pub_legs_found = this->create_publisher<std_msgs::msg::Bool>(
        "/hri/leg_finder/legs_found", 1);

    // tf2_ros::Buffer tf_buffer{this->get_clock()};
    // // TF Listener
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

    // Initialize filter input/output vectors
    legs_x_filter_input.resize(4, 0);
    legs_x_filter_output.resize(4, 0);
    legs_y_filter_input.resize(4, 0);
    legs_y_filter_output.resize(4, 0);

    // Initialization message
    RCLCPP_INFO(this->get_logger(), "LegFinderNode initialized.");
}

// Callbacks
void LegFinderNode::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // std::cout << "Callback scan" << std::endl;
    if(scan_downsampling > 1)
        msg->ranges = downsample_scan(msg->ranges, scan_downsampling);
    msg->ranges = filter_laser_ranges(msg->ranges);
    std::vector<float> legs_x, legs_y;
    find_leg_hypothesis(*msg, legs_x, legs_y);
    if(show_hypothesis)
    {
        std::cout << "Num of Found legs: " << legs_x.size() << "  " << legs_y.size() << std::endl;
        pub_legs_hypothesis->publish(get_hypothesis_marker(legs_x, legs_y));
    }

    float nearest_x, nearest_y;
    if(!legs_found && !stop_robot)
    {
        if(get_nearest_legs_in_front(legs_x, legs_y, nearest_x, nearest_y))
            legs_in_front_cnt++;
        if(legs_in_front_cnt > 20)
        {
            legs_found = true;
            legs_lost_counter = 0;
            last_legs_pose_x = nearest_x;
            last_legs_pose_y = nearest_y;
            for(int i=0; i < 4; i++)
            {
                legs_x_filter_input[i]  = nearest_x;
                legs_x_filter_output[i] = nearest_x;
                legs_y_filter_input[i]  = nearest_y;
                legs_y_filter_output[i] = nearest_y;
            }
        }
    }
    else if(legs_found){
        geometry_msgs::msg::PointStamped filtered_legs;
        filtered_legs.header.frame_id = frame_id;
        filtered_legs.point.z = 0.3;

        bool fobst_in_front = false;

        if(!fobst_in_front){

            //float diff = sqrt((nearest_x - last_legs_pose_x)*(nearest_x - last_legs_pose_x) +
            //		  (nearest_y - last_legs_pose_y)*(nearest_y - last_legs_pose_y));
            bool publish_legs = false;
            if(get_nearest_legs_to_last_legs(legs_x, legs_y, nearest_x, nearest_y, last_legs_pose_x, last_legs_pose_y))
            {
                last_legs_pose_x = nearest_x;
                last_legs_pose_y = nearest_y;
                legs_x_filter_input.insert(legs_x_filter_input.begin(), nearest_x);
                legs_y_filter_input.insert(legs_y_filter_input.begin(), nearest_y);
                legs_lost_counter = 0;
                publish_legs = true;
            }
            else
            {
                legs_x_filter_input.insert(legs_x_filter_input.begin(), last_legs_pose_x);
                legs_y_filter_input.insert(legs_y_filter_input.begin(), last_legs_pose_y);
                if(++legs_lost_counter > 20)
                {
                    legs_found = false;
                    legs_in_front_cnt = 0;
                }
            }
            legs_x_filter_input.pop_back();
            legs_y_filter_input.pop_back();
            legs_x_filter_output.pop_back();
            legs_y_filter_output.pop_back();
            legs_x_filter_output.insert(legs_x_filter_output.begin(), 0);
            legs_y_filter_output.insert(legs_y_filter_output.begin(), 0);

            legs_x_filter_output[0]  = BFB0X*legs_x_filter_input[0] + BFB1X*legs_x_filter_input[1] +
            BFB2X*legs_x_filter_input[2] + BFB3X*legs_x_filter_input[3];
            legs_x_filter_output[0] -= BFA1X*legs_x_filter_output[1] + BFA2X*legs_x_filter_output[2] + BFA3X*legs_x_filter_output[3];

            legs_y_filter_output[0]  = BFB0Y*legs_y_filter_input[0] + BFB1Y*legs_y_filter_input[1] +
                BFB2Y*legs_y_filter_input[2] + BFB3Y*legs_y_filter_input[3];
            legs_y_filter_output[0] -= BFA1Y*legs_y_filter_output[1] + BFA2Y*legs_y_filter_output[2] + BFA3Y*legs_y_filter_output[3];

            filtered_legs.point.x = legs_x_filter_output[0];
            filtered_legs.point.y = legs_y_filter_output[0];
            
            stop_robot = false;
        
            if(publish_legs)
                pub_legs_pose->publish(filtered_legs);
        }
        else
            stop_robot = true;
    }
    std_msgs::msg::Bool msg_found;
    msg_found.data = legs_found;
    pub_legs_found->publish(msg_found);
}

void LegFinderNode::callback_enable(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::cout << "Callback enable" << std::endl;
    if(msg->data){
        subLaserScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_scan_topic, 1, std::bind(&LegFinderNode::callback_scan, this, std::placeholders::_1));
        legs_found = true;
    }
    else
    {
        // subLaserScan.shutdown();
        legs_found = false;
        legs_in_front_cnt = 0;
    }
}

void LegFinderNode::callback_stop(const std_msgs::msg::Empty::SharedPtr )
{
    // subLaserScan.shutdown();
    legs_found = false;
    legs_in_front_cnt = 0;
}

// Helper functions
std::vector<float> LegFinderNode::downsample_scan(const std::vector<float>& ranges, int downsampling)
{
    std::vector<float> new_scans;
    new_scans.resize(ranges.size()/downsampling);
    for(int i=0; i < static_cast<int>(ranges.size()); i+=downsampling)
        new_scans[i/downsampling] = ranges[i];

    return new_scans;
}

std::vector<float> LegFinderNode::filter_laser_ranges(const std::vector<float>& laser_ranges)
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

bool LegFinderNode::is_leg(float x1, float y1, float x2, float y2)
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

bool LegFinderNode::obst_in_front(const sensor_msgs::msg::LaserScan& laser, float xmin, float xmax, float ymin, float ymax, float thr)
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
    //std::cout << "leg_finder_node.-> quantize : " << quantize << std::endl;
    if(quantize >= thr)
        return true;
    else   
        return false;
}

Eigen::Affine3d LegFinderNode::get_lidar_position()
{
    // geometry_msgs::msg::TransformStamped tf;
    // try {
    //     tf = tf_buffer.lookupTransform("base_link", laser_scan_frame, tf2::TimePointZero);
    // } catch (tf2::TransformException &ex) {
    //     RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    //     throw;
    // }

    Eigen::Affine3d e;
    e.linear() << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

    e.translation() << 0.283, 0.001, 0.085;
    // tf2::fromMsg(tf.transform, e);
    return e;
}

void LegFinderNode::find_leg_hypothesis(const sensor_msgs::msg::LaserScan& laser, std::vector<float>& legs_x, std::vector<float>& legs_y)
{
    std::vector<float> laser_x;
    std::vector<float> laser_y;
    laser_x.resize(laser.ranges.size());
    laser_y.resize(laser.ranges.size());
    Eigen::Affine3d lidar_to_robot = get_lidar_position();
    float theta = laser.angle_min;
    for(size_t i=0; i < static_cast<int>(laser.ranges.size()); i++)
    {
        theta = laser.angle_min + i*laser.angle_increment*scan_downsampling;
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

visualization_msgs::msg::Marker LegFinderNode::get_hypothesis_marker(const std::vector<float>& legs_x, const std::vector<float>& legs_y)
{
    visualization_msgs::msg::Marker marker_legs;
    marker_legs.header.stamp = get_clock()->now();
    marker_legs.header.frame_id = frame_id;
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

bool LegFinderNode::get_nearest_legs_in_front(const std::vector<float>& legs_x, const std::vector<float>& legs_y, float& nearest_x, float& nearest_y)
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

bool LegFinderNode::get_nearest_legs_to_last_legs(const std::vector<float>& legs_x, const std::vector<float>& legs_y,
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegFinderNode>());
    rclcpp::shutdown();
    return 0;
}
