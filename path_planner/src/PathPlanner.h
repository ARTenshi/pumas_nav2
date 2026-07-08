#include <queue>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    // max_goal_relocation_dist > 0 enables best-effort planning: when the goal
    // is unreachable (e.g. enclosed by furniture outlines), A* returns a path to
    // the nearest reachable cell, provided that cell is within this many meters
    // of the requested goal. 0 (default) keeps strict behavior (fail on
    // unreachable goal).
    static bool AStar(const nav_msgs::msg::OccupancyGrid &map,
                      const nav_msgs::msg::OccupancyGrid &cost_map,
                      const geometry_msgs::msg::Pose &start_pose,
                      const geometry_msgs::msg::Pose &goal_pose,
                      bool diagonal_paths,
                      nav_msgs::msg::Path &result_path,
                      bool use_online = false,
                      double max_goal_relocation_dist = 0.0);

    static nav_msgs::msg::Path SmoothPath(
                      const nav_msgs::msg::Path& path,
                      float weight_data = 0.1,
                      float weight_smooth = 0.9,
                      float tolerance = 0.00001);

    static void addViaPointBias(nav_msgs::msg::OccupancyGrid &cost_map,
                      const geometry_msgs::msg::Pose &via,
                      double radius, int cost_bias);

    // max_goal_relocation_dist applies only to the final goal segment (via
    // points are still planned strictly). See AStar above.
    static bool AStarWithViaPoints(const nav_msgs::msg::OccupancyGrid &map,
                      const nav_msgs::msg::OccupancyGrid &cost_map,
                      const geometry_msgs::msg::Pose &start_pose,
                      const std::vector<geometry_msgs::msg::Pose> &via_poses,
                      const geometry_msgs::msg::Pose &goal_pose,
                      bool diagonal_paths,
                      nav_msgs::msg::Path &result_path,
                      bool use_online = false,
                      double max_goal_relocation_dist = 0.0);
};

class Node
{
public:
    Node();
    ~Node();

    int   index;           //The index of the corresponding cell in the occupancy grid.
    float g_value;        //The accumulated distance of this node.
    float f_value;         //The f-value, used only in the A* algorithm.
    bool  in_open_list;    //A value indicating whether this node is in the open list or not.
    bool  in_closed_list;  //A value indicating whether this node is in the closed list or not.
    Node* parent;          //A pointer to the parent of this node.
};

class CompareByDistance
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->g_value > n2->g_value; }
};

class CompareByFValue
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->f_value > n2->f_value; }
};
