#include "PathPlanner.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>

bool PathPlanner::AStar(const nav_msgs::msg::OccupancyGrid &map,
                        const nav_msgs::msg::OccupancyGrid &cost_map,
                        const geometry_msgs::msg::Pose &start_pose,
                        const geometry_msgs::msg::Pose &goal_pose,
                        bool diagonal_paths, nav_msgs::msg::Path &result_path,
                        bool use_online, double max_goal_relocation_dist) {

  std::cout << "PathCalculator.-> Calculating by A* from "
            << start_pose.position.x << "  ";
  std::cout << start_pose.position.y << "  to " << goal_pose.position.x << "  "
            << goal_pose.position.y << std::endl;

  // use_online=true: treat unknown (-1) cells as traversable goals (SLAM mode);
  // use_online=false: only free (0) cells are navigable (default static-map
  // mode).
  auto allow_unknown_space_to_navigate = [use_online](int8_t v) {
    return use_online ? (v <= 0) : (v == 0);
  };

  int idx_start_x;
  int idx_start_y;
  int idx_goal_x;
  int idx_goal_y;
  idx_start_y = (int)((start_pose.position.y - map.info.origin.position.y) /
                      map.info.resolution);
  idx_start_x = (int)((start_pose.position.x - map.info.origin.position.x) /
                      map.info.resolution);
  int idx_start = idx_start_y * map.info.width + idx_start_x;

  idx_goal_y = (int)((goal_pose.position.y - map.info.origin.position.y) /
                     map.info.resolution);
  idx_goal_x = (int)((goal_pose.position.x - map.info.origin.position.x) /
                     map.info.resolution);
  int idx_goal = idx_goal_y * map.info.width + idx_goal_x;

  int best_idx = -1;
  double best_distance = 1e9;

  int _idx_goal_y = idx_goal_y;
  int _idx_goal_x = idx_goal_x;

  int count = 0;
  int loop_count = 0;
  // double radius = 0.1;
  double radius = 1.0;

  int MAX_GOAL_UPDATE = 16; // points on circle
  double angle_increment = 2 * M_PI / (MAX_GOAL_UPDATE);

  if (!allow_unknown_space_to_navigate(map.data[idx_goal])) {
    while (loop_count < 10) { // TODO

      double angle = count * angle_increment;
      idx_goal_y =
          static_cast<int>(std::round(_idx_goal_y + radius * cos(angle)));
      idx_goal_x =
          static_cast<int>(std::round(_idx_goal_x + radius * sin(angle)));
      // idx_goal_y = _idx_goal_y + radius * cos(angle);
      // idx_goal_x = _idx_goal_x + radius * sin(angle);

      // double world_x =
      //     idx_goal_x * map.info.resolution + map.info.origin.position.x;
      // double world_y =
      //     idx_goal_y * map.info.resolution + map.info.origin.position.y;

      // std::cout << "[relocate] ring=" << loop_count << " count=" << count
      //           << " radius(cell)=" << radius << " angle(rad)=" << angle
      //           << " idx=(" << idx_goal_x << ", " << idx_goal_y << ")"
      //           << " world=(" << world_x << ", " << world_y << ")";

      // check if not inside of map
      if (idx_goal_x >= 0 && idx_goal_x < map.info.width && idx_goal_y >= 0 &&
          idx_goal_y < map.info.height) {

        idx_goal = idx_goal_y * map.info.width + idx_goal_x;

        if (allow_unknown_space_to_navigate(map.data[idx_goal])) {
          double distance =
              std::hypot(idx_goal_x - _idx_goal_x, idx_goal_y - _idx_goal_y);
          if (distance < best_distance) {
            best_distance = distance;
            best_idx = idx_goal;
          }
        }
      }

      count++;
      if (count == MAX_GOAL_UPDATE) {
        count = 0;
        // radius += 0.1;
        radius += 1;
        loop_count++;
      }
    }
    if (best_idx != -1) {
      idx_goal = best_idx;
      idx_goal_y = idx_goal / map.info.width;
      idx_goal_x = idx_goal % map.info.width;
      std::cout << "PathPlanner.-> Goal updated to nearest free cell: "
                << idx_goal_x << ", " << idx_goal_y << std::endl;
    } else {
      // reloaction faile: restore the original goal so the checks below, fix
      // bug by r.k
      idx_goal_x = _idx_goal_x;
      idx_goal_y = _idx_goal_y;
      idx_goal = idx_goal_y * map.info.width + idx_goal_x;
      std::cout << "PathPlanner.-> Could not relocate to a free cell."
                << std::endl;
    }
  }

  // while (map.data[idx_goal] != 0 or loop_count < 4) {

  //  double angle = count * angle_increment;
  //  idx_goal_y = _idx_goal_y + radius * cos(angle);
  //  idx_goal_x = _idx_goal_x + radius * sin(angle);
  //  idx_goal = idx_goal_y * map.info.width + idx_goal_x;

  //  count++;
  //  if (count == MAX_GOAL_UPDATE) {
  //    count = 0;
  //    radius += 0.1;
  //    loop_count++;
  //  }
  //}

  if (idx_start == idx_goal) {
    result_path.header.frame_id = "map";
    result_path.poses.clear();

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose = start_pose;
    result_path.poses.push_back(p);

    std::cout << "PathPlanner.-> Start and goal are the same cell."
              << std::endl;
    return true;
  }

  if (!allow_unknown_space_to_navigate(map.data[idx_goal])) {
    // The goal cell itself is non-free (buried in an obstacle outline or on
    // unknown space) and the spiral above could not nudge it onto a free cell
    // within its small search ring (radius <= 10 cells ~= 0.5 m). When goal
    // relocation is enabled, do NOT fail here: fall through to A*, which floods
    // the reachable free space and relocates to the nearest reachable cell
    // within max_goal_relocation_dist of the requested goal (handled after the
    // search loop below, keyed off the original _idx_goal). With relocation
    // disabled keep the original fail-fast behavior. fix by r.k
    if (max_goal_relocation_dist <= 0.0) {
      std::cout << "PathPlanner.->Goal point is inside non-free space!!!!"
                << std::endl;
      return false;
    }
    std::cout << "PathPlanner.-> Goal is inside non-free space; relocating to "
                 "the nearest reachable cell (within "
              << max_goal_relocation_dist << " m)." << std::endl;
  }
  if (!allow_unknown_space_to_navigate(map.data[idx_start])) {
    std::cout << "PathPlanner.->Start point is inside non-free space!!!!"
              << std::endl;
    return false;
  }

  std::vector<Node> nodes;
  Node *current_node;
  std::vector<int> node_neighbors;
  int steps = 0;
  nodes.resize(map.data.size());
  if (diagonal_paths)
    node_neighbors.resize(8);
  else
    node_neighbors.resize(4);
  std::priority_queue<Node *, std::vector<Node *>, CompareByFValue> open_list;
  for (size_t i = 0; i < map.data.size(); i++)
    nodes[i].index = i;

  current_node = &nodes[idx_start];
  current_node->g_value = 0;
  current_node->in_open_list = true;
  open_list.push(current_node);

  // Track the reachable (closed) cell nearest the ORIGINALLY requested goal
  // (_idx_goal_x/_idx_goal_y, before any spiral relocation) so we can fall back
  // to it when the goal itself is unreachable, e.g. a person inside a furniture
  // outline. Distances are kept in cells (squared) for the comparison.
  int closest_idx = idx_start;
  long long closest_d2 =
      (long long)(idx_start_x - _idx_goal_x) * (idx_start_x - _idx_goal_x) +
      (long long)(idx_start_y - _idx_goal_y) * (idx_start_y - _idx_goal_y);

  while (!open_list.empty() && current_node->index != idx_goal) {

    current_node = open_list.top();
    open_list.pop();
    current_node->in_closed_list = true;

    {
      int cx = current_node->index % (int)map.info.width;
      int cy = current_node->index / (int)map.info.width;
      long long d2 = (long long)(cx - _idx_goal_x) * (cx - _idx_goal_x) +
                     (long long)(cy - _idx_goal_y) * (cy - _idx_goal_y);
      if (d2 < closest_d2) {
        closest_d2 = d2;
        closest_idx = current_node->index;
      }
    }

    node_neighbors[0] = current_node->index + map.info.width;
    node_neighbors[1] = current_node->index + 1;
    node_neighbors[2] = current_node->index - map.info.width;
    node_neighbors[3] = current_node->index - 1;
    if (diagonal_paths) {
      node_neighbors[4] = current_node->index + map.info.width + 1;
      node_neighbors[5] = current_node->index + map.info.width - 1;
      node_neighbors[6] = current_node->index - map.info.width + 1;
      node_neighbors[7] = current_node->index - map.info.width - 1;
    }

    for (size_t i = 0; i < node_neighbors.size(); i++) {
      int ni = node_neighbors[i];
      if (ni < 0 ||
          ni >= static_cast<int>(map.data.size())) // check out of range
        continue;

      int w = static_cast<int>(map.info.width);
      if (std::abs((ni % w) - (current_node->index % w)) >
          1) // reject horizontal wrap-around to the opposite side map edge, fix
             // bug r.k
        continue;

      if (!allow_unknown_space_to_navigate(map.data[node_neighbors[i]]) ||
          nodes[node_neighbors[i]].in_closed_list)
        continue;

      Node *neighbor = &nodes[node_neighbors[i]];
      float delta_g = i < 4 ? 1.0 : 1.414213562;
      float g_value = current_node->g_value + (i < 4 ? 1.0 : 1.414213562) +
                      cost_map.data[node_neighbors[i]];
      float h_value;
      int h_value_x = node_neighbors[i] % map.info.width - idx_goal_x;
      int h_value_y = node_neighbors[i] / map.info.width - idx_goal_y;
      if (diagonal_paths)
        h_value = sqrt(h_value_x * h_value_x + h_value_y * h_value_y);
      else
        h_value = fabs(h_value_x) + fabs(h_value_y);

      if (g_value < neighbor->g_value) {
        neighbor->g_value = g_value;
        neighbor->f_value = g_value + h_value;
        neighbor->parent = current_node;
      }

      if (!neighbor->in_open_list) {
        neighbor->in_open_list = true;
        open_list.push(neighbor);
      }
    }
    steps++;
  }
  std::cout << "PathPlanner.->A* Algorithm ended after " << steps << " steps"
            << std::endl;

  if (current_node->index != idx_goal) {
    // Goal is unreachable (the open list was exhausted without reaching it),
    // e.g. it sits inside a free pocket enclosed by furniture outlines. If
    // relocation is enabled, fall back to the nearest reachable cell (the
    // closed node closest to the requested goal) as long as it is within the
    // allowed distance; otherwise fail as before.
    if (max_goal_relocation_dist > 0.0) {
      int cx = closest_idx % (int)map.info.width;
      int cy = closest_idx / (int)map.info.width;
      double dist_m = std::hypot((double)(cx - _idx_goal_x),
                                 (double)(cy - _idx_goal_y)) *
                      map.info.resolution;
      if (dist_m <= max_goal_relocation_dist) {
        std::cout << "PathPlanner.-> Goal unreachable (enclosed). Relocated to "
                     "nearest reachable cell "
                  << dist_m << " m from requested goal." << std::endl;
        current_node = &nodes[closest_idx];
        // fall through to path reconstruction below
      } else {
        std::cout << "PathPlanner.-> Goal unreachable; nearest reachable cell "
                     "is "
                  << dist_m << " m away (> " << max_goal_relocation_dist
                  << " m). Giving up." << std::endl;
        return false;
      }
    } else {
      std::cout << "PathPlanner.-> current_node->index != idx_goal "
                << std::endl;
      return false;
    }
  }

  result_path.header.frame_id = "map";
  result_path.poses.clear();
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  while (current_node->parent != NULL) {
    p.pose.position.x =
        current_node->index % map.info.width * map.info.resolution +
        map.info.origin.position.x;
    p.pose.position.y =
        current_node->index / map.info.width * map.info.resolution +
        map.info.origin.position.y;
    result_path.poses.insert(result_path.poses.begin(), p);
    current_node = current_node->parent;
  }

  if (result_path.poses.empty()) {
    // Degenerate relocation: the robot's own cell is already the nearest
    // reachable point to the goal. Return a single-point path at the start (as
    // the start==goal branch does) so callers treat it as a valid trivial plan.
    p.pose = start_pose;
    result_path.poses.push_back(p);
  }

  std::cout << "PathCalculator.->Resulting path by A* has "
            << result_path.poses.size() << " points." << std::endl;
  return true;
}

nav_msgs::msg::Path PathPlanner::SmoothPath(const nav_msgs::msg::Path &path,
                                            float weight_data,
                                            float weight_smooth,
                                            float tolerance) {
  nav_msgs::msg::Path newPath;
  for (int i = 0; i < path.poses.size(); i++)
    newPath.poses.push_back(path.poses[i]);
  newPath.header.frame_id = "map";
  if (path.poses.size() < 3)
    return newPath;
  int attempts = 0;
  tolerance *= path.poses.size();
  float change = tolerance + 1;

  while (change >= tolerance && ++attempts < 1000) {
    change = 0;
    for (int i = 1; i < path.poses.size() - 1; i++) {
      geometry_msgs::msg::Point old_p = path.poses[i].pose.position;
      geometry_msgs::msg::Point new_p = newPath.poses[i].pose.position;
      geometry_msgs::msg::Point new_p_next = newPath.poses[i + 1].pose.position;
      geometry_msgs::msg::Point new_p_prev = newPath.poses[i - 1].pose.position;
      float last_x = newPath.poses[i].pose.position.x;
      float last_y = newPath.poses[i].pose.position.y;
      new_p.x += weight_data * (old_p.x - new_p.x) +
                 weight_smooth * (new_p_next.x + new_p_prev.x - 2.0 * new_p.x);
      new_p.y += weight_data * (old_p.y - new_p.y) +
                 weight_smooth * (new_p_next.y + new_p_prev.y - 2.0 * new_p.y);
      change += fabs(new_p.x - last_x) + fabs(new_p.y - last_y);
      newPath.poses[i].pose.position = new_p;
    }
  }
  std::cout << "PathCalculator.->Smoothing finished after " << attempts
            << " attempts" << std::endl;
  return newPath;
}

// Lower the cost of every cell within `radius` [m] of `via` by `cost_bias`
// (clamped at 0), making that region attractive to A*.
void PathPlanner::addViaPointBias(nav_msgs::msg::OccupancyGrid &cost_map,
                                  const geometry_msgs::msg::Pose &via,
                                  double radius, int cost_bias) {
  int width = cost_map.info.width;
  int height = cost_map.info.height;
  double resolution = cost_map.info.resolution;
  double origin_x = cost_map.info.origin.position.x;
  double origin_y = cost_map.info.origin.position.y;

  int center_x = (int)((via.position.x - origin_x) / resolution);
  int center_y = (int)((via.position.y - origin_y) / resolution);
  int cell_radius = (int)(radius / resolution);

  for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      int x = center_x + dx;
      int y = center_y + dy;
      if (x >= 0 && x < width && y >= 0 && y < height) {
        int idx = y * width + x;
        double dist = std::sqrt(dx * dx + dy * dy) * resolution;
        if (dist <= radius) {
          cost_map.data[idx] = static_cast<int8_t>(
              std::max(0, static_cast<int>(cost_map.data[idx]) - cost_bias));
        }
      }
    }
  }
}

bool PathPlanner::AStarWithViaPoints(
    const nav_msgs::msg::OccupancyGrid &map,
    const nav_msgs::msg::OccupancyGrid &cost_map,
    const geometry_msgs::msg::Pose &start_pose,
    const std::vector<geometry_msgs::msg::Pose> &via_poses,
    const geometry_msgs::msg::Pose &goal_pose, bool diagonal_paths,
    nav_msgs::msg::Path &result_path, bool use_online,
    double max_goal_relocation_dist) {
  result_path.poses.clear();
  nav_msgs::msg::Path partial_path;
  geometry_msgs::msg::Pose current_start = start_pose;

  for (const auto &via : via_poses) {
    nav_msgs::msg::OccupancyGrid biased_cost_map = cost_map;
    PathPlanner::addViaPointBias(biased_cost_map, via, 0.5,
                                 400); // via costs radius, bias
    partial_path.poses.clear();
    if (!PathPlanner::AStar(map, biased_cost_map, current_start, via,
                            diagonal_paths, partial_path, use_online))
      return false;
    result_path.poses.insert(result_path.poses.end(),
                             partial_path.poses.begin(),
                             partial_path.poses.end());
    current_start = via;
  }

  // Only the final goal segment may relocate to the nearest reachable cell;
  // via points are still planned strictly (passing 0.0 above).
  partial_path.poses.clear();
  if (!PathPlanner::AStar(map, cost_map, current_start, goal_pose,
                          diagonal_paths, partial_path, use_online,
                          max_goal_relocation_dist))
    return false;
  result_path.poses.insert(result_path.poses.end(), partial_path.poses.begin(),
                           partial_path.poses.end());
  return true;
}

Node::Node() {
  this->index = -1;
  this->g_value = INT_MAX;
  this->f_value = INT_MAX;
  this->in_open_list = false;
  this->in_closed_list = false;
  this->parent = NULL;
}

Node::~Node() {}
