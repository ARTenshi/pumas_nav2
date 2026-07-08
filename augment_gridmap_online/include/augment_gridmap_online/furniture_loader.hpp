// Self-contained helper for loading static furniture obstacles described in
// hma_env_manage2-style env_furniture YAML files and stamping them as cells
// into a nav_msgs::OccupancyGrid.
//
// All functions are header-only / inline. The header takes an rclcpp::Logger
// where logging is needed so the caller controls the logger name.
//
// Expected YAML schema (excerpt):
//
//   furniture:
//     bookcase:
//       frame: "map"
//       plane: [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
//       collision:
//         is_collision: true       # optional, default true
//         height_range: [0.0, 0.3] # ignored here (2D)
//
// Entries with `is_collision: false` are skipped. Entries with fewer than 3
// vertices are skipped with a warning.

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace augment_gridmap_online {

struct FurniturePolygon {
  std::string name;
  std::vector<std::pair<double, double>> vertices; // (x, y) in map frame
};

// Resolve absolute path to the furniture YAML. `absolute_yaml` wins when
// non-empty; otherwise builds <share(pkg)>/io/config/<path>.yaml via
// ament_index_cpp (rospkg-equivalent runtime lookup, no package.xml depend
// on `pkg` required). Returns "" when nothing is configured or the lookup
// fails (the latter is logged as an error).
inline std::string
resolve_furniture_yaml_path(const std::string &pkg, const std::string &path,
                            const std::string &absolute_yaml,
                            const rclcpp::Logger &logger) {
  if (!absolute_yaml.empty()) {
    return absolute_yaml;
  }
  if (pkg.empty() || path.empty()) {
    return "";
  }
  try {
    const std::string share =
        ament_index_cpp::get_package_share_directory(pkg);
    return share + "/io/config/" + path + ".yaml";
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        logger,
        "AugmentedGridMap.-> Cannot resolve share dir for package '%s': %s",
        pkg.c_str(), e.what());
    return "";
  }
}

// Parse the YAML at `yaml_path` and return one FurniturePolygon per valid
// entry. Returns an empty vector on any load failure (logged).
inline std::vector<FurniturePolygon>
load_furniture_polygons_from_yaml(const std::string &yaml_path,
                                  const rclcpp::Logger &logger,
                                  bool debug = false) {
  std::vector<FurniturePolygon> out;
  if (yaml_path.empty()) {
    return out;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger,
                 "AugmentedGridMap.-> Failed to load furniture YAML '%s': %s",
                 yaml_path.c_str(), e.what());
    return out;
  }

  if (!root["furniture"] || !root["furniture"].IsMap()) {
    RCLCPP_WARN(
        logger,
        "AugmentedGridMap.-> YAML '%s' has no 'furniture' map; nothing to load",
        yaml_path.c_str());
    return out;
  }

  const YAML::Node &furniture = root["furniture"];
  for (auto it = furniture.begin(); it != furniture.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node &f = it->second;

    // Honor collision.is_collision when present; default to true.
    bool is_collision = true;
    if (f["collision"] && f["collision"]["is_collision"]) {
      try {
        is_collision = f["collision"]["is_collision"].as<bool>();
      } catch (const std::exception &) {
        is_collision = true;
      }
    }
    if (!is_collision) {
      if (debug) {
        RCLCPP_INFO(logger,
                    "AugmentedGridMap.-> Skipping furniture '%s' "
                    "(is_collision: false)",
                    name.c_str());
      }
      continue;
    }

    if (!f["plane"] || !f["plane"].IsSequence()) {
      continue;
    }

    FurniturePolygon poly;
    poly.name = name;
    for (std::size_t i = 0; i < f["plane"].size(); ++i) {
      const YAML::Node &pt = f["plane"][i];
      if (!pt.IsSequence() || pt.size() < 2) {
        continue;
      }
      try {
        poly.vertices.emplace_back(pt[0].as<double>(), pt[1].as<double>());
      } catch (const std::exception &e) {
        RCLCPP_WARN(logger,
                    "AugmentedGridMap.-> Bad vertex in furniture '%s': %s",
                    name.c_str(), e.what());
      }
    }
    if (poly.vertices.size() >= 3) {
      out.push_back(std::move(poly));
    } else {
      RCLCPP_WARN(logger,
                  "AugmentedGridMap.-> Furniture '%s' has too few vertices "
                  "(%zu); skipped",
                  name.c_str(), poly.vertices.size());
    }
  }

  RCLCPP_INFO(logger,
              "AugmentedGridMap.-> Loaded %zu furniture polygon(s) from '%s'",
              out.size(), yaml_path.c_str());
  return out;
}

// Even-odd / ray-casting point-in-polygon test.
inline bool
point_in_polygon(double px, double py,
                 const std::vector<std::pair<double, double>> &poly) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].first, yi = poly[i].second;
    const double xj = poly[j].first, yj = poly[j].second;
    const double denom = (yj - yi);
    if (std::fabs(denom) < 1e-12) {
      continue;
    }
    const bool intersect =
        ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / denom + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

// Rasterize a polygon into the grid (cells whose centers fall inside the
// polygon are set to 100). Returns the number of cells filled.
inline int stamp_polygon_to_grid(nav_msgs::msg::OccupancyGrid &grid,
                                 const FurniturePolygon &poly) {
  if (poly.vertices.size() < 3) {
    return 0;
  }
  if (grid.data.empty()) {
    return 0;
  }

  const double x0 = grid.info.origin.position.x;
  const double y0 = grid.info.origin.position.y;
  const double res = grid.info.resolution;
  const int W = static_cast<int>(grid.info.width);
  const int H = static_cast<int>(grid.info.height);

  double minx = poly.vertices[0].first, maxx = poly.vertices[0].first;
  double miny = poly.vertices[0].second, maxy = poly.vertices[0].second;
  for (const auto &v : poly.vertices) {
    minx = std::min(minx, v.first);
    maxx = std::max(maxx, v.first);
    miny = std::min(miny, v.second);
    maxy = std::max(maxy, v.second);
  }

  int ci_min = std::max(0, static_cast<int>(std::floor((minx - x0) / res)));
  int ci_max =
      std::min(W - 1, static_cast<int>(std::ceil((maxx - x0) / res)));
  int cj_min = std::max(0, static_cast<int>(std::floor((miny - y0) / res)));
  int cj_max =
      std::min(H - 1, static_cast<int>(std::ceil((maxy - y0) / res)));

  int filled = 0;
  for (int i = ci_min; i <= ci_max; ++i) {
    const double cx = x0 + (i + 0.5) * res;
    for (int j = cj_min; j <= cj_max; ++j) {
      const double cy = y0 + (j + 0.5) * res;
      if (point_in_polygon(cx, cy, poly.vertices)) {
        grid.data[i + j * W] = 100;
        ++filled;
      }
    }
  }
  return filled;
}

// Build a MarkerArray to visualise the polygons as LINE_STRIP outlines.
// A leading DELETEALL marker wipes any previously published markers in the
// `furniture` namespace so repeated calls do not accumulate stale ones.
inline visualization_msgs::msg::MarkerArray
build_furniture_markers(const std::vector<FurniturePolygon> &polygons,
                        const std::string &frame_id,
                        const rclcpp::Time &stamp) {
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker del;
  del.header.frame_id = frame_id;
  del.header.stamp = stamp;
  del.ns = "furniture";
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  int id = 0;
  for (const auto &poly : polygons) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "furniture";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.03;
    m.color.r = 0.0;
    m.color.g = 0.6;
    m.color.b = 1.0;
    m.color.a = 1.0;
    for (const auto &v : poly.vertices) {
      geometry_msgs::msg::Point p;
      p.x = v.first;
      p.y = v.second;
      p.z = 0.0;
      m.points.push_back(p);
    }
    // Close the loop so the outline is a closed polygon.
    if (!poly.vertices.empty()) {
      geometry_msgs::msg::Point p;
      p.x = poly.vertices.front().first;
      p.y = poly.vertices.front().second;
      p.z = 0.0;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
  }
  return arr;
}

} // namespace augment_gridmap_online
