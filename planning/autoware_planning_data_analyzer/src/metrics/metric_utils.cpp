// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "metric_utils.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/utility/Utilities.h>
#include <tf2/utils.h>

#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
  }
}

struct CenterlinePoint
{
  double x{0.0};
  double y{0.0};
};

std::vector<CenterlinePoint> build_centerline_points(const lanelet::ConstLanelets & lanelets)
{
  std::vector<CenterlinePoint> points;
  for (const auto & lanelet : lanelets) {
    for (const auto & point : lanelet.centerline()) {
      const CenterlinePoint centerline_point{point.x(), point.y()};
      if (
        !points.empty() &&
        std::hypot(points.back().x - centerline_point.x, points.back().y - centerline_point.y) <
          1.0e-6) {
        continue;
      }
      points.push_back(centerline_point);
    }
  }
  return points;
}

}  // namespace

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return vehicle_info.vehicle_length_m > 0.0 && vehicle_info.vehicle_width_m > 0.0;
}

double get_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  return tf2::getYaw(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
}

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto local_footprint = vehicle_info.createFootprint(0.0);
  autoware_utils_geometry::Polygon2d polygon;
  polygon.outer() = autoware_utils_geometry::transform_vector(
    local_footprint, autoware_utils_geometry::pose2transform(pose));
  boost::geometry::correct(polygon);
  return polygon;
}

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady()) {
    return std::nullopt;
  }

  lanelet::ConstLanelet closest_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_lanelet)) {
    return closest_lanelet;
  }

  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      return lanelet;
    }
  }

  return std::nullopt;
}

lanelet::ConstLanelets collect_route_relevant_lanelets(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets route_lanelets;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return route_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & point : trajectory.points) {
    for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(point.pose)) {
      if (!route_handler->isRouteLanelet(lanelet)) {
        continue;
      }
      append_unique_lanelet(lanelet, route_lanelets, seen_ids);
    }
  }

  return route_lanelets;
}

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line)
{
  autoware_utils_geometry::LineString2d line_2d;
  for (const auto & point : lanelet::utils::to2D(line)) {
    line_2d.push_back({point.x(), point.y()});
  }
  return line_2d;
}

std::optional<CenterlineArcCoordinate> get_centerline_arc_coordinate(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & pose)
{
  const auto points = build_centerline_points(lanelets);
  if (points.size() < 2U) {
    return std::nullopt;
  }

  const double px = pose.position.x;
  const double py = pose.position.y;
  double cumulative_length = 0.0;
  double best_squared_distance = std::numeric_limits<double>::max();
  CenterlineArcCoordinate best_coordinate;

  for (std::size_t i = 1U; i < points.size(); ++i) {
    const auto & start = points.at(i - 1U);
    const auto & end = points.at(i);
    const double vx = end.x - start.x;
    const double vy = end.y - start.y;
    const double segment_length = std::hypot(vx, vy);
    if (segment_length < 1.0e-6) {
      continue;
    }

    const double wx = px - start.x;
    const double wy = py - start.y;
    const double t = std::clamp((wx * vx + wy * vy) / (segment_length * segment_length), 0.0, 1.0);
    const double projected_x = start.x + t * vx;
    const double projected_y = start.y + t * vy;
    const double dx = px - projected_x;
    const double dy = py - projected_y;
    const double squared_distance = dx * dx + dy * dy;

    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_coordinate.length = cumulative_length + t * segment_length;
      best_coordinate.distance = (vx * (py - start.y) - vy * (px - start.x)) / segment_length;
    }

    cumulative_length += segment_length;
  }

  if (best_squared_distance == std::numeric_limits<double>::max()) {
    return std::nullopt;
  }
  return best_coordinate;
}

std::optional<double> get_lateral_distance_to_centerline(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  const auto coordinate = get_centerline_arc_coordinate(lanelet::ConstLanelets{lanelet}, pose);
  if (!coordinate.has_value()) {
    return std::nullopt;
  }
  return coordinate->distance;
}

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto lanelet = find_reference_lanelet(pose, route_handler);
  return lanelet.has_value() &&
         autoware::experimental::lanelet2_utils::is_intersection_lanelet(*lanelet);
}

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  return std::cos(yaw) * dx + std::sin(yaw) * dy;
}

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return forward_offset_in_ego_frame(ego_pose, object_pose) < 0.0;
}

const autoware_perception_msgs::msg::PredictedPath * highest_confidence_path(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto it = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.confidence < rhs.confidence; });
  return it == object.kinematics.predicted_paths.end() ? nullptr : &(*it);
}

}  // namespace autoware::planning_data_analyzer::metrics
