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

#include "ttc_within_bound.hpp"

#include "metric_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

constexpr double kStoppedSpeedThreshold = 5.0e-3;
constexpr std::array<double, 4> kFutureProjectionOffsetsSec{{0.0, 0.3, 0.6, 0.9}};

tf2::Vector3 get_velocity_in_world_coordinate(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const double yaw = get_yaw(point.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return tf2::Vector3(
    c * point.longitudinal_velocity_mps - s * point.lateral_velocity_mps,
    s * point.longitudinal_velocity_mps + c * point.lateral_velocity_mps, 0.0);
}

geometry_msgs::msg::Pose project_pose(
  const geometry_msgs::msg::Pose & pose, const tf2::Vector3 & velocity_world, const double delta_t)
{
  auto projected = pose;
  projected.position.x += velocity_world.x() * delta_t;
  projected.position.y += velocity_world.y() * delta_t;
  return projected;
}

bool is_agent_ahead(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return forward_offset_in_ego_frame(ego_pose, object_pose) > 0.0;
}

}  // namespace

TTCWithinBoundResult calculate_ttc_within_bound(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedPredictedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  TTCWithinBoundResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (future_objects.empty()) {
    result.reason = "unavailable_no_future_objects";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  result.available = true;
  result.score = 1.0;
  result.reason = "available";

  const auto object_tracks = build_logged_object_tracks(future_objects);
  if (object_tracks.empty()) {
    return result;
  }

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);

  for (const auto & point : trajectory.points) {
    const auto velocity_world = get_velocity_in_world_coordinate(point);
    const double speed = std::hypot(velocity_world.x(), velocity_world.y());
    if (speed < kStoppedSpeedThreshold) {
      continue;
    }

    const bool ego_in_intersection = is_pose_in_intersection(point.pose, route_handler);

    for (const double future_offset_s : kFutureProjectionOffsetsSec) {
      const double query_time_s =
        rclcpp::Duration(point.time_from_start).seconds() + future_offset_s;
      const auto query_time = trajectory_start_time + rclcpp::Duration::from_seconds(query_time_s);
      const auto projected_pose = project_pose(point.pose, velocity_world, future_offset_s);
      const auto ego_polygon = create_pose_footprint(projected_pose, local_footprint);

      for (const auto & object_track : object_tracks) {
        const auto object_state = interpolate_logged_object_state(object_track, query_time);
        if (!object_state.has_value()) {
          continue;
        }

        if (!boost::geometry::intersects(ego_polygon, object_state->polygon)) {
          continue;
        }

        if (
          is_agent_ahead(point.pose, object_state->pose) ||
          (ego_in_intersection && !is_agent_behind(point.pose, object_state->pose))) {
          result.score = 0.0;
          result.reason = "collision_within_bound";
          result.infraction_time_s = query_time_s;
          return result;
        }
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
