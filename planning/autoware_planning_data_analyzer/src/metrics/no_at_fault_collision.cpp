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

#include "no_at_fault_collision.hpp"

#include "metric_utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
namespace bg = boost::geometry;

constexpr double kStoppedSpeedThreshold = 5.0e-2;

enum class CollisionType {
  StoppedEgo,
  StoppedTrack,
  ActiveRear,
  ActiveFront,
  ActiveLateral,
};

struct EgoAreaFlags
{
  bool multiple_lanes{false};
  bool non_drivable_area{false};
};

struct AtFaultCollision
{
  double score{1.0};
  std::string reason{"available"};
};

double ego_speed(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
}

bool front_bumper_intersects(
  const geometry_msgs::msg::Pose & ego_pose, const Polygon2d & object_polygon,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double yaw = get_yaw(ego_pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto transform = [&](const double x_local, const double y_local) {
    return Point2d{
      ego_pose.position.x + c * x_local - s * y_local,
      ego_pose.position.y + s * x_local + c * y_local};
  };

  bg::model::linestring<Point2d> front_bumper;
  front_bumper.push_back(
    transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m));
  front_bumper.push_back(
    transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m));
  return bg::intersects(front_bumper, object_polygon);
}

bool is_track_stopped(const InterpolatedLoggedObject & object_state)
{
  return !is_agent_classification(object_state.classification) ||
         object_state.speed_mps <= kStoppedSpeedThreshold;
}

CollisionType classify_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const InterpolatedLoggedObject & object_state,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (ego_speed(ego_point) <= kStoppedSpeedThreshold) {
    return CollisionType::StoppedEgo;
  }
  if (is_track_stopped(object_state)) {
    return CollisionType::StoppedTrack;
  }
  if (is_agent_behind(ego_point.pose, object_state.pose)) {
    return CollisionType::ActiveRear;
  }
  if (front_bumper_intersects(ego_point.pose, object_state.polygon, vehicle_info)) {
    return CollisionType::ActiveFront;
  }
  return CollisionType::ActiveLateral;
}

bool footprint_intersects_lanelet(
  const Polygon2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  return !bg::disjoint(footprint, lanelet.polygon2d().basicPolygon());
}

std::vector<Point2d> footprint_vertices(const Polygon2d & footprint)
{
  std::vector<Point2d> vertices;
  for (const auto & point : footprint.outer()) {
    if (vertices.empty() || !bg::equals(vertices.front(), point)) {
      vertices.push_back(point);
    }
  }
  return vertices;
}

lanelet::BoundingBox2d footprint_bounding_box(const Polygon2d & footprint)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : footprint.outer()) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }

  constexpr double kSearchMargin = 1.0e-3;
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{min_x - kSearchMargin, min_y - kSearchMargin},
    lanelet::BasicPoint2d{max_x + kSearchMargin, max_y + kSearchMargin}};
}

lanelet::ConstLanelets collect_candidate_road_lanelets(
  const Polygon2d & ego_polygon, const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets road_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & lanelet : map->laneletLayer.search(footprint_bounding_box(ego_polygon))) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    if (seen_ids.insert(lanelet.id()).second) {
      road_lanelets.push_back(lanelet);
    }
  }

  return road_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_candidate_parking_lots(
  const Polygon2d & ego_polygon, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstPolygon3d> parking_lots;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return parking_lots;
  }

  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & polygon : map->polygonLayer.search(footprint_bounding_box(ego_polygon))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == "parking_lot") {
      parking_lots.push_back(polygon);
    }
  }

  return parking_lots;
}

bool point_in_lanelet(const Point2d & point, const lanelet::ConstLanelet & lanelet)
{
  return bg::covered_by(
    lanelet::BasicPoint2d{point.x(), point.y()}, lanelet.polygon2d().basicPolygon());
}

bool point_in_parking_lot(const Point2d & point, const lanelet::ConstPolygon3d & parking_lot)
{
  return bg::covered_by(
    lanelet::BasicPoint2d{point.x(), point.y()}, lanelet::utils::to2D(parking_lot).basicPolygon());
}

bool detect_multiple_lanes(
  const std::vector<Point2d> & footprint_points, const lanelet::ConstLanelets & road_lanelets)
{
  if (footprint_points.empty() || road_lanelets.empty()) {
    return false;
  }

  std::size_t occupied_lanelets = 0;
  bool single_lane_contains_all_points = false;
  for (const auto & lanelet : road_lanelets) {
    std::size_t contained_points = 0;
    for (const auto & point : footprint_points) {
      if (point_in_lanelet(point, lanelet)) {
        ++contained_points;
      }
    }

    if (contained_points > 0U) {
      ++occupied_lanelets;
    }
    if (contained_points == footprint_points.size()) {
      single_lane_contains_all_points = true;
    }
  }

  return occupied_lanelets > 1U && !single_lane_contains_all_points;
}

bool detect_non_drivable_area(
  const std::vector<Point2d> & footprint_points, const lanelet::ConstLanelets & road_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  if (footprint_points.empty()) {
    return false;
  }

  for (const auto & point : footprint_points) {
    bool corner_drivable = false;
    for (const auto & lanelet : road_lanelets) {
      if (point_in_lanelet(point, lanelet)) {
        corner_drivable = true;
        break;
      }
    }
    if (!corner_drivable) {
      for (const auto & parking_lot : parking_lots) {
        if (point_in_parking_lot(point, parking_lot)) {
          corner_drivable = true;
          break;
        }
      }
    }
    if (!corner_drivable) {
      return true;
    }
  }

  return false;
}

std::optional<EgoAreaFlags> compute_ego_area_flags(
  const geometry_msgs::msg::Pose & pose, const Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler) {
    return std::nullopt;
  }
  if (!route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  auto road_lanelets = collect_candidate_road_lanelets(ego_polygon, route_handler);
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (
      route_handler->isRoadLanelet(lanelet) && footprint_intersects_lanelet(ego_polygon, lanelet)) {
      const auto duplicate = std::any_of(
        road_lanelets.begin(), road_lanelets.end(),
        [&lanelet](const auto & candidate) { return candidate.id() == lanelet.id(); });
      if (!duplicate) {
        road_lanelets.push_back(lanelet);
      }
    }
  }
  const auto parking_lots = collect_candidate_parking_lots(ego_polygon, route_handler);
  const auto points = footprint_vertices(ego_polygon);

  EgoAreaFlags flags;
  flags.multiple_lanes = detect_multiple_lanes(points, road_lanelets);
  flags.non_drivable_area = detect_non_drivable_area(points, road_lanelets, parking_lots);
  return flags;
}

AtFaultCollision make_at_fault_collision(
  const InterpolatedLoggedObject & object, const bool lateral_collision)
{
  const bool agent = is_agent_classification(object.classification);
  return AtFaultCollision{
    agent ? 0.0 : 0.5, lateral_collision ? (agent ? "at_fault_lateral_collision_with_agent"
                                                  : "at_fault_lateral_collision_with_non_agent")
                                         : (agent ? "at_fault_collision_with_agent"
                                                  : "at_fault_collision_with_non_agent")};
}

}  // namespace

NoAtFaultCollisionResult calculate_no_at_fault_collision(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedPredictedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  NoAtFaultCollisionResult result;

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
  std::set<std::array<uint8_t, 16>> collided_object_ids;
  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);

  for (const auto & point : trajectory.points) {
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto ego_polygon = create_pose_footprint(point.pose, local_footprint);

    for (const auto & object_track : object_tracks) {
      if (
        object_track.has_valid_object_id &&
        collided_object_ids.count(object_track.object_id) > 0U) {
        continue;
      }

      const auto object_state = interpolate_logged_object_state(object_track, query_time);
      if (!object_state.has_value()) {
        continue;
      }
      if (!bg::intersects(ego_polygon, object_state->polygon)) {
        continue;
      }
      if (object_state->has_valid_object_id) {
        collided_object_ids.insert(object_state->object_id);
      }

      const auto collision_type = classify_collision(point, *object_state, vehicle_info);

      const bool front_or_stopped_track = collision_type == CollisionType::ActiveFront ||
                                          collision_type == CollisionType::StoppedTrack;
      const bool lateral_collision = collision_type == CollisionType::ActiveLateral;

      if (front_or_stopped_track) {
        const auto at_fault_collision = make_at_fault_collision(*object_state, false);
        if (at_fault_collision.score < result.score) {
          result.score = at_fault_collision.score;
          result.reason = at_fault_collision.reason;
          result.infraction_time_s = query_time_s;
        }
        if (result.score <= 0.0) {
          return result;
        }
        continue;
      }

      if (lateral_collision) {
        const auto ego_area_flags = compute_ego_area_flags(point.pose, ego_polygon, route_handler);
        if (!ego_area_flags.has_value()) {
          result.available = false;
          result.score = 0.0;
          result.reason = !route_handler
                            ? "unavailable_no_route_handler_for_lateral_assessment"
                            : "unavailable_route_handler_not_ready_for_lateral_assessment";
          return result;
        }

        if (ego_area_flags->multiple_lanes || ego_area_flags->non_drivable_area) {
          const auto at_fault_collision = make_at_fault_collision(*object_state, true);
          if (at_fault_collision.score < result.score) {
            result.score = at_fault_collision.score;
            result.reason = at_fault_collision.reason;
            result.infraction_time_s = query_time_s;
          }
          if (result.score <= 0.0) {
            return result;
          }
        }
      }
    }
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
