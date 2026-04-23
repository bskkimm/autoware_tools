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

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
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

struct CollisionClassification
{
  CollisionType type{CollisionType::ActiveLateral};
  bool ego_stopped{false};
  bool track_stopped{false};
  bool behind{false};
  bool front_hit{false};
  std::vector<geometry_msgs::msg::Point> front_bumper;
};

std::string collision_type_to_string(const CollisionType type)
{
  switch (type) {
    case CollisionType::StoppedEgo:
      return "STOPPED_EGO";
    case CollisionType::StoppedTrack:
      return "STOPPED_TRACK";
    case CollisionType::ActiveRear:
      return "ACTIVE_REAR";
    case CollisionType::ActiveFront:
      return "ACTIVE_FRONT";
    case CollisionType::ActiveLateral:
      return "ACTIVE_LATERAL";
  }
  return "NONE";
}

std::string object_id_to_string(const std::array<uint8_t, 16> & object_id, const bool valid)
{
  if (!valid) {
    return "invalid";
  }

  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (const auto byte : object_id) {
    oss << std::setw(2) << static_cast<int>(byte);
  }
  return oss.str();
}

geometry_msgs::msg::Point to_msg_point(const Point2d & point, const double z = 0.0)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = z;
  return msg;
}

geometry_msgs::msg::Point to_msg_point(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point msg;
  msg.x = pose.position.x;
  msg.y = pose.position.y;
  msg.z = pose.position.z;
  return msg;
}

std::vector<geometry_msgs::msg::Point> polygon_to_points(const Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size());
  for (const auto & point : polygon.outer()) {
    points.push_back(to_msg_point(point, z));
  }
  return points;
}

std::vector<std::vector<geometry_msgs::msg::Point>> overlap_polygons_to_points(
  const Polygon2d & ego_polygon, const Polygon2d & object_polygon, const double z)
{
  std::vector<Polygon2d> intersections;
  bg::intersection(ego_polygon, object_polygon, intersections);

  std::vector<std::vector<geometry_msgs::msg::Point>> polygons;
  polygons.reserve(intersections.size());
  for (const auto & intersection : intersections) {
    if (intersection.outer().size() >= 4U && bg::area(intersection) > 1.0e-6) {
      polygons.push_back(polygon_to_points(intersection, z));
    }
  }
  return polygons;
}

double ego_speed(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  return std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps);
}

std::vector<geometry_msgs::msg::Point> front_bumper_points(
  const geometry_msgs::msg::Pose & ego_pose,
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

  return {
    to_msg_point(
      transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m),
      ego_pose.position.z),
    to_msg_point(
      transform(vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m),
      ego_pose.position.z)};
}

bool front_bumper_intersects(
  const std::vector<geometry_msgs::msg::Point> & front_bumper_points,
  const Polygon2d & object_polygon)
{
  if (front_bumper_points.size() < 2U) {
    return false;
  }

  bg::model::linestring<Point2d> front_bumper;
  front_bumper.push_back(Point2d{front_bumper_points.at(0).x, front_bumper_points.at(0).y});
  front_bumper.push_back(Point2d{front_bumper_points.at(1).x, front_bumper_points.at(1).y});
  return bg::intersects(front_bumper, object_polygon);
}

bool is_track_stopped(const InterpolatedLoggedObject & object_state)
{
  return !is_agent_classification(object_state.classification) ||
         object_state.speed_mps <= kStoppedSpeedThreshold;
}

CollisionClassification classify_collision(
  const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const InterpolatedLoggedObject & object_state,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  CollisionClassification classification;
  classification.ego_stopped = ego_speed(ego_point) <= kStoppedSpeedThreshold;
  classification.track_stopped = is_track_stopped(object_state);
  classification.behind = is_agent_behind(ego_point.pose, object_state.pose);
  classification.front_bumper = front_bumper_points(ego_point.pose, vehicle_info);
  classification.front_hit = front_bumper_intersects(
    classification.front_bumper, object_state.polygon);

  if (classification.ego_stopped) {
    classification.type = CollisionType::StoppedEgo;
    return classification;
  }
  if (classification.track_stopped) {
    classification.type = CollisionType::StoppedTrack;
    return classification;
  }
  if (classification.behind) {
    classification.type = CollisionType::ActiveRear;
    return classification;
  }
  if (classification.front_hit) {
    classification.type = CollisionType::ActiveFront;
    return classification;
  }
  classification.type = CollisionType::ActiveLateral;
  return classification;
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

NoAtFaultCollisionDebugEvent make_debug_event(
  const double query_time_s, const autoware_planning_msgs::msg::TrajectoryPoint & ego_point,
  const Polygon2d & ego_polygon, const InterpolatedLoggedObject & object_state,
  const CollisionClassification & collision)
{
  NoAtFaultCollisionDebugEvent event;
  event.time_s = query_time_s;
  event.object_id = object_id_to_string(object_state.object_id, object_state.has_valid_object_id);
  event.object_label =
    autoware::object_recognition_utils::convertLabelToString(object_state.classification);
  event.collision_type = collision_type_to_string(collision.type);
  event.agent = is_agent_classification(object_state.classification);
  event.ego_stopped = collision.ego_stopped;
  event.track_stopped = collision.track_stopped;
  event.behind = collision.behind;
  event.front_hit = collision.front_hit;
  event.ego_center = to_msg_point(ego_point.pose);
  event.object_center = to_msg_point(object_state.pose);
  event.ego_footprint = polygon_to_points(ego_polygon, ego_point.pose.position.z);
  event.object_footprint = polygon_to_points(object_state.polygon, object_state.pose.position.z);
  event.front_bumper = collision.front_bumper;
  return event;
}

void fill_horizon_debug_footprints(
  NoAtFaultCollisionDebugInfo & debug_info,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<LoggedObjectTrack> & object_tracks,
  const LinearRing2d & local_footprint)
{
  if (debug_info.events.empty()) {
    return;
  }

  std::unordered_map<std::string, bool> engaged_objects;
  for (const auto & event : debug_info.events) {
    engaged_objects[event.object_id] = engaged_objects[event.object_id] || event.at_fault;
  }
  if (engaged_objects.empty()) {
    return;
  }

  const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);
  for (const auto & point : trajectory.points) {
    const auto query_time = trajectory_start_time + rclcpp::Duration(point.time_from_start);
    const auto query_time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto ego_polygon = create_pose_footprint(point.pose, local_footprint);

    bool ego_collision = false;
    bool ego_at_fault = false;
    for (const auto & object_track : object_tracks) {
      const auto object_id =
        object_id_to_string(object_track.object_id, object_track.has_valid_object_id);
      const auto object_it = engaged_objects.find(object_id);
      if (object_it == engaged_objects.end()) {
        continue;
      }

      const auto object_state = interpolate_logged_object_state(object_track, query_time);
      if (!object_state.has_value()) {
        continue;
      }

      const bool intersects = bg::intersects(ego_polygon, object_state->polygon);
      const bool object_at_fault = object_it->second;
      ego_collision = ego_collision || intersects;
      ego_at_fault = ego_at_fault || (intersects && object_at_fault);

      NoAtFaultCollisionHorizonFootprint object_footprint;
      object_footprint.time_s = query_time_s;
      object_footprint.object_id = object_id;
      object_footprint.object_label =
        autoware::object_recognition_utils::convertLabelToString(object_state->classification);
      object_footprint.collision = intersects;
      object_footprint.at_fault = intersects && object_at_fault;
      object_footprint.footprint =
        polygon_to_points(object_state->polygon, object_state->pose.position.z);
      debug_info.object_horizon_footprints.push_back(std::move(object_footprint));

      if (intersects) {
        const double overlap_z = 0.5 * (point.pose.position.z + object_state->pose.position.z);
        for (const auto & overlap_polygon :
             overlap_polygons_to_points(ego_polygon, object_state->polygon, overlap_z)) {
          NoAtFaultCollisionOverlapArea overlap_area;
          overlap_area.time_s = query_time_s;
          overlap_area.object_id = object_id;
          overlap_area.object_label =
            autoware::object_recognition_utils::convertLabelToString(object_state->classification);
          overlap_area.at_fault = object_at_fault;
          overlap_area.polygon = overlap_polygon;
          debug_info.overlap_areas.push_back(std::move(overlap_area));
        }
      }
    }

    NoAtFaultCollisionHorizonFootprint ego_footprint;
    ego_footprint.time_s = query_time_s;
    ego_footprint.object_id = "ego";
    ego_footprint.object_label = "EGO";
    ego_footprint.collision = ego_collision;
    ego_footprint.at_fault = ego_at_fault;
    ego_footprint.footprint = polygon_to_points(ego_polygon, point.pose.position.z);
    debug_info.ego_horizon_footprints.push_back(std::move(ego_footprint));
  }
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

      const auto collision = classify_collision(point, *object_state, vehicle_info);
      auto debug_event = make_debug_event(query_time_s, point, ego_polygon, *object_state, collision);

      const bool front_or_stopped_track = collision.type == CollisionType::ActiveFront ||
                                          collision.type == CollisionType::StoppedTrack;
      const bool lateral_collision = collision.type == CollisionType::ActiveLateral;

      if (front_or_stopped_track) {
        const auto at_fault_collision = make_at_fault_collision(*object_state, false);
        debug_event.at_fault = true;
        debug_event.event_score = at_fault_collision.score;
        debug_event.reason = at_fault_collision.reason;
        result.debug_info.events.push_back(debug_event);
        if (at_fault_collision.score < result.score) {
          result.score = at_fault_collision.score;
          result.reason = at_fault_collision.reason;
          result.infraction_time_s = query_time_s;
        }
        continue;
      }

      if (lateral_collision) {
        const auto ego_area_flags = compute_ego_area_flags(point.pose, ego_polygon, route_handler);
        if (!ego_area_flags.has_value()) {
          debug_event.reason = !route_handler
                                 ? "unavailable_no_route_handler_for_lateral_assessment"
                                 : "unavailable_route_handler_not_ready_for_lateral_assessment";
          result.debug_info.events.push_back(debug_event);
          result.available = false;
          result.score = 0.0;
          result.reason = !route_handler
                            ? "unavailable_no_route_handler_for_lateral_assessment"
                            : "unavailable_route_handler_not_ready_for_lateral_assessment";
          return result;
        }

        debug_event.multiple_lanes = ego_area_flags->multiple_lanes;
        debug_event.non_drivable_area = ego_area_flags->non_drivable_area;
        if (ego_area_flags->multiple_lanes || ego_area_flags->non_drivable_area) {
          const auto at_fault_collision = make_at_fault_collision(*object_state, true);
          debug_event.at_fault = true;
          debug_event.event_score = at_fault_collision.score;
          debug_event.reason = at_fault_collision.reason;
          if (at_fault_collision.score < result.score) {
            result.score = at_fault_collision.score;
            result.reason = at_fault_collision.reason;
            result.infraction_time_s = query_time_s;
          }
        }
        result.debug_info.events.push_back(debug_event);
        continue;
      }

      result.debug_info.events.push_back(debug_event);
    }
  }

  fill_horizon_debug_footprints(result.debug_info, trajectory, object_tracks, local_footprint);

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
