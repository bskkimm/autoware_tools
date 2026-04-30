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
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/utility/Utilities.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

constexpr double kLocalLaneSearchRadiusM = 5.0;
constexpr double kDirectionSimilarityThresholdRad = M_PI_4;
constexpr double kAdmissibleLaneMarginM = 0.35;

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
  }
}

void append_unique_polygon(
  const lanelet::ConstPolygon3d & polygon, std::vector<lanelet::ConstPolygon3d> & polygons,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(polygon.id()).second) {
    polygons.push_back(polygon);
  }
}

double closest_pi_symmetric_yaw(const double reference_yaw, const double yaw)
{
  double best_yaw = yaw;
  double best_error = std::abs(yaw - reference_yaw);
  for (int multiplier = -2; multiplier <= 2; ++multiplier) {
    const double candidate_yaw = yaw + static_cast<double>(multiplier) * M_PI;
    const double candidate_error = std::abs(candidate_yaw - reference_yaw);
    if (candidate_error < best_error) {
      best_yaw = candidate_yaw;
      best_error = candidate_error;
    }
  }
  return best_yaw;
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  namespace bg = boost::geometry;

  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back({point.x(), point.y()});
  }
  bg::correct(converted);
  return converted;
}

bool footprint_intersects_lanelet(
  const autoware_utils_geometry::Polygon2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return !bg::disjoint(footprint, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

std::vector<autoware_utils_geometry::Point2d> footprint_vertices(
  const autoware_utils_geometry::Polygon2d & footprint)
{
  namespace bg = boost::geometry;

  std::vector<autoware_utils_geometry::Point2d> vertices;
  for (const auto & point : footprint.outer()) {
    if (vertices.empty() || !bg::equals(vertices.front(), point)) {
      vertices.push_back(point);
    }
  }
  return vertices;
}

lanelet::BoundingBox2d footprint_bounding_box(const autoware_utils_geometry::Polygon2d & footprint)
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

lanelet::BoundingBox2d point_bounding_box(
  const geometry_msgs::msg::Point & point, const double radius_m = kLocalLaneSearchRadiusM)
{
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{point.x - radius_m, point.y - radius_m},
    lanelet::BasicPoint2d{point.x + radius_m, point.y + radius_m}};
}

lanelet::ConstLanelets collect_candidate_road_lanelets(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & designated_lanelets)
{
  lanelet::ConstLanelets road_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & lanelet : designated_lanelets) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    if (seen_ids.insert(lanelet.id()).second) {
      road_lanelets.push_back(lanelet);
    }
  }

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
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
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

bool point_in_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

bool point_in_parking_lot(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & parking_lot)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet::utils::to2D(parking_lot).basicPolygon()));
}

bool point_in_polygon(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & polygon)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
}

bool point_within_lanelet_margin(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet,
  const double margin_m)
{
  namespace bg = boost::geometry;
  const auto polygon = to_polygon_2d(lanelet.polygon2d().basicPolygon());
  return bg::distance(point, polygon) <= margin_m;
}

lanelet::ConstLanelets collect_local_route_consistent_lanelets(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets local_lanelets;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return local_lanelets;
  }

  const auto map = route_handler->getLaneletMapPtr();
  const auto nearby_bbox = point_bounding_box(pose.position);
  std::unordered_set<lanelet::Id> nearby_road_lanelet_ids;
  std::unordered_set<lanelet::Id> nearby_shoulder_lanelet_ids;
  lanelet::ConstLanelets nearby_road_lanelets;
  lanelet::ConstLanelets nearby_shoulder_lanelets;
  for (const auto & lanelet : map->laneletLayer.search(nearby_bbox)) {
    if (route_handler->isRoadLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_road_lanelets, nearby_road_lanelet_ids);
    } else if (route_handler->isShoulderLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_shoulder_lanelets, nearby_shoulder_lanelet_ids);
    }
  }

  lanelet::ConstLanelets seed_lanelets;
  std::unordered_set<lanelet::Id> seed_ids;
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      append_unique_lanelet(lanelet, seed_lanelets, seed_ids);
    }
  }
  lanelet::ConstLanelet closest_route_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_route_lanelet)) {
    append_unique_lanelet(closest_route_lanelet, seed_lanelets, seed_ids);
  }

  if (seed_lanelets.empty()) {
    return local_lanelets;
  }

  const double reference_yaw = lanelet::utils::getLaneletAngle(seed_lanelets.front(), pose.position);
  std::unordered_set<lanelet::Id> local_lanelet_ids;
  for (const auto & lanelet : nearby_road_lanelets) {
    const bool on_route = route_handler->isRouteLanelet(lanelet);
    const double lanelet_yaw = lanelet::utils::getLaneletAngle(lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(lanelet_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!on_route && !same_direction) {
      continue;
    }
    append_unique_lanelet(lanelet, local_lanelets, local_lanelet_ids);
  }

  for (const auto & shoulder_lanelet : nearby_shoulder_lanelets) {
    const double shoulder_yaw = lanelet::utils::getLaneletAngle(shoulder_lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(shoulder_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!same_direction) {
      continue;
    }
    append_unique_lanelet(shoulder_lanelet, local_lanelets, local_lanelet_ids);
  }

  return local_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_local_intersection_areas(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return intersection_areas;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & polygon : map->polygonLayer.search(point_bounding_box(pose.position))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "intersection_area") {
      continue;
    }
    append_unique_polygon(polygon, intersection_areas, seen_ids);
  }

  return intersection_areas;
}

bool detect_multiple_lanes(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets)
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

std::vector<bool> evaluate_corner_drivable(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  std::vector<bool> corner_drivable(footprint_points.size(), false);

  for (std::size_t index = 0; index < footprint_points.size(); ++index) {
    const auto & point = footprint_points.at(index);
    for (const auto & lanelet : road_lanelets) {
      if (point_in_lanelet(point, lanelet)) {
        corner_drivable.at(index) = true;
        break;
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & parking_lot : parking_lots) {
        if (point_in_parking_lot(point, parking_lot)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
  }

  return corner_drivable;
}

double planar_speed_mps(const geometry_msgs::msg::Twist & twist)
{
  return std::hypot(twist.linear.x, twist.linear.y);
}

double planar_displacement_m(
  const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  return autoware_utils_geometry::calc_distance2d(lhs.position, rhs.position);
}

bool should_hold_long_object_yaw(
  const LoggedObjectState & previous_state, const LoggedObjectState & current_state,
  const double previous_yaw, const double current_yaw)
{
  constexpr double kLongObjectLengthThresholdM = 8.0;
  constexpr double kSlowObjectSpeedThresholdMps = 2.0;
  constexpr double kSmallDisplacementThresholdM = 0.75;
  constexpr double kLargeYawJumpThresholdRad = 10.0 * M_PI / 180.0;

  const double object_length = std::max(previous_state.shape.dimensions.x, current_state.shape.dimensions.x);
  if (object_length < kLongObjectLengthThresholdM) {
    return false;
  }

  const double max_speed =
    std::max(planar_speed_mps(previous_state.twist), planar_speed_mps(current_state.twist));
  if (max_speed > kSlowObjectSpeedThresholdMps) {
    return false;
  }

  const double displacement = planar_displacement_m(previous_state.pose, current_state.pose);
  if (displacement > kSmallDisplacementThresholdM) {
    return false;
  }

  const double yaw_delta = std::abs(normalize_angle(current_yaw - previous_yaw));
  return yaw_delta > kLargeYawJumpThresholdRad;
}

void canonicalize_bounding_box_yaws(LoggedObjectTrack & track)
{
  if (track.states.size() < 2U) {
    return;
  }

  bool has_reference_yaw = false;
  double reference_yaw = 0.0;
  const LoggedObjectState * previous_state = nullptr;
  for (auto & state : track.states) {
    if (state.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      has_reference_yaw = false;
      previous_state = nullptr;
      continue;
    }

    const double raw_yaw = get_yaw(state.pose.orientation);
    double canonical_yaw =
      has_reference_yaw ? closest_pi_symmetric_yaw(reference_yaw, raw_yaw) : raw_yaw;
    if (
      has_reference_yaw && previous_state &&
      should_hold_long_object_yaw(*previous_state, state, reference_yaw, canonical_yaw)) {
      canonical_yaw = reference_yaw;
    }
    state.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(canonical_yaw);
    reference_yaw = canonical_yaw;
    has_reference_yaw = true;
    previous_state = &state;
  }
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
  return create_pose_footprint(pose, vehicle_info.createFootprint(0.0));
}

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint)
{
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

std::optional<EgoAreaEvaluation> compute_ego_area_evaluation(
  const geometry_msgs::msg::Pose & pose, const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & designated_lanelets)
{
  if (!route_handler) {
    return std::nullopt;
  }
  if (!route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  auto road_lanelets = collect_candidate_road_lanelets(ego_polygon, route_handler, designated_lanelets);
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
  const auto corner_drivable = evaluate_corner_drivable(points, road_lanelets, parking_lots);

  EgoAreaEvaluation evaluation;
  evaluation.flags.multiple_lanes = detect_multiple_lanes(points, road_lanelets);
  evaluation.flags.non_drivable_area = std::any_of(
    corner_drivable.begin(), corner_drivable.end(), [](const bool value) { return !value; });
  evaluation.footprint_points = points;
  evaluation.corner_drivable = corner_drivable;
  evaluation.road_lanelets = std::move(road_lanelets);
  evaluation.parking_lots = std::move(parking_lots);
  evaluation.designated_lanelet_count = designated_lanelets.size();
  return evaluation;
}

std::vector<TrajectoryFootprintEvaluation> evaluate_trajectory_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<TrajectoryFootprintEvaluation> evaluations;
  if (!is_vehicle_info_valid(vehicle_info)) {
    return evaluations;
  }

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  if (local_footprint.empty()) {
    return evaluations;
  }

  const auto designated_lanelets =
    route_handler && route_handler->isHandlerReady()
      ? collect_route_relevant_lanelets(trajectory, route_handler)
      : lanelet::ConstLanelets{};

  evaluations.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    TrajectoryFootprintEvaluation evaluation;
    evaluation.ego_polygon = create_pose_footprint(point.pose, local_footprint);
    if (route_handler) {
      evaluation.ego_area_evaluation = compute_ego_area_evaluation(
        point.pose, evaluation.ego_polygon, route_handler, designated_lanelets);
    }
    evaluations.push_back(std::move(evaluation));
  }

  return evaluations;
}

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line)
{
  autoware_utils_geometry::LineString2d line_2d;
  for (const auto & point : lanelet::utils::to2D(line)) {
    line_2d.push_back({point.x(), point.y()});
  }
  return line_2d;
}

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_intersection;
}

bool is_pose_in_route_lane_polygon(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_route_lane_polygon;
}

std::optional<DrivingDirectionLocalContext> compute_driving_direction_local_context(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  const autoware_utils_geometry::Point2d search_point{pose.position.x, pose.position.y};
  DrivingDirectionLocalContext context;
  context.route_lanelets = collect_local_route_consistent_lanelets(pose, route_handler);
  context.intersection_areas = collect_local_intersection_areas(pose, route_handler);

  const bool in_route_lane_polygon_exact = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(),
    [&search_point](const auto & lanelet) { return point_in_lanelet(search_point, lanelet); });
  const bool in_route_lane_polygon_margin = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(), [&search_point](const auto & lanelet) {
      return point_within_lanelet_margin(search_point, lanelet, kAdmissibleLaneMarginM);
    });
  context.in_route_lane_polygon = in_route_lane_polygon_exact || in_route_lane_polygon_margin;
  context.in_lane_margin_only = !in_route_lane_polygon_exact && in_route_lane_polygon_margin;
  context.in_intersection = std::any_of(
    context.intersection_areas.begin(), context.intersection_areas.end(),
    [&search_point](const auto & polygon) { return point_in_polygon(search_point, polygon); });

  if (!context.in_intersection) {
    for (const auto & lanelet : context.route_lanelets) {
      if (
        autoware::experimental::lanelet2_utils::is_intersection_lanelet(lanelet) &&
        point_in_lanelet(search_point, lanelet)) {
        context.in_intersection = true;
        break;
      }
    }
  }
  return context;
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
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kBehindAngleThresholdRad = 5.0 * kPi / 6.0;

  const double yaw = get_yaw(ego_pose.orientation);
  const double dx = object_pose.position.x - ego_pose.position.x;
  const double dy = object_pose.position.y - ego_pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1.0e-6) {
    return false;
  }

  const double cos_angle =
    std::clamp((std::cos(yaw) * dx + std::sin(yaw) * dy) / distance, -1.0, 1.0);
  return std::acos(cos_angle) > kBehindAngleThresholdRad;
}

bool has_valid_object_id(const unique_identifier_msgs::msg::UUID & object_id)
{
  return std::any_of(
    object_id.uuid.begin(), object_id.uuid.end(), [](const auto byte) { return byte != 0U; });
}

std::array<uint8_t, 16> object_id_key(const unique_identifier_msgs::msg::UUID & object_id)
{
  return object_id.uuid;
}

bool is_agent_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  const auto label = autoware::object_recognition_utils::getHighestProbLabel(classification);
  return autoware::object_recognition_utils::isVehicle(label) ||
         label == ObjectClassification::PEDESTRIAN || label == ObjectClassification::ANIMAL;
}

bool is_unknown_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  return autoware::object_recognition_utils::getHighestProbLabel(classification) ==
         ObjectClassification::UNKNOWN;
}

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedTrackedObjects> & future_objects)
{
  std::map<std::array<uint8_t, 16>, LoggedObjectTrack> keyed_tracks;
  std::vector<LoggedObjectTrack> invalid_id_tracks;

  for (const auto & timed_objects : future_objects) {
    if (!timed_objects.objects) {
      continue;
    }
    for (const auto & object : timed_objects.objects->objects) {
      LoggedObjectState state;
      state.stamp = timed_objects.stamp;
      state.pose = object.kinematics.pose_with_covariance.pose;
      state.twist = object.kinematics.twist_with_covariance.twist;
      state.shape = object.shape;
      state.classification = object.classification;

      const bool valid_id = has_valid_object_id(object.object_id);
      if (!valid_id) {
        LoggedObjectTrack track;
        track.has_valid_object_id = false;
        track.states.push_back(state);
        invalid_id_tracks.push_back(track);
        continue;
      }

      const auto key = object_id_key(object.object_id);
      auto & track = keyed_tracks[key];
      track.object_id = key;
      track.has_valid_object_id = true;
      track.states.push_back(state);
    }
  }

  std::vector<LoggedObjectTrack> tracks;
  tracks.reserve(keyed_tracks.size() + invalid_id_tracks.size());
  for (auto & [_, track] : keyed_tracks) {
    std::sort(track.states.begin(), track.states.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.stamp.nanoseconds() < rhs.stamp.nanoseconds();
    });
    canonicalize_bounding_box_yaws(track);
    tracks.push_back(std::move(track));
  }
  for (auto & track : invalid_id_tracks) {
    tracks.push_back(std::move(track));
  }
  return tracks;
}

std::optional<InterpolatedLoggedObject> interpolate_logged_object_state(
  const LoggedObjectTrack & track, const rclcpp::Time & query_time)
{
  if (track.states.empty()) {
    return std::nullopt;
  }

  auto make_object = [&](const LoggedObjectState & state, const double speed_mps) {
    InterpolatedLoggedObject object;
    object.object_id = track.object_id;
    object.has_valid_object_id = track.has_valid_object_id;
    object.pose = state.pose;
    object.speed_mps = speed_mps;
    object.shape = state.shape;
    object.classification = state.classification;
    object.polygon = autoware_utils_geometry::to_polygon2d(object.pose, object.shape);
    return object;
  };

  if (track.states.size() == 1U) {
    const auto & state = track.states.front();
    constexpr int64_t kSingleStateToleranceNs = 50'000'000;
    if (
      std::llabs(query_time.nanoseconds() - state.stamp.nanoseconds()) > kSingleStateToleranceNs) {
      return std::nullopt;
    }
    return make_object(state, std::hypot(state.twist.linear.x, state.twist.linear.y));
  }

  if (
    query_time.nanoseconds() < track.states.front().stamp.nanoseconds() ||
    query_time.nanoseconds() > track.states.back().stamp.nanoseconds()) {
    return std::nullopt;
  }

  auto after = std::lower_bound(
    track.states.begin(), track.states.end(), query_time,
    [](const auto & state, const auto & time) {
      return state.stamp.nanoseconds() < time.nanoseconds();
    });
  if (after == track.states.begin()) {
    return make_object(*after, std::hypot(after->twist.linear.x, after->twist.linear.y));
  }
  if (after == track.states.end()) {
    const auto & state = track.states.back();
    return make_object(state, std::hypot(state.twist.linear.x, state.twist.linear.y));
  }

  const auto & previous = *(after - 1);
  const auto & next = *after;
  const double dt = (next.stamp - previous.stamp).seconds();
  if (dt <= 0.0) {
    return make_object(next, std::hypot(next.twist.linear.x, next.twist.linear.y));
  }

  const double ratio = std::clamp((query_time - previous.stamp).seconds() / dt, 0.0, 1.0);
  InterpolatedLoggedObject object;
  object.object_id = track.object_id;
  object.has_valid_object_id = track.has_valid_object_id;
  object.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio, false);
  const double logged_speed = std::hypot(previous.twist.linear.x, previous.twist.linear.y);
  if (std::isfinite(logged_speed) && logged_speed > 1.0e-6) {
    object.speed_mps = logged_speed;
  } else {
    object.speed_mps =
      autoware_utils_geometry::calc_distance2d(previous.pose.position, next.pose.position) / dt;
  }
  object.shape = previous.shape;
  object.classification = previous.classification;
  object.polygon = autoware_utils_geometry::to_polygon2d(object.pose, object.shape);
  return object;
}

}  // namespace autoware::planning_data_analyzer::metrics
