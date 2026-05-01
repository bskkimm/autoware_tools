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

#include "drivable_area_compliance.hpp"

#include "metric_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <cmath>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back({point.x(), point.y()});
  }
  boost::geometry::correct(converted);
  return converted;
}

geometry_msgs::msg::Point to_msg_point(
  const autoware_utils_geometry::Point2d & point, const double z = 0.0)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = z;
  return msg;
}

std::vector<geometry_msgs::msg::Point> polygon_to_points(
  const autoware_utils_geometry::Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size());
  for (const auto & point : polygon.outer()) {
    points.push_back(to_msg_point(point, z));
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> lanelet_polygon_to_points(
  const lanelet::ConstLanelet & lanelet, const double z)
{
  return polygon_to_points(
    to_polygon_2d(lanelet.polygon2d().basicPolygon()), z);
}

std::vector<geometry_msgs::msg::Point> parking_polygon_to_points(
  const lanelet::ConstPolygon3d & polygon, const double z)
{
  return polygon_to_points(
    to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), z);
}

void fill_debug_info(
  DrivableAreaComplianceDebugInfo & debug_info,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TrajectoryFootprintEvaluation> & evaluations)
{
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points.at(index);
    const auto & evaluation = evaluations.at(index);
    if (!evaluation.ego_area_evaluation.has_value()) {
      continue;
    }

    const auto & area = *evaluation.ego_area_evaluation;
    const auto time_s = rclcpp::Duration(point.time_from_start).seconds();
    DrivableAreaComplianceHorizonFootprint footprint;
    footprint.time_s = time_s;
    footprint.non_drivable_area = area.flags.non_drivable_area;
    footprint.footprint = polygon_to_points(evaluation.ego_polygon, point.pose.position.z);
    debug_info.ego_horizon_footprints.push_back(std::move(footprint));

    if (!area.flags.non_drivable_area || std::isfinite(debug_info.first_failure_time_s)) {
      continue;
    }

    debug_info.first_failure_time_s = time_s;
    debug_info.route_candidate_count = area.designated_lanelet_count;
    debug_info.road_candidate_count = area.road_lanelets.size();
    debug_info.shoulder_candidate_count = area.shoulder_lanelets.size();
    debug_info.intersection_candidate_count = area.intersection_areas.size();
    debug_info.hatched_road_marking_candidate_count = area.hatched_road_markings.size();
    debug_info.parking_candidate_count = area.parking_lots.size();
    debug_info.corner_count_inside = std::count(
      area.corner_drivable.begin(), area.corner_drivable.end(), true);
    if (!area.footprint_points.empty()) {
      debug_info.label_anchor = to_msg_point(area.footprint_points.front(), point.pose.position.z);
    }

    for (std::size_t corner_index = 0; corner_index < area.footprint_points.size(); ++corner_index) {
      if (area.corner_drivable.at(corner_index)) {
        continue;
      }
      debug_info.failing_corner_indices.push_back(corner_index);
      debug_info.failing_corners.push_back(DrivableAreaComplianceDebugCorner{
        time_s, corner_index, to_msg_point(area.footprint_points.at(corner_index), point.pose.position.z)});
    }

    for (const auto & road_lanelet : area.road_lanelets) {
      debug_info.admissible_road_areas.push_back(DrivableAreaComplianceDebugPolygon{
        time_s, lanelet_polygon_to_points(road_lanelet, point.pose.position.z + 0.02)});
    }
    for (const auto & shoulder_lanelet : area.shoulder_lanelets) {
      debug_info.admissible_shoulder_areas.push_back(DrivableAreaComplianceDebugPolygon{
        time_s, lanelet_polygon_to_points(shoulder_lanelet, point.pose.position.z + 0.03)});
    }
    for (const auto & intersection_area : area.intersection_areas) {
      debug_info.admissible_intersection_areas.push_back(DrivableAreaComplianceDebugPolygon{
        time_s, parking_polygon_to_points(intersection_area, point.pose.position.z + 0.04)});
    }
    for (const auto & hatched_road_marking : area.hatched_road_markings) {
      debug_info.admissible_hatched_road_markings.push_back(DrivableAreaComplianceDebugPolygon{
        time_s, parking_polygon_to_points(hatched_road_marking, point.pose.position.z + 0.05)});
    }
    for (const auto & parking_lot : area.parking_lots) {
      debug_info.admissible_parking_areas.push_back(DrivableAreaComplianceDebugPolygon{
        time_s, parking_polygon_to_points(parking_lot, point.pose.position.z + 0.06)});
    }
  }
}

}  // namespace

DrivableAreaComplianceResult calculate_drivable_area_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::vector<TrajectoryFootprintEvaluation> * footprint_evaluations)
{
  DrivableAreaComplianceResult result;
  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (!route_handler) {
    result.reason = "unavailable_no_route_handler";
    return result;
  }
  if (!route_handler->isMapMsgReady()) {
    result.reason = "unavailable_route_handler_map_not_ready";
    return result;
  }
  if (!route_handler->isHandlerReady()) {
    result.reason = "unavailable_route_handler_not_ready";
    return result;
  }
  if (!is_vehicle_info_valid(vehicle_info)) {
    result.reason = "unavailable_invalid_vehicle_info";
    return result;
  }

  const auto local_evaluations =
    footprint_evaluations ? std::vector<TrajectoryFootprintEvaluation>{}
                          : evaluate_trajectory_footprints(trajectory, vehicle_info, route_handler);
  const auto & evaluations = footprint_evaluations ? *footprint_evaluations : local_evaluations;
  if (evaluations.size() != trajectory.points.size()) {
    result.reason = "unavailable_invalid_footprint";
    return result;
  }

  fill_debug_info(result.debug_info, trajectory, evaluations);
  result.available = true;
  result.reason = "compliant";

  for (const auto & evaluation : evaluations) {
    if (!evaluation.ego_area_evaluation.has_value()) {
      result.available = false;
      result.reason = "unavailable_drivable_area_query_failed";
      return result;
    }
    if (evaluation.ego_area_evaluation->flags.non_drivable_area) {
      result.reason = "non_compliant_corner_outside_drivable_area";
      return result;
    }
  }

  result.score = 1.0;
  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
