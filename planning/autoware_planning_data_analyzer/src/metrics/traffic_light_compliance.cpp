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

#include "traffic_light_compliance.hpp"

#include "metric_utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

struct ControlledTrafficLightGroup
{
  lanelet::Id regulatory_element_id{0};
  lanelet::ConstLanelets lanelets;
  std::optional<lanelet::ConstLineString3d> stop_line;
};

const autoware_perception_msgs::msg::TrafficLightGroup * find_signal_group(
  const TrafficLightGroupArray & traffic_signals, const lanelet::Id reg_elem_id)
{
  const auto it = std::find_if(
    traffic_signals.traffic_light_groups.begin(), traffic_signals.traffic_light_groups.end(),
    [reg_elem_id](const auto & group) {
      return static_cast<lanelet::Id>(group.traffic_light_group_id) == reg_elem_id;
    });
  return it == traffic_signals.traffic_light_groups.end() ? nullptr : &(*it);
}

autoware_utils_geometry::Polygon2d lanelet_polygon_2d(const lanelet::ConstLanelet & lanelet)
{
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : lanelet.polygon2d().basicPolygon()) {
    polygon.outer().push_back({point.x(), point.y()});
  }
  boost::geometry::correct(polygon);
  return polygon;
}

std::vector<geometry_msgs::msg::Point> polygon_to_msg_points(
  const autoware_utils_geometry::Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size() + 1U);
  for (const auto & point : polygon.outer()) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  if (!points.empty() && (points.front().x != points.back().x || points.front().y != points.back().y)) {
    points.push_back(points.front());
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> lanelet_to_msg_points(
  const lanelet::ConstLanelet & lanelet, const double z)
{
  return polygon_to_msg_points(lanelet_polygon_2d(lanelet), z);
}

std::vector<geometry_msgs::msg::Point> stop_line_to_msg_points(
  const lanelet::ConstLineString3d & stop_line, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(stop_line.size());
  for (const auto & point : stop_line) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  return points;
}

std::optional<autoware_utils_geometry::Polygon2d> intersection_polygon(
  const autoware_utils_geometry::Polygon2d & lhs, const autoware_utils_geometry::Polygon2d & rhs)
{
  namespace bg = boost::geometry;
  bg::model::multi_polygon<autoware_utils_geometry::Polygon2d> intersections;
  bg::intersection(lhs, rhs, intersections);
  if (intersections.empty()) {
    return std::nullopt;
  }

  auto polygon = intersections.front();
  bg::correct(polygon);
  if (polygon.outer().empty()) {
    return std::nullopt;
  }
  return polygon;
}

std::vector<ControlledTrafficLightGroup> build_controlled_traffic_light_groups(
  const lanelet::ConstLanelets & route_lanelets, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<ControlledTrafficLightGroup> groups;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return groups;
  }

  std::unordered_map<lanelet::Id, std::size_t> index_by_reg_elem_id;
  for (const auto & lanelet : route_lanelets) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    for (const auto & reg_elem :
         lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      const auto [it, inserted] =
        index_by_reg_elem_id.emplace(reg_elem->id(), index_by_reg_elem_id.size());
      if (inserted) {
        ControlledTrafficLightGroup group;
        group.regulatory_element_id = reg_elem->id();
        if (const auto stop_line = reg_elem->stopLine(); stop_line && !stop_line->empty()) {
          group.stop_line = *stop_line;
        }
        groups.push_back(group);
      }

      auto & group = groups.at(it->second);
      const auto duplicate = std::any_of(
        group.lanelets.begin(), group.lanelets.end(),
        [&lanelet](const auto & candidate) { return candidate.id() == lanelet.id(); });
      if (!duplicate) {
        group.lanelets.push_back(lanelet);
      }
      if (!group.stop_line.has_value()) {
        if (const auto stop_line = reg_elem->stopLine(); stop_line && !stop_line->empty()) {
          group.stop_line = *stop_line;
        }
      }
    }
  }

  return groups;
}

void record_first_failure_debug_info(
  TrafficLightComplianceDebugInfo & debug_info, const double time_s,
  const autoware_utils_geometry::Polygon2d & ego_polygon, const double z,
  const ControlledTrafficLightGroup & group, const lanelet::ConstLanelet & lanelet,
  const autoware_utils_geometry::Polygon2d & overlap_polygon)
{
  if (!std::isinf(debug_info.first_failure_time_s)) {
    return;
  }

  debug_info.first_failure_time_s = time_s;
  debug_info.label_anchor = polygon_to_msg_points(ego_polygon, z + 0.12).front();
  debug_info.regulatory_element_ids.push_back(group.regulatory_element_id);
  debug_info.controlled_lane_ids.push_back(lanelet.id());
  debug_info.active_red_polygon_count = 1U;
  debug_info.overlap_count = 1U;

  debug_info.red_controlled_lane_polygons.push_back(TrafficLightComplianceDebugPolygon{
    time_s, lanelet_to_msg_points(lanelet, z + 0.04), lanelet.id()});
  debug_info.overlap_areas.push_back(TrafficLightComplianceDebugPolygon{
    time_s, polygon_to_msg_points(overlap_polygon, z + 0.08), lanelet.id()});
  if (group.stop_line.has_value()) {
    debug_info.stop_lines.push_back(TrafficLightComplianceDebugPolygon{
      time_s, stop_line_to_msg_points(*group.stop_line, z + 0.10), group.stop_line->id()});
  }
}

void fill_debug_horizon_footprints(
  TrafficLightComplianceDebugInfo & debug_info,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TrajectoryFootprintEvaluation> & evaluations)
{
  if (evaluations.size() != trajectory.points.size()) {
    return;
  }

  debug_info.ego_horizon_footprints.reserve(trajectory.points.size());
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto time_s = rclcpp::Duration(trajectory.points.at(index).time_from_start).seconds();
    debug_info.ego_horizon_footprints.push_back(TrafficLightComplianceDebugPolygon{
      time_s,
      polygon_to_msg_points(evaluations.at(index).ego_polygon, trajectory.points.at(index).pose.position.z + 0.02),
      static_cast<lanelet::Id>(index)});
  }
}

}  // namespace

TrafficLightComplianceResult calculate_traffic_light_compliance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<TrafficLightGroupArray> & traffic_signals,
  const std::shared_ptr<RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::vector<TrajectoryFootprintEvaluation> * evaluations)
{
  TrafficLightComplianceResult result;

  if (trajectory.points.empty()) {
    result.reason = "unavailable_empty_trajectory";
    return result;
  }
  if (!route_handler) {
    result.reason = "unavailable_no_route_handler";
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

  const auto route_lanelets = collect_route_relevant_lanelets(trajectory, route_handler);
  const auto groups = build_controlled_traffic_light_groups(route_lanelets, route_handler);
  result.available = true;
  result.reason = groups.empty() ? "available_no_relevant_traffic_lights" : "available";
  result.score = 1.0;
  if (groups.empty()) {
    return result;
  }

  if (!traffic_signals) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_no_traffic_signals";
    return result;
  }

  std::unordered_set<lanelet::Id> missing_signal_ids;
  const auto local_footprint = vehicle_info.createFootprint(0.0);
  const bool can_reuse_evaluations =
    evaluations != nullptr && evaluations->size() == trajectory.points.size();
  if (can_reuse_evaluations) {
    fill_debug_horizon_footprints(result.debug_info, trajectory, *evaluations);
  }

  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points.at(point_index);
    const auto time_s = rclcpp::Duration(point.time_from_start).seconds();
    const auto ego_polygon =
      can_reuse_evaluations ? evaluations->at(point_index).ego_polygon
                            : create_pose_footprint(point.pose, local_footprint);
    if (!can_reuse_evaluations) {
      result.debug_info.ego_horizon_footprints.push_back(TrafficLightComplianceDebugPolygon{
        time_s, polygon_to_msg_points(ego_polygon, point.pose.position.z + 0.02),
        static_cast<lanelet::Id>(point_index)});
    }

    for (const auto & group : groups) {
      const auto * signal_group = find_signal_group(*traffic_signals, group.regulatory_element_id);
      if (!signal_group) {
        missing_signal_ids.insert(group.regulatory_element_id);
        continue;
      }

      std::size_t active_red_polygons = 0U;
      for (const auto & lanelet : group.lanelets) {
        if (!autoware::traffic_light_utils::isTrafficSignalStop(lanelet, *signal_group)) {
          continue;
        }

        ++active_red_polygons;
        const auto controlled_polygon = lanelet_polygon_2d(lanelet);
        if (!boost::geometry::intersects(ego_polygon, controlled_polygon)) {
          continue;
        }

        const auto overlap_polygon = intersection_polygon(ego_polygon, controlled_polygon);
        if (!overlap_polygon.has_value()) {
          continue;
        }

        record_first_failure_debug_info(
          result.debug_info, time_s, ego_polygon, point.pose.position.z, group, lanelet,
          *overlap_polygon);
        result.debug_info.active_red_polygon_count =
          std::max(result.debug_info.active_red_polygon_count, active_red_polygons);
        result.reason = "red_light_controlled_lane_entered";
        result.score = 0.0;
        return result;
      }
    }
  }

  if (!missing_signal_ids.empty()) {
    result.available = false;
    result.score = 0.0;
    result.reason = "unavailable_missing_signal_group";
    return result;
  }

  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
