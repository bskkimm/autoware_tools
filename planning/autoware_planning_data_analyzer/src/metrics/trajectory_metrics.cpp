// Copyright 2025 TIER IV, Inc.
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

#include "trajectory_metrics.hpp"

#include "drivable_area_compliance.hpp"
#include "driving_direction_compliance.hpp"
#include "history_comfort.hpp"
#include "metric_utils.hpp"
#include "no_at_fault_collision.hpp"
#include "traffic_light_compliance.hpp"
#include "ttc_within_bound.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <boost/geometry.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

namespace
{

constexpr double kDDCAdmissibleLaneMarkerMarginM = 0.35;

template <typename MessageT, typename ActivePredicate>
std::vector<std::pair<double, double>> collect_signal_active_windows(
  const std::vector<std::shared_ptr<const MessageT>> & history, const rclcpp::Time & trajectory_start,
  const ActivePredicate & is_active, const double pre_grace_s, const double post_grace_s)
{
  std::vector<std::pair<double, double>> windows;
  if (history.empty()) {
    return windows;
  }

  bool has_active_start = false;
  double active_start_s = 0.0;
  bool previous_active = false;
  for (const auto & msg : history) {
    if (!msg) {
      continue;
    }
    const auto sample_time_s = (rclcpp::Time(msg->stamp) - trajectory_start).seconds();
    const bool active = is_active(*msg);
    if (active && !previous_active) {
      active_start_s = sample_time_s;
      has_active_start = true;
    } else if (!active && previous_active && has_active_start) {
      windows.emplace_back(active_start_s - pre_grace_s, sample_time_s + post_grace_s);
      has_active_start = false;
    }
    previous_active = active;
  }

  if (previous_active && has_active_start) {
    const auto last_time_s = (rclcpp::Time(history.back()->stamp) - trajectory_start).seconds();
    windows.emplace_back(active_start_s - pre_grace_s, last_time_s + post_grace_s);
  }

  return windows;
}

std::vector<std::pair<double, double>> merge_windows(
  std::vector<std::pair<double, double>> windows)
{
  if (windows.empty()) {
    return windows;
  }
  std::sort(
    windows.begin(), windows.end(),
    [](const auto & lhs, const auto & rhs) { return lhs.first < rhs.first; });
  std::vector<std::pair<double, double>> merged;
  merged.push_back(windows.front());
  for (std::size_t index = 1; index < windows.size(); ++index) {
    auto & current = merged.back();
    if (windows[index].first <= current.second) {
      current.second = std::max(current.second, windows[index].second);
    } else {
      merged.push_back(windows[index]);
    }
  }
  return merged;
}

geometry_msgs::msg::Point to_msg_point(
  const geometry_msgs::msg::Point & point, const double z_offset = 0.0)
{
  auto msg = point;
  msg.z += z_offset;
  return msg;
}

std::vector<geometry_msgs::msg::Point> lanelet_polygon_to_points(
  const lanelet::ConstLanelet & lanelet, const double z, const double margin_m = 0.0)
{
  namespace bg = boost::geometry;
  std::vector<geometry_msgs::msg::Point> points;
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : lanelet.polygon2d().basicPolygon()) {
    polygon.outer().push_back({point.x(), point.y()});
  }
  bg::correct(polygon);

  autoware_utils_geometry::Polygon2d visualization_polygon = polygon;
  if (margin_m > 0.0) {
    bg::model::multi_polygon<autoware_utils_geometry::Polygon2d> buffered;
    bg::strategy::buffer::distance_symmetric<double> distance_strategy(margin_m);
    bg::strategy::buffer::join_round join_strategy(16);
    bg::strategy::buffer::end_round end_strategy(16);
    bg::strategy::buffer::point_circle circle_strategy(16);
    bg::strategy::buffer::side_straight side_strategy;
    bg::buffer(
      polygon, buffered, distance_strategy, side_strategy, join_strategy, end_strategy,
      circle_strategy);
    if (!buffered.empty()) {
      visualization_polygon = buffered.front();
      bg::correct(visualization_polygon);
    }
  }

  for (const auto & point : visualization_polygon.outer()) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  if (!points.empty()) {
    points.push_back(points.front());
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> polygon_to_points(
  const lanelet::ConstPolygon3d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & point : lanelet::utils::to2D(polygon).basicPolygon()) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  if (!points.empty()) {
    points.push_back(points.front());
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> centerline_to_points(
  const lanelet::ConstLanelet & lanelet, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(lanelet.centerline3d().size());
  for (const auto & point : lanelet.centerline3d()) {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = z;
    points.push_back(msg);
  }
  return points;
}

/**
 * @brief Get velocity in world coordinate frame from trajectory point
 * Reference: autoware_trajectory_ranker/src/metrics/metrics_utils.cpp
 * @param point Trajectory point with velocities in vehicle frame
 * @return Velocity vector in world frame
 */
tf2::Vector3 get_velocity_in_world_coordinate(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const auto & pose = point.pose;
  const double yaw = get_yaw(pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Rotate velocity from vehicle frame to world frame
  const double vx_world =
    cos_yaw * point.longitudinal_velocity_mps - sin_yaw * point.lateral_velocity_mps;
  const double vy_world =
    sin_yaw * point.longitudinal_velocity_mps + cos_yaw * point.lateral_velocity_mps;

  return tf2::Vector3(vx_world, vy_world, 0.0);
}

/**
 * @brief Calculate time to collision between two trajectory points
 * Reference: autoware_trajectory_ranker/src/metrics/metrics_utils.cpp
 * @param point1 First trajectory point
 * @param point2 Second trajectory point
 * @return Time to collision [s], or infinity if no collision
 */
double calculate_ttc_between_points(
  const autoware_planning_msgs::msg::TrajectoryPoint & point1,
  const autoware_planning_msgs::msg::TrajectoryPoint & point2)
{
  constexpr double eps = 1e-6;

  // Calculate displacement vector
  const auto & pos1 = point1.pose.position;
  const auto & pos2 = point2.pose.position;
  const tf2::Vector3 displacement(pos2.x - pos1.x, pos2.y - pos1.y, 0.0);
  const double distance = displacement.length();

  if (distance < eps) {
    return 0.0;
  }

  const auto dir = displacement.normalized();

  // Get velocities in world frame
  const auto v1 = get_velocity_in_world_coordinate(point1);
  const auto v2 = get_velocity_in_world_coordinate(point2);

  // Calculate relative velocity along displacement direction
  const double relative_velocity = tf2::tf2Dot(dir, v1) - tf2::tf2Dot(dir, v2);

  if (std::abs(relative_velocity) < eps) {
    return std::numeric_limits<double>::max();
  }

  return distance / relative_velocity;
}

}  // namespace

TrajectoryPointMetrics calculate_trajectory_point_metrics(
  const std::shared_ptr<SynchronizedData> & sync_data,
  const autoware_planning_msgs::msg::Trajectory * ground_truth_trajectory,
  const std::shared_ptr<RouteHandler> & route_handler,
  const HistoryComfortParameters & history_comfort_params,
  const LaneKeepingParameters & lane_keeping_params,
  const DrivingDirectionComplianceParameters & driving_direction_params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const EnabledMetrics & enabled_metrics,
  const std::vector<TimedTrackedObjects> & future_objects)
{
  (void)ground_truth_trajectory;
  TrajectoryPointMetrics metrics;

  if (!sync_data || !sync_data->trajectory) {
    return metrics;
  }

  const auto & trajectory = *sync_data->trajectory;
  const auto & logged_future_objects =
    future_objects.empty() ? sync_data->future_objects : future_objects;
  const size_t num_points = trajectory.points.size();
  const auto shared_footprint_evaluations =
    (enabled_metrics.time_to_collision_within_bound || enabled_metrics.drivable_area_compliance ||
     enabled_metrics.no_at_fault_collision || enabled_metrics.traffic_light_compliance ||
     enabled_metrics.lane_keeping) &&
        is_vehicle_info_valid(vehicle_info)
      ? evaluate_trajectory_footprints(trajectory, vehicle_info, route_handler)
      : std::vector<TrajectoryFootprintEvaluation>{};

  // Initialize vectors
  metrics.ttc_values.resize(num_points, std::numeric_limits<double>::max());
  metrics.lateral_deviations.resize(num_points, 0.0);
  metrics.travel_distances.resize(num_points, 0.0);

  if (num_points == 0U) {
    return metrics;
  }

  if (enabled_metrics.history_comfort) {
    calculate_history_comfort_metrics(*sync_data, history_comfort_params, metrics);
  }

  if (enabled_metrics.time_to_collision_within_bound) {
    const auto ttc_within_bound = calculate_ttc_within_bound(
      trajectory, logged_future_objects, vehicle_info, route_handler,
      shared_footprint_evaluations.empty() ? nullptr : &shared_footprint_evaluations);
    metrics.time_to_collision_within_bound = ttc_within_bound.score;
    metrics.time_to_collision_within_bound_available = ttc_within_bound.available;
    metrics.time_to_collision_within_bound_reason = ttc_within_bound.reason;
    metrics.time_to_collision_infraction_time_s = ttc_within_bound.infraction_time_s;
    metrics.time_to_collision_within_bound_debug = ttc_within_bound.debug_info;
  } else {
    metrics.time_to_collision_within_bound_reason = "disabled";
  }

  if (enabled_metrics.no_at_fault_collision) {
    const auto no_at_fault_collision = calculate_no_at_fault_collision(
      trajectory, logged_future_objects, vehicle_info, route_handler,
      shared_footprint_evaluations.empty() ? nullptr : &shared_footprint_evaluations);
    metrics.no_at_fault_collision = no_at_fault_collision.score;
    metrics.no_at_fault_collision_available = no_at_fault_collision.available;
    metrics.no_at_fault_collision_reason = no_at_fault_collision.reason;
    metrics.time_to_at_fault_collision_s = no_at_fault_collision.infraction_time_s;
    metrics.no_at_fault_collision_debug = no_at_fault_collision.debug_info;
  } else {
    metrics.no_at_fault_collision_reason = "disabled";
  }

  if (!enabled_metrics.driving_direction_compliance) {
    metrics.driving_direction_compliance_reason = "disabled";
  } else if (!route_handler) {
    metrics.driving_direction_compliance_reason = "unavailable_no_route_handler";
  } else if (!route_handler->isHandlerReady()) {
    metrics.driving_direction_compliance_reason = "unavailable_route_handler_not_ready";
  } else {
    std::vector<DrivingDirectionEvaluationPoint> driving_direction_evaluation_points;
    std::vector<DrivingDirectionLocalContext> driving_direction_contexts;
    bool label_anchor_set = false;
    driving_direction_evaluation_points.reserve(num_points);
    driving_direction_contexts.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      double progress_m = 0.0;
      if (i > 0) {
        progress_m = autoware_utils_geometry::calc_distance2d(
          trajectory.points.at(i - 1).pose.position, trajectory.points.at(i).pose.position);
      }
      const auto & point = trajectory.points.at(i);
      const auto local_context =
        compute_driving_direction_local_context(point.pose, route_handler).value_or(
          DrivingDirectionLocalContext{});
      driving_direction_evaluation_points.push_back(DrivingDirectionEvaluationPoint{
        rclcpp::Duration(point.time_from_start).seconds(), progress_m,
        !local_context.in_route_lane_polygon, local_context.in_intersection});
      driving_direction_contexts.push_back(local_context);
    }

    const auto ddc_result = calculate_driving_direction_compliance(
      driving_direction_evaluation_points, driving_direction_params);
    metrics.driving_direction_compliance = ddc_result.score;
    metrics.driving_direction_compliance_available = ddc_result.available;
    metrics.driving_direction_compliance_reason = ddc_result.reason;
    metrics.max_oncoming_progress_m = ddc_result.max_oncoming_progress_m;
    metrics.driving_direction_compliance_debug.worst_window_start_time_s =
      ddc_result.worst_window_start_time_s;
    metrics.driving_direction_compliance_debug.worst_window_end_time_s =
      ddc_result.worst_window_end_time_s;
    metrics.driving_direction_compliance_debug.worst_window_sample_count =
      ddc_result.worst_window_sample_count;
    metrics.driving_direction_compliance_debug.window_progress_m =
      ddc_result.max_oncoming_progress_m;

    for (size_t i = 0; i < num_points; ++i) {
      const auto & point = trajectory.points.at(i);
      const auto & context = driving_direction_contexts.at(i);
      const auto time_s = rclcpp::Duration(point.time_from_start).seconds();
      const auto counted_progress_m =
        driving_direction_evaluation_points.at(i).in_oncoming_traffic &&
            !driving_direction_evaluation_points.at(i).is_intersection
          ? std::max(0.0, driving_direction_evaluation_points.at(i).progress_m)
          : 0.0;

      metrics.driving_direction_compliance_debug.samples.push_back(DrivingDirectionDebugSample{
        time_s,
        driving_direction_evaluation_points.at(i).progress_m,
        counted_progress_m,
        driving_direction_evaluation_points.at(i).in_oncoming_traffic,
        context.in_lane_margin_only,
        driving_direction_evaluation_points.at(i).is_intersection,
        to_msg_point(point.pose.position, 0.05)});

      const bool in_worst_window =
        time_s + 1.0e-6 >= ddc_result.worst_window_start_time_s &&
        time_s <= ddc_result.worst_window_end_time_s + 1.0e-6;
      if (!in_worst_window) {
        continue;
      }
      if (!label_anchor_set) {
        metrics.driving_direction_compliance_debug.label_anchor = to_msg_point(point.pose.position, 0.35);
        label_anchor_set = true;
      }
      for (const auto & lanelet : context.route_lanelets) {
        metrics.driving_direction_compliance_debug.route_lane_polygons.push_back(
          DrivingDirectionDebugPolygon{
            time_s,
            lanelet_polygon_to_points(
              lanelet, point.pose.position.z + 0.02, kDDCAdmissibleLaneMarkerMarginM)});
      }
      for (const auto & polygon : context.intersection_areas) {
        metrics.driving_direction_compliance_debug.intersection_lane_polygons.push_back(
          DrivingDirectionDebugPolygon{
            time_s, polygon_to_points(polygon, point.pose.position.z + 0.08)});
      }
    }
  }

  if (!enabled_metrics.drivable_area_compliance) {
    metrics.drivable_area_compliance_reason = "disabled";
  }
  if (!enabled_metrics.traffic_light_compliance) {
    metrics.traffic_light_compliance_reason = "disabled";
  }

  if (
    (enabled_metrics.drivable_area_compliance || enabled_metrics.traffic_light_compliance) &&
    !route_handler) {
    if (enabled_metrics.drivable_area_compliance) {
      metrics.drivable_area_compliance_reason = "unavailable_no_route_handler";
    }
    if (enabled_metrics.traffic_light_compliance) {
      metrics.traffic_light_compliance_reason = "unavailable_no_route_handler";
    }
  } else if (
    (enabled_metrics.drivable_area_compliance || enabled_metrics.traffic_light_compliance) &&
    !route_handler->isHandlerReady()) {
    if (enabled_metrics.drivable_area_compliance) {
      metrics.drivable_area_compliance_reason = "unavailable_route_handler_not_ready";
    }
    if (enabled_metrics.traffic_light_compliance) {
      metrics.traffic_light_compliance_reason = "unavailable_route_handler_not_ready";
    }
  } else if (enabled_metrics.drivable_area_compliance || enabled_metrics.traffic_light_compliance) {
    if (enabled_metrics.drivable_area_compliance) {
      const auto drivable_area_compliance =
        calculate_drivable_area_compliance(
          trajectory, route_handler, vehicle_info,
          shared_footprint_evaluations.empty() ? nullptr : &shared_footprint_evaluations);
      metrics.drivable_area_compliance = drivable_area_compliance.score;
      metrics.drivable_area_compliance_available = drivable_area_compliance.available;
      metrics.drivable_area_compliance_reason = drivable_area_compliance.reason;
      metrics.drivable_area_compliance_debug = drivable_area_compliance.debug_info;
    }

    if (enabled_metrics.traffic_light_compliance) {
      const auto traffic_light_compliance = calculate_traffic_light_compliance(
        trajectory, sync_data->traffic_signals, route_handler, vehicle_info,
        sync_data->turn_indicators_status,
        shared_footprint_evaluations.empty() ? nullptr : &shared_footprint_evaluations);
      metrics.traffic_light_compliance = traffic_light_compliance.score;
      metrics.traffic_light_compliance_available = traffic_light_compliance.available;
      metrics.traffic_light_compliance_reason = traffic_light_compliance.reason;
      metrics.traffic_light_compliance_debug = traffic_light_compliance.debug_info;
    }
  }

  // Calculate TTC for each point (based on autoware_trajectory_ranker implementation)
  constexpr double max_ttc_value = 10.0;  // Maximum TTC value in seconds
  const auto object_tracks =
    enabled_metrics.time_to_collision_within_bound ? build_logged_object_tracks(logged_future_objects)
                                                   : std::vector<LoggedObjectTrack>{};
  if (enabled_metrics.time_to_collision_within_bound && !object_tracks.empty()) {
    const auto trajectory_start_time = rclcpp::Time(trajectory.header.stamp);
    for (size_t i = 0; i < num_points; ++i) {
      double min_ttc = std::numeric_limits<double>::max();

      const auto & ego_point = trajectory.points[i];
      const auto query_time = trajectory_start_time + rclcpp::Duration(ego_point.time_from_start);

      // Check TTC with all objects
      for (const auto & object_track : object_tracks) {
        const auto object_state = interpolate_logged_object_state(object_track, query_time);
        if (!object_state.has_value()) {
          continue;
        }

        autoware_planning_msgs::msg::TrajectoryPoint object_point;
        object_point.pose = object_state->pose;
        object_point.longitudinal_velocity_mps = object_state->speed_mps;
        const double ttc =
          std::min(calculate_ttc_between_points(ego_point, object_point), max_ttc_value);
        if (std::isfinite(ttc) && ttc >= 0.0) {
          min_ttc = std::min(min_ttc, ttc);
        }
      }

      if (!std::isfinite(min_ttc)) {
        min_ttc = max_ttc_value;
      }
      metrics.ttc_values[i] = std::min(min_ttc, max_ttc_value);
    }
  }

  std::vector<LaneKeepingEvaluationPoint> lane_keeping_evaluation_points;
  lane_keeping_evaluation_points.reserve(num_points);
  std::vector<std::pair<double, double>> lane_change_windows_s;

  // Calculate travel distances once and reuse them in LK queue/creep logic.
  for (size_t i = 0; i < num_points; ++i) {
    metrics.travel_distances[i] =
      autoware::motion_utils::calcSignedArcLength(trajectory.points, 0, i);
  }

  // Calculate lateral deviation from the local route lane at each pose.
  if (!enabled_metrics.lane_keeping) {
    metrics.lane_keeping_reason = "disabled";
  } else if (!route_handler) {
    metrics.lane_keeping_reason = "unavailable_no_route_handler";
  } else if (!route_handler->isHandlerReady()) {
    metrics.lane_keeping_reason = "unavailable_route_handler_not_ready";
  } else {
    for (size_t i = 0; i < num_points; ++i) {
      const auto & point = trajectory.points[i];
      geometry_msgs::msg::Point ego_center = point.pose.position;
      const auto reference_lanelet = find_reference_lanelet(point.pose, route_handler);
      if (!reference_lanelet.has_value()) {
        metrics.lateral_deviations[i] = std::numeric_limits<double>::quiet_NaN();
        const bool multiple_lanes =
          i < shared_footprint_evaluations.size() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation.has_value() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation->flags.multiple_lanes;
        const bool non_drivable_area =
          i < shared_footprint_evaluations.size() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation.has_value() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation->flags.non_drivable_area;
        lane_keeping_evaluation_points.push_back(LaneKeepingEvaluationPoint{
          point.time_from_start,
          metrics.lateral_deviations[i],
          false,
          multiple_lanes,
          non_drivable_area,
          ego_center,
          {},
          -1,
          std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps),
          metrics.travel_distances[i]});
      } else {
        metrics.lateral_deviations[i] =
          lanelet::utils::getLateralDistanceToCenterline(reference_lanelet.value(), point.pose);
        const bool multiple_lanes =
          i < shared_footprint_evaluations.size() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation.has_value() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation->flags.multiple_lanes;
        const bool non_drivable_area =
          i < shared_footprint_evaluations.size() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation.has_value() &&
          shared_footprint_evaluations.at(i).ego_area_evaluation->flags.non_drivable_area;
        lane_keeping_evaluation_points.push_back(LaneKeepingEvaluationPoint{
          point.time_from_start,
          metrics.lateral_deviations[i],
          is_pose_in_intersection(point.pose, route_handler),
          multiple_lanes,
          non_drivable_area,
          ego_center,
          centerline_to_points(reference_lanelet.value(), point.pose.position.z),
          reference_lanelet->id(),
          std::hypot(point.longitudinal_velocity_mps, point.lateral_velocity_mps),
          metrics.travel_distances[i]});
      }
    }

    const auto trajectory_start_time = rclcpp::Time(sync_data->trajectory->header.stamp);
    auto turn_windows = collect_signal_active_windows<TurnIndicatorsReport>(
      sync_data->turn_indicators_history, trajectory_start_time,
      [](const auto & msg) {
        return msg.report == TurnIndicatorsReport::ENABLE_LEFT ||
               msg.report == TurnIndicatorsReport::ENABLE_RIGHT;
      },
      1.0, 1.0);
    auto hazard_windows = collect_signal_active_windows<HazardLightsReport>(
      sync_data->hazard_lights_history, trajectory_start_time,
      [](const auto & msg) { return msg.report == HazardLightsReport::ENABLE; }, 1.0, 1.0);
    turn_windows.insert(
      turn_windows.end(), std::make_move_iterator(hazard_windows.begin()),
      std::make_move_iterator(hazard_windows.end()));
    lane_change_windows_s = merge_windows(std::move(turn_windows));
  }
  if (enabled_metrics.lane_keeping) {
    const auto has_finite_lane_keeping_sample = std::any_of(
      lane_keeping_evaluation_points.begin(), lane_keeping_evaluation_points.end(),
      [](const auto & evaluation_point) {
        return std::isfinite(evaluation_point.lateral_deviation);
      });
    if (has_finite_lane_keeping_sample) {
      const auto lane_keeping_result =
        calculate_lane_keeping_result(
          lane_keeping_evaluation_points, lane_keeping_params, lane_change_windows_s);
      metrics.lane_keeping = lane_keeping_result.score;
      metrics.lane_keeping_available = true;
      metrics.lane_keeping_reason = "available";
      metrics.lane_keeping_debug = lane_keeping_result.debug;
    } else if (metrics.lane_keeping_reason == "unavailable") {
      metrics.lane_keeping_reason = "unavailable_no_reference_lanelet";
    }
  }

  return metrics;
}

}  // namespace autoware::planning_data_analyzer::metrics
