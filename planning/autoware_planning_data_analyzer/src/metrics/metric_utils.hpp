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

#ifndef METRICS__METRIC_UTILS_HPP_
#define METRICS__METRIC_UTILS_HPP_

#include "../data_types.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

struct LoggedObjectState
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  autoware_perception_msgs::msg::Shape shape;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
};

struct LoggedObjectTrack
{
  std::array<uint8_t, 16> object_id{};
  bool has_valid_object_id{false};
  std::vector<LoggedObjectState> states;
};

struct InterpolatedLoggedObject
{
  std::array<uint8_t, 16> object_id{};
  bool has_valid_object_id{false};
  geometry_msgs::msg::Pose pose;
  double speed_mps{0.0};
  autoware_perception_msgs::msg::Shape shape;
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
  autoware_utils_geometry::Polygon2d polygon;
};

struct EgoAreaFlags
{
  bool multiple_lanes{false};
  bool non_drivable_area{false};
};

struct EgoAreaEvaluation
{
  EgoAreaFlags flags;
  std::vector<autoware_utils_geometry::Point2d> footprint_points;
  std::vector<bool> corner_drivable;
  lanelet::ConstLanelets road_lanelets;
  std::vector<lanelet::ConstPolygon3d> parking_lots;
  std::size_t designated_lanelet_count{0};
};

struct TrajectoryFootprintEvaluation
{
  autoware_utils_geometry::Polygon2d ego_polygon;
  std::optional<EgoAreaEvaluation> ego_area_evaluation;
};

struct DrivingDirectionLocalContext
{
  bool in_route_lane_polygon{false};
  bool in_lane_margin_only{false};
  bool in_intersection{false};
  lanelet::ConstLanelets route_lanelets;
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
};

bool is_vehicle_info_valid(const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

double get_yaw(const geometry_msgs::msg::Quaternion & orientation);

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

autoware_utils_geometry::Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware_utils_geometry::LinearRing2d & local_footprint);

std::optional<lanelet::ConstLanelet> find_reference_lanelet(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

lanelet::ConstLanelets collect_route_relevant_lanelets(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::shared_ptr<RouteHandler> & route_handler);

std::optional<EgoAreaEvaluation> compute_ego_area_evaluation(
  const geometry_msgs::msg::Pose & pose, const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler,
  const lanelet::ConstLanelets & designated_lanelets = {});

std::vector<TrajectoryFootprintEvaluation> evaluate_trajectory_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler = nullptr);

autoware_utils_geometry::LineString2d to_linestring2d(const lanelet::ConstLineString3d & line);

bool is_pose_in_intersection(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

bool is_pose_in_route_lane_polygon(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

std::optional<DrivingDirectionLocalContext> compute_driving_direction_local_context(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler);

double forward_offset_in_ego_frame(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

bool is_agent_behind(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & object_pose);

bool has_valid_object_id(const unique_identifier_msgs::msg::UUID & object_id);

std::array<uint8_t, 16> object_id_key(const unique_identifier_msgs::msg::UUID & object_id);

bool is_agent_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);

bool is_unknown_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedTrackedObjects> & future_objects);

std::optional<InterpolatedLoggedObject> interpolate_logged_object_state(
  const LoggedObjectTrack & track, const rclcpp::Time & query_time);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__METRIC_UTILS_HPP_
