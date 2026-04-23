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

#include "../../src/metrics/metric_utils.hpp"
#include "../../src/metrics/no_at_fault_collision.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{
namespace
{

autoware::vehicle_info_utils::VehicleInfo make_vehicle_info()
{
  autoware::vehicle_info_utils::VehicleInfo info;
  info.wheel_base_m = 2.74;
  info.wheel_tread_m = 1.63;
  info.front_overhang_m = 1.00;
  info.rear_overhang_m = 1.03;
  info.left_overhang_m = 0.10;
  info.right_overhang_m = 0.10;
  info.vehicle_height_m = 2.5;
  info.vehicle_length_m = 4.77;
  info.vehicle_width_m = 1.83;
  info.min_longitudinal_offset_m = -1.03;
  info.max_longitudinal_offset_m = 3.74;
  info.min_lateral_offset_m = -0.915;
  info.max_lateral_offset_m = 0.915;
  return info;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

autoware_planning_msgs::msg::Trajectory make_straight_trajectory(const double speed_mps)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points[0].pose = make_pose(0.0, 0.0);
  trajectory.points[0].longitudinal_velocity_mps = speed_mps;
  trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  trajectory.points[1].pose = make_pose(speed_mps, 0.0);
  trajectory.points[1].longitudinal_velocity_mps = speed_mps;
  trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);
  return trajectory;
}

autoware_planning_msgs::msg::Trajectory make_single_point_trajectory(
  const double x, const double y, const double speed_mps = 5.0)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.points.resize(1);
  trajectory.points[0].pose = make_pose(x, y);
  trajectory.points[0].longitudinal_velocity_mps = speed_mps;
  trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  return trajectory;
}

autoware_perception_msgs::msg::PredictedObject make_object(
  const double x, const double y, const std::uint8_t label, const double speed_mps = 0.0)
{
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose = make_pose(x, y);
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed_mps;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.5;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0f;
  object.classification.push_back(classification);

  autoware_perception_msgs::msg::PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(0.5);
  path.confidence = 1.0;
  path.path.push_back(make_pose(x, y));
  path.path.push_back(make_pose(x + 0.5 * speed_mps, y));
  path.path.push_back(make_pose(x + speed_mps, y));
  object.kinematics.predicted_paths.push_back(path);
  return object;
}

lanelet::Lanelet make_road_lanelet(const lanelet::Id id, const double y_min, const double y_max)
{
  lanelet::LineString3d left_bound{
    id * 10 + 1,
    {lanelet::Point3d{id * 100 + 1, -10.0, y_max, 0.0},
     lanelet::Point3d{id * 100 + 2, 20.0, y_max, 0.0}}};
  lanelet::LineString3d right_bound{
    id * 10 + 2,
    {lanelet::Point3d{id * 100 + 3, -10.0, y_min, 0.0},
     lanelet::Point3d{id * 100 + 4, 20.0, y_min, 0.0}}};
  lanelet::Lanelet lanelet{id, left_bound, right_bound};
  lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
  return lanelet;
}

std::shared_ptr<RouteHandler> make_route_handler(const lanelet::Lanelets & lanelets)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : lanelets) {
    map->add(lanelet);
  }

  autoware_map_msgs::msg::LaneletMapBin map_msg;
  lanelet::utils::conversion::toBinMsg(map, &map_msg);

  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_msg);
  return route_handler;
}

void set_object_id(autoware_perception_msgs::msg::PredictedObject & object, const uint8_t value)
{
  object.object_id.uuid.fill(0U);
  object.object_id.uuid.at(15) = value;
}

builtin_interfaces::msg::Time make_stamp(const double stamp_s)
{
  const auto stamp_ns = static_cast<int64_t>(std::llround(stamp_s * 1.0e9));
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000);
  return stamp;
}

std::vector<TimedPredictedObjects> make_future_objects(
  std::vector<autoware_perception_msgs::msg::PredictedObject> objects, const double stamp_s = 0.0)
{
  auto msg = std::make_shared<PredictedObjects>();
  msg->header.stamp = make_stamp(stamp_s);
  msg->objects = std::move(objects);
  return {TimedPredictedObjects{rclcpp::Time(msg->header.stamp), msg}};
}

std::vector<TimedPredictedObjects> append_future_objects(
  std::vector<TimedPredictedObjects> future_objects,
  std::vector<autoware_perception_msgs::msg::PredictedObject> objects, const double stamp_s)
{
  auto msg = std::make_shared<PredictedObjects>();
  msg->header.stamp = make_stamp(stamp_s);
  msg->objects = std::move(objects);
  future_objects.push_back(TimedPredictedObjects{rclcpp::Time(msg->header.stamp), msg});
  return future_objects;
}

}  // namespace

TEST(NoAtFaultCollision, BehindPredicateUsesNuPlanStyleAngleThreshold)
{
  const auto ego_pose = make_pose(0.0, 0.0);
  constexpr double kPi = 3.14159265358979323846;

  EXPECT_FALSE(is_agent_behind(ego_pose, make_pose(-0.5, std::sqrt(3.0) / 2.0)));
  EXPECT_TRUE(is_agent_behind(
    ego_pose, make_pose(std::cos(170.0 * kPi / 180.0), std::sin(170.0 * kPi / 180.0))));
}

TEST(NoAtFaultCollision, LoggedBoundingBoxYawFlipDoesNotInterpolateThroughSidewaysPose)
{
  constexpr double kPi = 3.14159265358979323846;

  auto first = make_object(5.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR);
  first.shape.dimensions.x = 4.0;
  first.shape.dimensions.y = 2.0;
  first.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  set_object_id(first, 10U);

  auto second = first;
  second.kinematics.initial_pose_with_covariance.pose.position.x = 5.1;
  second.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(kPi);

  auto future_objects = make_future_objects({first}, 0.0);
  future_objects = append_future_objects(std::move(future_objects), {second}, 1.0);
  const auto tracks = build_logged_object_tracks(future_objects);

  ASSERT_EQ(tracks.size(), 1U);
  const auto object_state =
    interpolate_logged_object_state(tracks.front(), rclcpp::Time(make_stamp(0.5)));
  ASSERT_TRUE(object_state.has_value());
  EXPECT_NEAR(get_yaw(object_state->pose.orientation), 0.0, 1.0e-3);

  const auto & outer = object_state->polygon.outer();
  ASSERT_GE(outer.size(), 4U);
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : outer) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }
  EXPECT_GT(max_x - min_x, max_y - min_y);
}

TEST(NoAtFaultCollision, EmptyObjectsPasses)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, FrontCollisionWithAgentFails)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  EXPECT_GE(result.infraction_time_s, 0.0);
  ASSERT_EQ(result.debug_info.events.size(), 1U);
  const auto & event = result.debug_info.events.front();
  EXPECT_DOUBLE_EQ(event.time_s, 0.0);
  EXPECT_EQ(event.object_label, "CAR");
  EXPECT_EQ(event.collision_type, "ACTIVE_FRONT");
  EXPECT_TRUE(event.agent);
  EXPECT_TRUE(event.at_fault);
  EXPECT_TRUE(event.front_hit);
  EXPECT_FALSE(event.behind);
  EXPECT_DOUBLE_EQ(event.event_score, 0.0);
  EXPECT_FALSE(event.ego_footprint.empty());
  EXPECT_FALSE(event.object_footprint.empty());
  EXPECT_EQ(event.front_bumper.size(), 2U);
  EXPECT_EQ(result.debug_info.ego_horizon_footprints.size(), trajectory.points.size());
  EXPECT_FALSE(result.debug_info.object_horizon_footprints.empty());
  EXPECT_FALSE(result.debug_info.overlap_areas.empty());
  EXPECT_TRUE(std::any_of(
    result.debug_info.ego_horizon_footprints.begin(),
    result.debug_info.ego_horizon_footprints.end(),
    [](const auto & footprint) { return footprint.collision && footprint.at_fault; }));
}

TEST(NoAtFaultCollision, DebugFootprintsPreserveMapZ)
{
  auto trajectory = make_single_point_trajectory(0.0, 0.0);
  trajectory.points.front().pose.position.z = 12.3;

  auto object =
    make_object(3.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 0.0);
  object.kinematics.initial_pose_with_covariance.pose.position.z = 40.5;
  for (auto & path : object.kinematics.predicted_paths) {
    for (auto & pose : path.path) {
      pose.position.z = 40.5;
    }
  }

  const auto result =
    calculate_no_at_fault_collision(trajectory, make_future_objects({object}), make_vehicle_info());

  ASSERT_TRUE(result.available);
  ASSERT_FALSE(result.debug_info.events.empty());
  ASSERT_FALSE(result.debug_info.ego_horizon_footprints.empty());
  ASSERT_FALSE(result.debug_info.object_horizon_footprints.empty());
  ASSERT_FALSE(result.debug_info.overlap_areas.empty());

  for (const auto & point : result.debug_info.events.front().ego_footprint) {
    EXPECT_DOUBLE_EQ(point.z, 12.3);
  }
  for (const auto & point : result.debug_info.events.front().object_footprint) {
    EXPECT_DOUBLE_EQ(point.z, 40.5);
  }
  for (const auto & point : result.debug_info.ego_horizon_footprints.front().footprint) {
    EXPECT_DOUBLE_EQ(point.z, 12.3);
  }
  for (const auto & point : result.debug_info.object_horizon_footprints.front().footprint) {
    EXPECT_DOUBLE_EQ(point.z, 40.5);
  }
  for (const auto & point : result.debug_info.overlap_areas.front().polygon) {
    EXPECT_DOUBLE_EQ(point.z, 0.5 * (12.3 + 40.5));
  }
}

TEST(NoAtFaultCollision, FrontCollisionWithNonAgentGetsHalfPenalty)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::HAZARD));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.5);
  EXPECT_EQ(result.reason, "at_fault_collision_with_non_agent");
}

TEST(NoAtFaultCollision, RearCollisionDoesNotFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(-3.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, MovingRearAgentCollisionDoesNotFail)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(-1.5, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, SlowRearAgentCollisionIsStoppedTrackAndFails)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(-1.5, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 0.04));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
}

TEST(NoAtFaultCollision, NonAgentRearCollisionIsStoppedTrackByDefinition)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(-1.5, 0.0, autoware_perception_msgs::msg::ObjectClassification::HAZARD, 2.0));

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.5);
  EXPECT_EQ(result.reason, "at_fault_collision_with_non_agent");
}

TEST(NoAtFaultCollision, LaterAgentCollisionCanReduceEarlierNonAgentHalfPenalty)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();

  auto non_agent =
    make_object(4.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::HAZARD, 0.0);
  set_object_id(non_agent, 1U);
  objects->objects.push_back(non_agent);

  auto agent =
    make_object(20.0, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 0.0);
  set_object_id(agent, 2U);
  agent.kinematics.initial_twist_with_covariance.twist.linear.x = 10.0;
  objects->objects.push_back(agent);

  auto agent_later = agent;
  agent_later.kinematics.initial_pose_with_covariance.pose = make_pose(8.5, 0.0);
  auto future_objects = make_future_objects(objects->objects);
  future_objects = append_future_objects(future_objects, {agent_later}, 1.0);

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
  EXPECT_DOUBLE_EQ(result.infraction_time_s, 1.0);
}

TEST(NoAtFaultCollision, AlreadyCollidedObjectIsSkippedAtLaterTimesteps)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();

  auto object =
    make_object(-1.5, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR, 10.0);
  set_object_id(object, 3U);
  objects->objects.push_back(object);

  auto object_later = object;
  object_later.kinematics.initial_pose_with_covariance.pose = make_pose(8.5, 0.0);
  auto future_objects = make_future_objects(objects->objects);
  future_objects = append_future_objects(future_objects, {object_later}, 1.0);

  const auto result =
    calculate_no_at_fault_collision(trajectory, future_objects, make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, LateralCollisionInsideRoadLaneIsNotAtFault)
{
  const auto trajectory = make_single_point_trajectory(0.0, 0.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(1.0, 0.9, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));
  const auto route_handler = make_route_handler({make_road_lanelet(1, -2.0, 2.0)});

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), route_handler);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, LateralCollisionWithCornerOutsideDrivableAreaIsAtFault)
{
  const auto trajectory = make_single_point_trajectory(0.0, 1.5);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(1.0, 2.4, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));
  const auto route_handler = make_route_handler({make_road_lanelet(1, -2.0, 2.0)});

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), route_handler);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_lateral_collision_with_agent");
}

TEST(NoAtFaultCollision, NonRouteRoadSurfaceIsStillDrivableForLateralAssessment)
{
  const auto trajectory = make_single_point_trajectory(0.0, 3.0);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(1.0, 3.9, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));
  const auto route_handler =
    make_route_handler({make_road_lanelet(1, -2.0, 2.0), make_road_lanelet(2, 2.0, 6.0)});

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), route_handler);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_EQ(result.reason, "available");
}

TEST(NoAtFaultCollision, LateralCollisionStraddlingMultipleLanesIsAtFault)
{
  const auto trajectory = make_single_point_trajectory(0.0, 1.9);
  auto objects = std::make_shared<PredictedObjects>();
  objects->objects.push_back(
    make_object(1.0, 2.8, autoware_perception_msgs::msg::ObjectClassification::CAR, 2.0));
  const auto route_handler =
    make_route_handler({make_road_lanelet(1, -2.0, 2.0), make_road_lanelet(2, 2.0, 6.0)});

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info(), route_handler);

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_lateral_collision_with_agent");
}

TEST(NoAtFaultCollision, IgnoresPredictedPathsAndUsesLoggedObjectPose)
{
  const auto trajectory = make_straight_trajectory(5.0);
  auto objects = std::make_shared<PredictedObjects>();

  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose = make_pose(4.0, 0.0);
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.5;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0f;
  object.classification.push_back(classification);

  autoware_perception_msgs::msg::PredictedPath colliding_path;
  colliding_path.time_step = rclcpp::Duration::from_seconds(0.5);
  colliding_path.confidence = 0.1;
  colliding_path.path.push_back(make_pose(4.0, 0.0));
  colliding_path.path.push_back(make_pose(4.0, 0.0));
  colliding_path.path.push_back(make_pose(4.0, 0.0));

  autoware_perception_msgs::msg::PredictedPath safe_path;
  safe_path.time_step = rclcpp::Duration::from_seconds(0.5);
  safe_path.confidence = 0.9;
  safe_path.path.push_back(make_pose(20.0, 0.0));
  safe_path.path.push_back(make_pose(20.0, 0.0));
  safe_path.path.push_back(make_pose(20.0, 0.0));

  object.kinematics.predicted_paths = {colliding_path, safe_path};
  objects->objects.push_back(object);

  const auto result = calculate_no_at_fault_collision(
    trajectory, make_future_objects(objects->objects), make_vehicle_info());

  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_EQ(result.reason, "at_fault_collision_with_agent");
}

}  // namespace autoware::planning_data_analyzer::metrics
