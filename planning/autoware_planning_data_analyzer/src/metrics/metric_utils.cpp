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

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
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

void canonicalize_bounding_box_yaws(LoggedObjectTrack & track)
{
  if (track.states.size() < 2U) {
    return;
  }

  bool has_reference_yaw = false;
  double reference_yaw = 0.0;
  for (auto & state : track.states) {
    if (state.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      has_reference_yaw = false;
      continue;
    }

    const double raw_yaw = get_yaw(state.pose.orientation);
    const double canonical_yaw =
      has_reference_yaw ? closest_pi_symmetric_yaw(reference_yaw, raw_yaw) : raw_yaw;
    state.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(canonical_yaw);
    reference_yaw = canonical_yaw;
    has_reference_yaw = true;
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

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedPredictedObjects> & future_objects)
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
      state.pose = object.kinematics.initial_pose_with_covariance.pose;
      state.twist = object.kinematics.initial_twist_with_covariance.twist;
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
  object.pose = autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio);
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
