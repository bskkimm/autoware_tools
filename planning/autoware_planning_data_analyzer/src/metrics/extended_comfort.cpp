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

#include "extended_comfort.hpp"

#include "comfort_signal.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

constexpr double kDefaultTrajectoryDt = 0.1;

double get_time_seconds(const builtin_interfaces::msg::Duration & time_from_start)
{
  return static_cast<double>(time_from_start.sec) +
         static_cast<double>(time_from_start.nanosec) * 1e-9;
}

double trajectory_dt_s(const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.size() < 2U) {
    return kDefaultTrajectoryDt;
  }
  const double dt =
    get_time_seconds(trajectory.points.at(1).time_from_start) -
    get_time_seconds(trajectory.points.front().time_from_start);
  return dt > 0.0 ? dt : kDefaultTrajectoryDt;
}

std::vector<ComfortSignalInput> make_overlap_signal_inputs(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const std::size_t start_index,
  const std::size_t count, const double dt)
{
  std::vector<ComfortSignalInput> inputs;
  inputs.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto & point = trajectory.points.at(start_index + index);
    inputs.push_back(
      ComfortSignalInput{static_cast<double>(index) * dt, point.pose, point.acceleration_mps2, 0.0});
  }
  return inputs;
}

std::vector<double> subtract_signals(const std::vector<double> & current, const std::vector<double> & previous)
{
  const auto count = std::min(current.size(), previous.size());
  std::vector<double> delta;
  delta.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    delta.push_back(current.at(index) - previous.at(index));
  }
  return delta;
}

double rms(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum_squared = 0.0;
  for (const auto value : values) {
    sum_squared += value * value;
  }
  return std::sqrt(sum_squared / static_cast<double>(values.size()));
}

std::pair<double, double> abs_max_with_time(
  const std::vector<double> & values, const std::vector<double> & sample_times)
{
  double max_abs = 0.0;
  double time_s = 0.0;
  for (std::size_t index = 0; index < values.size() && index < sample_times.size(); ++index) {
    const double abs_value = std::abs(values.at(index));
    if (abs_value > max_abs) {
      max_abs = abs_value;
      time_s = sample_times.at(index);
    }
  }
  return {max_abs, time_s};
}

bool are_parameters_valid(const ExtendedComfortParameters & parameters)
{
  return parameters.max_acceleration_rms >= 0.0 && parameters.max_jerk_rms >= 0.0 &&
         parameters.max_yaw_rate_rms >= 0.0 && parameters.max_yaw_acceleration_rms >= 0.0 &&
         parameters.finite_difference_epsilon > 0.0;
}

void append_failed_component(
  std::ostringstream & summary, bool & first, const std::string & name, const bool ok)
{
  if (ok) {
    return;
  }
  if (!first) {
    summary << ",";
  }
  summary << "\"" << name << "\"";
  first = false;
}

std::string build_summary(
  const double score, const std::string & reason, const double rms_acceleration,
  const double rms_jerk, const double rms_yaw_rate, const double rms_yaw_accel,
  const bool acceleration_ok, const bool jerk_ok, const bool yaw_rate_ok,
  const bool yaw_accel_ok, const std::string & worst_component, const double worst_sample_time_s,
  const double worst_delta)
{
  std::ostringstream summary;
  summary << "{\"score\":" << score << ",\"reason\":\"" << reason << "\""
          << ",\"rms_acceleration\":" << rms_acceleration << ",\"rms_jerk\":" << rms_jerk
          << ",\"rms_yaw_rate\":" << rms_yaw_rate << ",\"rms_yaw_accel\":" << rms_yaw_accel
          << ",\"failed_components\":[";
  bool first = true;
  append_failed_component(summary, first, "acceleration", acceleration_ok);
  append_failed_component(summary, first, "jerk", jerk_ok);
  append_failed_component(summary, first, "yaw_rate", yaw_rate_ok);
  append_failed_component(summary, first, "yaw_accel", yaw_accel_ok);
  summary << "],\"worst_component\":\"" << worst_component
          << "\",\"worst_sample_time_s\":" << worst_sample_time_s
          << ",\"worst_delta\":" << worst_delta << "}";
  return summary.str();
}

}  // namespace

ExtendedComfortResult calculate_extended_comfort(
  const autoware_planning_msgs::msg::Trajectory & previous_trajectory,
  const autoware_planning_msgs::msg::Trajectory & current_trajectory,
  const ExtendedComfortParameters & parameters)
{
  ExtendedComfortResult result;
  if (!are_parameters_valid(parameters)) {
    result.reason = "unavailable_invalid_parameters";
    return result;
  }

  const double dt = trajectory_dt_s(current_trajectory);
  const double raw_observation_interval =
    (rclcpp::Time(current_trajectory.header.stamp) - rclcpp::Time(previous_trajectory.header.stamp))
      .seconds();
  const double observation_interval = raw_observation_interval > 0.0 ? raw_observation_interval : dt;
  const auto overlap_shift = static_cast<std::size_t>(std::llround(observation_interval / dt));
  if (dt <= 0.0 || overlap_shift == 0U) {
    result.reason = "unavailable_invalid_time_alignment";
    return result;
  }

  const auto previous_size = previous_trajectory.points.size();
  const auto current_size = current_trajectory.points.size();
  if (previous_size < 3U || current_size < 3U || overlap_shift >= previous_size) {
    result.reason = "unavailable_short_trajectory";
    return result;
  }

  const auto overlap_count = std::min(current_size, previous_size - overlap_shift);
  if (overlap_count < 3U) {
    result.reason = "unavailable_short_overlap";
    return result;
  }

  result.sample_times.reserve(overlap_count);
  for (std::size_t index = 0; index < overlap_count; ++index) {
    result.sample_times.push_back(static_cast<double>(index) * dt);
  }

  const auto current_inputs = make_overlap_signal_inputs(current_trajectory, 0U, overlap_count, dt);
  const auto previous_inputs =
    make_overlap_signal_inputs(previous_trajectory, overlap_shift, overlap_count, dt);
  const auto current_signals = compute_comfort_signals(current_inputs);
  const auto previous_signals = compute_comfort_signals(previous_inputs);

  result.delta_acceleration = subtract_signals(
    current_signals.acceleration_magnitudes, previous_signals.acceleration_magnitudes);
  result.delta_jerk =
    subtract_signals(current_signals.jerk_magnitudes, previous_signals.jerk_magnitudes);
  result.delta_yaw_rate = subtract_signals(current_signals.yaw_rates, previous_signals.yaw_rates);
  result.delta_yaw_accel =
    subtract_signals(current_signals.yaw_accelerations, previous_signals.yaw_accelerations);

  const double rms_acceleration = rms(result.delta_acceleration);
  const double rms_jerk = rms(result.delta_jerk);
  const double rms_yaw_rate = rms(result.delta_yaw_rate);
  const double rms_yaw_accel = rms(result.delta_yaw_accel);

  const bool acceleration_ok = rms_acceleration <= parameters.max_acceleration_rms;
  const bool jerk_ok = rms_jerk <= parameters.max_jerk_rms;
  const bool yaw_rate_ok = rms_yaw_rate <= parameters.max_yaw_rate_rms;
  const bool yaw_accel_ok = rms_yaw_accel <= parameters.max_yaw_acceleration_rms;

  std::string worst_component = "acceleration";
  auto [worst_delta, worst_sample_time_s] =
    abs_max_with_time(result.delta_acceleration, result.sample_times);
  const auto update_worst = [&](const std::string & name, const std::vector<double> & values) {
    const auto [component_delta, component_time] = abs_max_with_time(values, result.sample_times);
    if (component_delta > worst_delta) {
      worst_component = name;
      worst_delta = component_delta;
      worst_sample_time_s = component_time;
    }
  };
  update_worst("jerk", result.delta_jerk);
  update_worst("yaw_rate", result.delta_yaw_rate);
  update_worst("yaw_accel", result.delta_yaw_accel);

  result.available = true;
  result.reason = "available";
  result.score = acceleration_ok && jerk_ok && yaw_rate_ok && yaw_accel_ok ? 1.0 : 0.0;
  result.debug_summary = build_summary(
    result.score, result.reason, rms_acceleration, rms_jerk, rms_yaw_rate, rms_yaw_accel,
    acceleration_ok, jerk_ok, yaw_rate_ok, yaw_accel_ok, worst_component, worst_sample_time_s,
    worst_delta);
  return result;
}

}  // namespace autoware::planning_data_analyzer::metrics
