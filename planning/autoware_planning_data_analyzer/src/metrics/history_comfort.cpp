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

#include "history_comfort.hpp"

#include "comfort_signal.hpp"
#include "metric_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

struct ComfortState
{
  double time_s{0.0};
  geometry_msgs::msg::Pose pose;
  double yaw{0.0};
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double longitudinal_acceleration_mps2{0.0};
  double lateral_acceleration_mps2{0.0};
  double segment_id{2.0};
};

double stamp_to_seconds(const rclcpp::Time & stamp)
{
  return static_cast<double>(stamp.nanoseconds()) * 1.0e-9;
}

template <typename MessageT>
std::shared_ptr<const MessageT> closest_message(
  const std::vector<std::shared_ptr<const MessageT>> & history, const rclcpp::Time & target,
  const double tolerance_s)
{
  if (history.empty()) {
    return nullptr;
  }
  std::shared_ptr<const MessageT> best;
  double best_diff = std::numeric_limits<double>::max();
  for (const auto & msg : history) {
    if (!msg) {
      continue;
    }
    const double diff = std::abs(stamp_to_seconds(rclcpp::Time(msg->header.stamp)) -
                                 stamp_to_seconds(target));
    if (diff < best_diff) {
      best_diff = diff;
      best = msg;
    }
  }
  return best_diff <= tolerance_s ? best : nullptr;
}

ComfortState make_state_from_odometry(
  const Odometry & odometry, const AccelWithCovarianceStamped * acceleration,
  const double relative_time_s)
{
  ComfortState state;
  state.time_s = relative_time_s;
  state.pose = odometry.pose.pose;
  state.yaw = get_yaw(odometry.pose.pose.orientation);
  state.longitudinal_velocity_mps = odometry.twist.twist.linear.x;
  state.lateral_velocity_mps = odometry.twist.twist.linear.y;
  state.segment_id = 0.0;
  if (acceleration) {
    state.longitudinal_acceleration_mps2 = acceleration->accel.accel.linear.x;
    state.lateral_acceleration_mps2 = acceleration->accel.accel.linear.y;
  }
  return state;
}

ComfortState make_state_from_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & point, const double relative_time_s)
{
  ComfortState state;
  state.time_s = relative_time_s;
  state.pose = point.pose;
  state.yaw = get_yaw(point.pose.orientation);
  state.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  state.lateral_velocity_mps = point.lateral_velocity_mps;
  state.longitudinal_acceleration_mps2 = point.acceleration_mps2;
  state.segment_id = 2.0;
  return state;
}

std::vector<ComfortState> build_padded_states(
  const SynchronizedData & sync_data, const HistoryComfortParameters & parameters)
{
  std::vector<ComfortState> states;
  if (!sync_data.trajectory) {
    return states;
  }

  const auto trajectory_start = rclcpp::Time(sync_data.trajectory->header.stamp);
  const double dt = parameters.sample_interval_s;
  if (dt <= 0.0) {
    return states;
  }

  for (double t = -parameters.past_horizon_s; t < -dt; t += dt) {
    const auto target = trajectory_start + rclcpp::Duration::from_seconds(t);
    const auto odom = closest_message(sync_data.kinematic_state_history, target, dt * 0.75);
    if (!odom) {
      continue;
    }
    const auto accel = closest_message(sync_data.acceleration_history, target, dt * 0.75);
    states.push_back(make_state_from_odometry(*odom, accel.get(), t));
  }

  for (const auto & point : sync_data.trajectory->points) {
    const double t = rclcpp::Duration(point.time_from_start).seconds();
    if (t < 0.0) {
      continue;
    }
    if (t > parameters.future_horizon_s + 1.0e-6) {
      break;
    }
    states.push_back(make_state_from_trajectory_point(point, t));
  }

  return states;
}

bool is_within(const std::vector<double> & values, const double min_value, const double max_value)
{
  return std::all_of(values.begin(), values.end(), [min_value, max_value](const double value) {
    return min_value < value && value < max_value;
  });
}

bool is_abs_within(const std::vector<double> & values, const double max_abs_value)
{
  return std::all_of(values.begin(), values.end(), [max_abs_value](const double value) {
    return std::abs(value) < max_abs_value;
  });
}

std::pair<double, double> min_max(const std::vector<double> & values)
{
  if (values.empty()) {
    return {0.0, 0.0};
  }
  const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
  return {*min_it, *max_it};
}

std::pair<double, double> abs_max_with_time(
  const std::vector<double> & values, const std::vector<double> & relative_times)
{
  double max_abs = 0.0;
  double time_s = 0.0;
  for (std::size_t i = 0; i < values.size() && i < relative_times.size(); ++i) {
    const double value = std::abs(values.at(i));
    if (value > max_abs) {
      max_abs = value;
      time_s = relative_times.at(i);
    }
  }
  return {max_abs, time_s};
}

}  // namespace

void calculate_history_comfort_metrics(
  const SynchronizedData & sync_data, const HistoryComfortParameters & history_comfort_params,
  TrajectoryPointMetrics & metrics)
{
  const auto states = build_padded_states(sync_data, history_comfort_params);
  if (states.empty()) {
    metrics.history_comfort = 1.0;
    metrics.history_comfort_debug_summary =
      "{\"score\":1,\"reason\":\"missing_past_or_future_states_navsim_default_pass\"}";
    return;
  }

  std::vector<ComfortSignalInput> signal_inputs;
  signal_inputs.reserve(states.size());
  metrics.history_comfort_sample_times.reserve(states.size());
  metrics.history_comfort_segment_ids.reserve(states.size());
  metrics.history_comfort_sample_poses.reserve(states.size());

  for (const auto & state : states) {
    metrics.history_comfort_sample_times.push_back(state.time_s);
    metrics.history_comfort_segment_ids.push_back(state.segment_id);
    metrics.history_comfort_sample_poses.push_back(state.pose);
    signal_inputs.push_back(
      ComfortSignalInput{state.time_s, state.pose, state.longitudinal_acceleration_mps2,
                         state.lateral_acceleration_mps2});
  }

  const auto signals = compute_comfort_signals(signal_inputs, history_comfort_params.past_horizon_s);
  metrics.longitudinal_accelerations = signals.longitudinal_accelerations;
  metrics.lateral_accelerations = signals.lateral_accelerations;
  metrics.longitudinal_jerks = signals.longitudinal_jerks;
  metrics.lateral_jerks = signals.lateral_jerks;
  metrics.jerk_magnitudes = signals.jerk_magnitudes;
  metrics.yaw_rates = signals.yaw_rates;
  metrics.yaw_accelerations = signals.yaw_accelerations;

  const bool longitudinal_acceleration_ok = is_within(
    metrics.longitudinal_accelerations, history_comfort_params.min_longitudinal_acceleration,
    history_comfort_params.max_longitudinal_acceleration);
  const bool lateral_acceleration_ok =
    is_abs_within(metrics.lateral_accelerations, history_comfort_params.max_lateral_acceleration);
  const bool jerk_magnitude_ok =
    is_abs_within(metrics.jerk_magnitudes, history_comfort_params.max_jerk_magnitude);
  const bool longitudinal_jerk_ok =
    is_abs_within(metrics.longitudinal_jerks, history_comfort_params.max_longitudinal_jerk);
  const bool yaw_rate_ok = is_abs_within(metrics.yaw_rates, history_comfort_params.max_yaw_rate);
  const bool yaw_acceleration_ok =
    is_abs_within(metrics.yaw_accelerations, history_comfort_params.max_yaw_acceleration);

  metrics.history_comfort = longitudinal_acceleration_ok && lateral_acceleration_ok &&
                                jerk_magnitude_ok && longitudinal_jerk_ok && yaw_rate_ok &&
                                yaw_acceleration_ok
                              ? 1.0
                              : 0.0;

  const auto [min_ax, max_ax] = min_max(metrics.longitudinal_accelerations);
  const auto [max_abs_ay, max_abs_ay_t] =
    abs_max_with_time(metrics.lateral_accelerations, metrics.history_comfort_sample_times);
  const auto [max_abs_j, max_abs_j_t] =
    abs_max_with_time(metrics.jerk_magnitudes, metrics.history_comfort_sample_times);
  const auto [max_abs_jx, max_abs_jx_t] =
    abs_max_with_time(metrics.longitudinal_jerks, metrics.history_comfort_sample_times);
  const auto [max_abs_yaw_rate, max_abs_yaw_rate_t] =
    abs_max_with_time(metrics.yaw_rates, metrics.history_comfort_sample_times);
  const auto [max_abs_yaw_accel, max_abs_yaw_accel_t] =
    abs_max_with_time(metrics.yaw_accelerations, metrics.history_comfort_sample_times);
  std::ostringstream summary;
  summary << "{\"score\":" << metrics.history_comfort << ",\"sample_count\":" << states.size()
          << ",\"past_horizon_s\":" << history_comfort_params.past_horizon_s
          << ",\"future_horizon_s\":" << history_comfort_params.future_horizon_s
          << ",\"failed_components\":[";
  bool first = true;
  const auto add_failed = [&](const char * name, const bool ok) {
    if (ok) {
      return;
    }
    if (!first) {
      summary << ",";
    }
    summary << "\"" << name << "\"";
    first = false;
  };
  add_failed("ax", longitudinal_acceleration_ok);
  add_failed("ay", lateral_acceleration_ok);
  add_failed("jerk", jerk_magnitude_ok);
  add_failed("jx", longitudinal_jerk_ok);
  add_failed("yaw_rate", yaw_rate_ok);
  add_failed("yaw_accel", yaw_acceleration_ok);
  summary << "],\"ax_min\":" << min_ax << ",\"ax_max\":" << max_ax
          << ",\"ay_abs_max\":" << max_abs_ay << ",\"ay_abs_max_time_s\":" << max_abs_ay_t
          << ",\"jerk_abs_max\":" << max_abs_j << ",\"jerk_abs_max_time_s\":" << max_abs_j_t
          << ",\"jx_abs_max\":" << max_abs_jx << ",\"jx_abs_max_time_s\":" << max_abs_jx_t
          << ",\"yaw_rate_abs_max\":" << max_abs_yaw_rate
          << ",\"yaw_rate_abs_max_time_s\":" << max_abs_yaw_rate_t
          << ",\"yaw_accel_abs_max\":" << max_abs_yaw_accel
          << ",\"yaw_accel_abs_max_time_s\":" << max_abs_yaw_accel_t << "}";
  metrics.history_comfort_debug_summary = summary.str();
}

}  // namespace autoware::planning_data_analyzer::metrics
