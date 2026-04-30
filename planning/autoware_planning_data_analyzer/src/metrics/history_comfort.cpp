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

#include "metric_utils.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

namespace
{

struct ComfortState
{
  double time_s{0.0};
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

double factorial(const int n)
{
  double value = 1.0;
  for (int i = 2; i <= n; ++i) {
    value *= static_cast<double>(i);
  }
  return value;
}

std::vector<double> unwrap_angles(const std::vector<double> & values)
{
  if (values.empty()) {
    return {};
  }
  std::vector<double> unwrapped(values.size(), 0.0);
  unwrapped.front() = values.front();
  for (std::size_t i = 1; i < values.size(); ++i) {
    const double delta = autoware_utils_math::normalize_radian(values.at(i) - values.at(i - 1U));
    unwrapped.at(i) = unwrapped.at(i - 1U) + delta;
  }
  return unwrapped;
}

std::optional<std::vector<double>> solve_linear_system(
  std::vector<std::vector<double>> lhs, std::vector<double> rhs)
{
  const std::size_t n = rhs.size();
  for (std::size_t pivot = 0; pivot < n; ++pivot) {
    std::size_t best = pivot;
    for (std::size_t row = pivot + 1U; row < n; ++row) {
      if (std::abs(lhs.at(row).at(pivot)) > std::abs(lhs.at(best).at(pivot))) {
        best = row;
      }
    }
    if (std::abs(lhs.at(best).at(pivot)) < 1.0e-12) {
      return std::nullopt;
    }
    if (best != pivot) {
      std::swap(lhs.at(best), lhs.at(pivot));
      std::swap(rhs.at(best), rhs.at(pivot));
    }
    const double divisor = lhs.at(pivot).at(pivot);
    for (std::size_t col = pivot; col < n; ++col) {
      lhs.at(pivot).at(col) /= divisor;
    }
    rhs.at(pivot) /= divisor;
    for (std::size_t row = 0; row < n; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = lhs.at(row).at(pivot);
      for (std::size_t col = pivot; col < n; ++col) {
        lhs.at(row).at(col) -= factor * lhs.at(pivot).at(col);
      }
      rhs.at(row) -= factor * rhs.at(pivot);
    }
  }
  return rhs;
}

std::vector<double> local_polynomial_filter(
  const std::vector<double> & values, const std::vector<double> & times, const int window_length,
  const int poly_order, const int derivative_order)
{
  std::vector<double> output(values.size(), 0.0);
  if (values.empty() || values.size() != times.size()) {
    return output;
  }
  const int effective_poly_order =
    std::min<int>(poly_order, std::max<int>(0, static_cast<int>(values.size()) - 1));
  if (derivative_order > effective_poly_order) {
    return output;
  }

  const std::size_t coefficients = static_cast<std::size_t>(effective_poly_order + 1);
  const std::size_t target_window =
    std::max<std::size_t>(coefficients, std::min<std::size_t>(window_length, values.size()));

  for (std::size_t i = 0; i < values.size(); ++i) {
    std::size_t start = i > target_window / 2U ? i - target_window / 2U : 0U;
    std::size_t end = std::min(values.size(), start + target_window);
    if (end - start < target_window) {
      start = end > target_window ? end - target_window : 0U;
    }
    std::vector<std::vector<double>> lhs(coefficients, std::vector<double>(coefficients, 0.0));
    std::vector<double> rhs(coefficients, 0.0);
    for (std::size_t sample = start; sample < end; ++sample) {
      const double centered_t = times.at(sample) - times.at(i);
      std::vector<double> powers(coefficients, 1.0);
      for (std::size_t power = 1; power < coefficients; ++power) {
        powers.at(power) = powers.at(power - 1U) * centered_t;
      }
      for (std::size_t row = 0; row < coefficients; ++row) {
        rhs.at(row) += powers.at(row) * values.at(sample);
        for (std::size_t col = 0; col < coefficients; ++col) {
          lhs.at(row).at(col) += powers.at(row) * powers.at(col);
        }
      }
    }
    const auto solution = solve_linear_system(std::move(lhs), std::move(rhs));
    if (solution.has_value()) {
      output.at(i) = solution->at(static_cast<std::size_t>(derivative_order)) *
                     factorial(derivative_order);
    }
  }
  return output;
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

  std::vector<double> times;
  std::vector<double> yaws;
  std::vector<double> longitudinal_accelerations_raw;
  std::vector<double> lateral_accelerations_raw;
  std::vector<double> acceleration_magnitudes_raw;
  times.reserve(states.size());
  metrics.history_comfort_sample_times.reserve(states.size());
  metrics.history_comfort_segment_ids.reserve(states.size());
  yaws.reserve(states.size());
  longitudinal_accelerations_raw.reserve(states.size());
  lateral_accelerations_raw.reserve(states.size());
  acceleration_magnitudes_raw.reserve(states.size());

  for (const auto & state : states) {
    times.push_back(state.time_s + history_comfort_params.past_horizon_s);
    metrics.history_comfort_sample_times.push_back(state.time_s);
    metrics.history_comfort_segment_ids.push_back(state.segment_id);
    yaws.push_back(state.yaw);
    longitudinal_accelerations_raw.push_back(state.longitudinal_acceleration_mps2);
    lateral_accelerations_raw.push_back(state.lateral_acceleration_mps2);
    acceleration_magnitudes_raw.push_back(
      std::hypot(state.longitudinal_acceleration_mps2, state.lateral_acceleration_mps2));
  }

  metrics.longitudinal_accelerations = local_polynomial_filter(
    longitudinal_accelerations_raw, times, 8, 2, 0);
  metrics.lateral_accelerations = local_polynomial_filter(lateral_accelerations_raw, times, 8, 2, 0);
  const auto acceleration_magnitudes =
    local_polynomial_filter(acceleration_magnitudes_raw, times, 8, 2, 0);
  metrics.longitudinal_jerks = local_polynomial_filter(
    metrics.longitudinal_accelerations, times, 15, 2, 1);
  metrics.lateral_jerks =
    local_polynomial_filter(metrics.lateral_accelerations, times, 15, 2, 1);
  metrics.jerk_magnitudes = local_polynomial_filter(acceleration_magnitudes, times, 15, 2, 1);

  const auto unwrapped_yaws = unwrap_angles(yaws);
  metrics.yaw_rates = local_polynomial_filter(unwrapped_yaws, times, 15, 2, 1);
  metrics.yaw_accelerations = local_polynomial_filter(unwrapped_yaws, times, 15, 3, 2);

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
