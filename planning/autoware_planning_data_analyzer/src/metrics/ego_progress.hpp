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

#ifndef METRICS__EGO_PROGRESS_HPP_
#define METRICS__EGO_PROGRESS_HPP_

#include "../data_types.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

struct EgoProgressResult
{
  double score{0.0};
  bool available{false};
  std::string reason{"unavailable"};
  double raw_progress_m{0.0};
  double best_raw_progress_m{0.0};
  double multiplicative_mask{0.0};
  double denominator_m{0.0};
  // Debug/visualization fields consumed by the open_loop debug topics.
  geometry_msgs::msg::Point start_point;
  geometry_msgs::msg::Point end_point;
  std::vector<geometry_msgs::msg::Point> route_reference_points;
};

struct EgoProgressMultiplicativeInputs
{
  double no_at_fault_collision{0.0};
  bool no_at_fault_collision_available{false};
  double drivable_area_compliance{0.0};
  bool drivable_area_compliance_available{false};
  double driving_direction_compliance{0.0};
  bool driving_direction_compliance_available{false};
  double traffic_light_compliance{0.0};
  bool traffic_light_compliance_available{false};
};

EgoProgressResult calculate_ego_progress(
  const std::shared_ptr<Trajectory> & selected_trajectory,
  const std::shared_ptr<RouteHandler> & route_handler,
  const EgoProgressMultiplicativeInputs & multiplicative_inputs);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EGO_PROGRESS_HPP_
