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

#ifndef METRICS__EPDMS__EPDMS_CONTEXT_HPP_
#define METRICS__EPDMS__EPDMS_CONTEXT_HPP_

#include "epdms_types.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>

namespace autoware::planning_data_analyzer::metrics
{

using autoware::route_handler::RouteHandler;

EpdmsContext build_epdms_context(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const EpdmsContextBuildOptions & options);

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__EPDMS_CONTEXT_HPP_
