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

#include "epdms_context.hpp"

namespace autoware::planning_data_analyzer::metrics
{

EpdmsContext build_epdms_context(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<TimedTrackedObjects> & future_objects,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const EpdmsContextBuildOptions & options)
{
  EpdmsContext context;

  const bool needs_route_lanelets =
    options.route_relevant_lanelets || options.trajectory_footprint_evaluations;
  if (needs_route_lanelets && route_handler && route_handler->isHandlerReady()) {
    context.route_relevant_lanelets = collect_route_relevant_lanelets(trajectory, route_handler);
  }

  if (options.trajectory_footprint_evaluations && is_vehicle_info_valid(vehicle_info)) {
    context.trajectory_footprint_evaluations = evaluate_trajectory_footprints(
      trajectory, vehicle_info, route_handler, &context.route_relevant_lanelets,
      options.intersection_context);
  }

  if (options.object_tracks) {
    context.object_tracks = build_logged_object_tracks(future_objects);
  }

  return context;
}

}  // namespace autoware::planning_data_analyzer::metrics
