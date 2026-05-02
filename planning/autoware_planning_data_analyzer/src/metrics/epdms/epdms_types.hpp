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

#ifndef METRICS__EPDMS__EPDMS_TYPES_HPP_
#define METRICS__EPDMS__EPDMS_TYPES_HPP_

#include "../geometry/metric_utils.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

struct EpdmsContextBuildOptions
{
  bool route_relevant_lanelets{false};
  bool trajectory_footprint_evaluations{false};
  bool object_tracks{false};
  bool intersection_context{false};
};

struct EpdmsContext
{
  lanelet::ConstLanelets route_relevant_lanelets;
  std::vector<TrajectoryFootprintEvaluation> trajectory_footprint_evaluations;
  std::vector<LoggedObjectTrack> object_tracks;
};

}  // namespace autoware::planning_data_analyzer::metrics

#endif  // METRICS__EPDMS__EPDMS_TYPES_HPP_
