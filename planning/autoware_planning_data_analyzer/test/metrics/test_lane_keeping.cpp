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

#include "../../src/metrics/lane_keeping.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

using autoware::planning_data_analyzer::metrics::LaneKeepingEvaluationPoint;
using autoware::planning_data_analyzer::metrics::LaneKeepingParameters;

namespace
{

LaneKeepingEvaluationPoint make_evaluation_point(
  const double seconds, const double lateral_deviation, const bool is_in_intersection = false)
{
  LaneKeepingEvaluationPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(seconds);
  point.lateral_deviation = lateral_deviation;
  point.is_in_intersection = is_in_intersection;
  point.reference_lanelet_id = -1;
  return point;
}

LaneKeepingEvaluationPoint make_exemptable_point(
  const double seconds, const double lateral_deviation, const std::int64_t lanelet_id,
  const double speed_mps, const double cumulative_progress_m, const bool multiple_lanes = false)
{
  auto point = make_evaluation_point(seconds, lateral_deviation, false);
  point.reference_lanelet_id = lanelet_id;
  point.speed_mps = speed_mps;
  point.cumulative_progress_m = cumulative_progress_m;
  point.multiple_lanes = multiple_lanes;
  return point;
}

}  // namespace

TEST(LaneKeepingTest, ReturnsOneWhenAllDeviationsStayWithinThreshold)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.1), make_evaluation_point(1.0, -0.3),
    make_evaluation_point(2.0, 0.5)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsOneForShortThresholdSpike)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.0), make_evaluation_point(0.5, 0.8),
    make_evaluation_point(1.0, 0.1)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsZeroForContinuousViolationLongerThanWindow)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7), make_evaluation_point(1.0, 0.8),
    make_evaluation_point(2.1, 0.9)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(LaneKeepingTest, ReportsFailureRunMetadata)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.0), make_evaluation_point(0.5, 0.7),
    make_evaluation_point(1.5, 0.8), make_evaluation_point(2.6, 0.9)};

  const auto result = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_DOUBLE_EQ(result.debug.first_failure_time_s, 2.6);
  EXPECT_DOUBLE_EQ(result.debug.failure_run_start_time_s, 0.5);
  EXPECT_DOUBLE_EQ(result.debug.failure_run_end_time_s, 2.6);
  EXPECT_NEAR(result.debug.max_continuous_violation_time_s, 2.1, 1.0e-9);
  EXPECT_DOUBLE_EQ(result.debug.peak_abs_lateral_deviation_m, 0.9);
  ASSERT_EQ(result.debug.samples.size(), evaluation_points.size());
  EXPECT_FALSE(result.debug.samples.front().in_failure_run);
  EXPECT_TRUE(result.debug.samples.at(1).in_failure_run);
  EXPECT_TRUE(result.debug.samples.at(2).in_failure_run);
  EXPECT_TRUE(result.debug.samples.at(3).in_failure_run);
}

TEST(LaneKeepingTest, LaneChangeGraceSuppressesViolationRun)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_exemptable_point(0.0, 0.6, 1, 4.0, 0.0),
    make_exemptable_point(0.5, 0.6, 1, 4.0, 2.0, true),
    make_exemptable_point(1.0, 0.6, 2, 4.0, 4.0, true),
    make_exemptable_point(1.5, 0.6, 2, 4.0, 6.0),
    make_exemptable_point(2.1, 0.6, 2, 4.0, 8.0)};

  const auto result = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(
    evaluation_points, LaneKeepingParameters{0.5, 2.0}, {{0.0, 2.0}});

  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_TRUE(result.debug.samples.at(0).lane_change_exempt);
  EXPECT_TRUE(result.debug.samples.at(1).lane_change_exempt);
  EXPECT_TRUE(result.debug.samples.at(2).lane_change_exempt);
}

TEST(LaneKeepingTest, NoLaneChangeWindowDoesNotSuppressViolationRun)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_exemptable_point(0.0, 0.6, 1, 4.0, 0.0), make_exemptable_point(0.5, 0.6, 1, 4.0, 2.0),
    make_exemptable_point(1.0, 0.6, 2, 4.0, 4.0), make_exemptable_point(1.5, 0.6, 2, 4.0, 6.0),
    make_exemptable_point(2.1, 0.6, 2, 4.0, 8.0)};

  const auto result = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(result.score, 0.0);
  for (const auto & sample : result.debug.samples) {
    EXPECT_FALSE(sample.lane_change_exempt);
  }
}

TEST(LaneKeepingTest, QueueAndReleaseGraceSuppressViolationRun)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_exemptable_point(0.0, 0.59, 1, 0.0, 0.0),
    make_exemptable_point(0.5, 0.59, 1, 0.2, 0.05),
    make_exemptable_point(1.0, 0.59, 1, 0.3, 0.12),
    make_exemptable_point(1.5, 0.59, 1, 0.8, 0.30),
    make_exemptable_point(2.0, 0.59, 1, 1.2, 0.80),
    make_exemptable_point(2.5, 0.59, 1, 1.3, 1.50)};

  const auto result = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_TRUE(result.debug.samples.at(0).queue_exempt);
  EXPECT_TRUE(result.debug.samples.at(4).queue_release_exempt);
}

TEST(LaneKeepingTest, RoadBorderEnvelopeDoesNotSuppressCenterlineDeviationRun)
{
  std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.8), make_evaluation_point(1.0, 0.8),
    make_evaluation_point(2.0, 0.8), make_evaluation_point(3.0, 0.8)};
  for (auto & point : evaluation_points) {
    point.inside_road_border_envelope = true;
  }

  const auto result = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(result.score, 0.0);
  EXPECT_TRUE(std::all_of(
    result.debug.samples.begin(), result.debug.samples.end(),
    [](const auto & sample) { return sample.over_threshold; }));
}

TEST(LaneKeepingTest, DefaultThresholdAllowsSixtyCentimeterDeviation)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.6), make_evaluation_point(1.0, 0.6),
    make_evaluation_point(2.1, 0.6)};

  const auto result =
    autoware::planning_data_analyzer::metrics::calculate_lane_keeping_result(evaluation_points);

  EXPECT_DOUBLE_EQ(result.score, 1.0);
  EXPECT_FALSE(result.debug.samples.at(0).over_threshold);
}

TEST(LaneKeepingTest, IgnoresViolationsInsideIntersections)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.8, true), make_evaluation_point(1.0, 0.9, true),
    make_evaluation_point(2.5, 0.1, false)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 1.0);
}

TEST(LaneKeepingTest, ReturnsZeroWhenSamplesAreEmpty)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points;

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(LaneKeepingTest, NonFiniteGapBreaksContinuousViolationWindow)
{
  const std::vector<LaneKeepingEvaluationPoint> evaluation_points{
    make_evaluation_point(0.0, 0.7),
    make_evaluation_point(1.0, std::numeric_limits<double>::quiet_NaN()),
    make_evaluation_point(2.1, 0.9)};

  const auto score = autoware::planning_data_analyzer::metrics::calculate_lane_keeping_score(
    evaluation_points, LaneKeepingParameters{0.5, 2.0});

  EXPECT_DOUBLE_EQ(score, 1.0);
}
