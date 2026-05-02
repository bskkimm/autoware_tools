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
#include <autoware_lanelet2_extension/utility/utilities.hpp>
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

constexpr double kLocalLaneSearchRadiusM = 5.0;
constexpr double kDirectionSimilarityThresholdRad = M_PI_4;
constexpr double kAdmissibleLaneMarginM = 0.35;
constexpr double kSemanticDrivableAreaSearchMarginM = 15.0;
constexpr double kRoadBorderSearchMarginM = 5.0;
constexpr double kRoadBorderMaxGapM = 3.0;
constexpr double kRoadBorderMaxSemanticToBorderM = 4.0;
constexpr double kRoadBorderBetweenToleranceM = 0.75;
constexpr double kRoadBorderMaxTangentAlignment = 0.6;
constexpr double kMinRoadBorderSegmentLengthM = 1.0e-3;
constexpr double kRoadBorderSideEpsilon = 1.0e-3;
constexpr std::array<double, 8> kRoadBorderSideProbeDistancesM{0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0};

void append_unique_lanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets & lanelets,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(lanelet.id()).second) {
    lanelets.push_back(lanelet);
  }
}

void append_unique_polygon(
  const lanelet::ConstPolygon3d & polygon, std::vector<lanelet::ConstPolygon3d> & polygons,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(polygon.id()).second) {
    polygons.push_back(polygon);
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

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & polygon)
{
  namespace bg = boost::geometry;

  autoware_utils_geometry::Polygon2d converted;
  for (const auto & point : polygon) {
    converted.outer().push_back({point.x(), point.y()});
  }
  bg::correct(converted);
  return converted;
}

bool footprint_intersects_lanelet(
  const autoware_utils_geometry::Polygon2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return !bg::disjoint(footprint, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

std::vector<autoware_utils_geometry::Point2d> footprint_vertices(
  const autoware_utils_geometry::Polygon2d & footprint)
{
  namespace bg = boost::geometry;

  std::vector<autoware_utils_geometry::Point2d> vertices;
  for (const auto & point : footprint.outer()) {
    if (vertices.empty() || !bg::equals(vertices.front(), point)) {
      vertices.push_back(point);
    }
  }
  return vertices;
}

lanelet::BoundingBox2d footprint_bounding_box(
  const autoware_utils_geometry::Polygon2d & footprint, const double search_margin_m = 1.0e-3)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : footprint.outer()) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }

  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{min_x - search_margin_m, min_y - search_margin_m},
    lanelet::BasicPoint2d{max_x + search_margin_m, max_y + search_margin_m}};
}

lanelet::BoundingBox2d point_bounding_box(
  const geometry_msgs::msg::Point & point, const double radius_m = kLocalLaneSearchRadiusM)
{
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{point.x - radius_m, point.y - radius_m},
    lanelet::BasicPoint2d{point.x + radius_m, point.y + radius_m}};
}

lanelet::ConstLanelets collect_candidate_road_lanelets(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & designated_lanelets)
{
  lanelet::ConstLanelets road_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & lanelet : designated_lanelets) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    if (seen_ids.insert(lanelet.id()).second) {
      road_lanelets.push_back(lanelet);
    }
  }

  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & lanelet :
       map->laneletLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    if (!route_handler->isRoadLanelet(lanelet)) {
      continue;
    }
    if (seen_ids.insert(lanelet.id()).second) {
      road_lanelets.push_back(lanelet);
    }
  }

  return road_lanelets;
}

lanelet::ConstLanelets collect_candidate_shoulder_lanelets(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets shoulder_lanelets;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return shoulder_lanelets;
  }

  std::unordered_set<lanelet::Id> seen_ids;
  const auto map = route_handler->getLaneletMapPtr();
  for (const auto & lanelet :
       map->laneletLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    if (!route_handler->isShoulderLanelet(lanelet)) {
      continue;
    }
    if (seen_ids.insert(lanelet.id()).second) {
      shoulder_lanelets.push_back(lanelet);
    }
  }

  return shoulder_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_candidate_map_polygons(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler, const std::unordered_set<std::string> & types)
{
  std::vector<lanelet::ConstPolygon3d> polygons;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return polygons;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & polygon :
       map->polygonLayer.search(
         footprint_bounding_box(ego_polygon, kSemanticDrivableAreaSearchMarginM))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (types.count(type) == 0U) {
      continue;
    }
    if (seen_ids.insert(polygon.id()).second) {
      polygons.push_back(polygon);
    }
  }

  return polygons;
}

std::vector<lanelet::ConstLineString3d> collect_candidate_road_border_lines(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstLineString3d> road_border_lines;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return road_border_lines;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  const auto search_bbox = footprint_bounding_box(ego_polygon, kRoadBorderSearchMarginM);
  for (const auto & line_string : map->lineStringLayer.search(search_bbox)) {
    const std::string type = line_string.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "road_border") {
      continue;
    }
    if (seen_ids.insert(line_string.id()).second) {
      road_border_lines.push_back(line_string);
    }
  }

  return road_border_lines;
}

bool point_in_lanelet(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet.polygon2d().basicPolygon()));
}

bool point_in_polygon(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstPolygon3d & polygon)
{
  namespace bg = boost::geometry;
  return bg::covered_by(point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
}

bool point_in_semantic_drivable_area(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  for (const auto & lanelet : road_lanelets) {
    if (point_in_lanelet(point, lanelet)) {
      return true;
    }
  }
  for (const auto & lanelet : shoulder_lanelets) {
    if (point_in_lanelet(point, lanelet)) {
      return true;
    }
  }
  for (const auto & polygon : intersection_areas) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  for (const auto & polygon : hatched_road_markings) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  for (const auto & polygon : parking_lots) {
    if (point_in_polygon(point, polygon)) {
      return true;
    }
  }
  return false;
}

double distance_to_semantic_drivable_area(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  namespace bg = boost::geometry;

  double min_distance_m = std::numeric_limits<double>::infinity();
  const auto update_min_distance = [&point, &min_distance_m](const auto & polygon) {
    min_distance_m = std::min(min_distance_m, bg::distance(point, polygon));
  };

  for (const auto & lanelet : road_lanelets) {
    update_min_distance(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
  }
  for (const auto & lanelet : shoulder_lanelets) {
    update_min_distance(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
  }
  for (const auto & polygon : intersection_areas) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  for (const auto & polygon : hatched_road_markings) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  for (const auto & polygon : parking_lots) {
    update_min_distance(to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()));
  }
  return min_distance_m;
}

struct ClosestSemanticBoundaryPoint
{
  autoware_utils_geometry::Point2d point;
  double distance_m{std::numeric_limits<double>::infinity()};
};

double squared_distance(
  const autoware_utils_geometry::Point2d & a, const autoware_utils_geometry::Point2d & b)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  return dx * dx + dy * dy;
}

autoware_utils_geometry::Point2d closest_point_on_segment(
  const autoware_utils_geometry::Point2d & point, const autoware_utils_geometry::Point2d & start,
  const autoware_utils_geometry::Point2d & end)
{
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  const double length_sq = dx * dx + dy * dy;
  if (length_sq < kMinRoadBorderSegmentLengthM * kMinRoadBorderSegmentLengthM) {
    return start;
  }
  const double projection =
    ((point.x() - start.x()) * dx + (point.y() - start.y()) * dy) / length_sq;
  const double clamped_projection = std::clamp(projection, 0.0, 1.0);
  return autoware_utils_geometry::Point2d{
    start.x() + clamped_projection * dx, start.y() + clamped_projection * dy};
}

void update_closest_boundary_from_polygon(
  const autoware_utils_geometry::Point2d & point, const autoware_utils_geometry::Polygon2d & polygon,
  std::optional<ClosestSemanticBoundaryPoint> & best)
{
  const auto & ring = polygon.outer();
  if (ring.size() < 2U) {
    return;
  }
  for (std::size_t index = 1; index < ring.size(); ++index) {
    const auto closest = closest_point_on_segment(point, ring.at(index - 1U), ring.at(index));
    const double distance = std::sqrt(squared_distance(point, closest));
    if (!best.has_value() || distance < best->distance_m) {
      best = ClosestSemanticBoundaryPoint{closest, distance};
    }
  }
}

std::optional<ClosestSemanticBoundaryPoint> find_closest_semantic_boundary_point(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  std::optional<ClosestSemanticBoundaryPoint> best;
  for (const auto & lanelet : road_lanelets) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet.polygon2d().basicPolygon()), best);
  }
  for (const auto & lanelet : shoulder_lanelets) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet.polygon2d().basicPolygon()), best);
  }
  for (const auto & polygon : intersection_areas) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  for (const auto & polygon : hatched_road_markings) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  for (const auto & polygon : parking_lots) {
    update_closest_boundary_from_polygon(
      point, to_polygon_2d(lanelet::utils::to2D(polygon).basicPolygon()), best);
  }
  return best;
}

std::vector<bool> evaluate_road_border_side_fallback(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const std::vector<bool> & semantic_corner_drivable, const lanelet::ConstLanelets & road_lanelets,
  const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots,
  const std::vector<lanelet::ConstLineString3d> & road_border_lines,
  std::vector<RoadBorderSideTest> & side_tests)
{
  std::vector<bool> corner_drivable_by_road_border(footprint_points.size(), false);
  for (std::size_t index = 0; index < footprint_points.size(); ++index) {
    if (semantic_corner_drivable.at(index)) {
      continue;
    }
    const auto semantic_boundary = find_closest_semantic_boundary_point(
      footprint_points.at(index), road_lanelets, shoulder_lanelets, intersection_areas,
      hatched_road_markings, parking_lots);
    if (!semantic_boundary.has_value()) {
      continue;
    }

    std::optional<RoadBorderSideTest> best_rejected_test;
    std::optional<RoadBorderSideTest> best_accepted_test;
    for (const auto & line_string : road_border_lines) {
      const auto line_2d = lanelet::utils::to2D(line_string);
      if (line_2d.size() < 2U) {
        continue;
      }
      for (std::size_t segment_index = 1; segment_index < line_2d.size(); ++segment_index) {
        RoadBorderSideTest test;
        test.corner_index = index;
        test.corner = footprint_points.at(index);
        test.semantic_closest_point = semantic_boundary->point;
        test.segment_start = autoware_utils_geometry::Point2d{
          line_2d[segment_index - 1U].x(), line_2d[segment_index - 1U].y()};
        test.segment_end =
          autoware_utils_geometry::Point2d{line_2d[segment_index].x(), line_2d[segment_index].y()};
        const double segment_dx = test.segment_end.x() - test.segment_start.x();
        const double segment_dy = test.segment_end.y() - test.segment_start.y();
        const double segment_length = std::hypot(segment_dx, segment_dy);
        if (segment_length < kMinRoadBorderSegmentLengthM) {
          continue;
        }

        test.closest_point = closest_point_on_segment(test.corner, test.segment_start, test.segment_end);
        test.distance_m = std::sqrt(squared_distance(test.corner, test.closest_point));
        test.corner_semantic_distance_m = semantic_boundary->distance_m;

        const double semantic_to_border_x = test.closest_point.x() - semantic_boundary->point.x();
        const double semantic_to_border_y = test.closest_point.y() - semantic_boundary->point.y();
        const double semantic_to_border_sq =
          semantic_to_border_x * semantic_to_border_x + semantic_to_border_y * semantic_to_border_y;
        test.semantic_to_border_distance_m = std::sqrt(semantic_to_border_sq);
        if (semantic_to_border_sq > kMinRoadBorderSegmentLengthM * kMinRoadBorderSegmentLengthM) {
          const double corner_vector_x = test.corner.x() - semantic_boundary->point.x();
          const double corner_vector_y = test.corner.y() - semantic_boundary->point.y();
          test.corner_between_ratio =
            (corner_vector_x * semantic_to_border_x + corner_vector_y * semantic_to_border_y) /
            semantic_to_border_sq;
          const auto projected_on_semantic_to_border = autoware_utils_geometry::Point2d{
            semantic_boundary->point.x() + test.corner_between_ratio * semantic_to_border_x,
            semantic_boundary->point.y() + test.corner_between_ratio * semantic_to_border_y};
          test.corner_to_semantic_border_line_m =
            std::sqrt(squared_distance(test.corner, projected_on_semantic_to_border));
          const double tangent_x = segment_dx / segment_length;
          const double tangent_y = segment_dy / segment_length;
          test.border_tangent_alignment = std::abs(
            (semantic_to_border_x / test.semantic_to_border_distance_m) * tangent_x +
            (semantic_to_border_y / test.semantic_to_border_distance_m) * tangent_y);
        }

        const double normal_x = -segment_dy / segment_length;
        const double normal_y = segment_dx / segment_length;
        bool found_unambiguous_side = false;
        for (const double probe_distance_m : kRoadBorderSideProbeDistancesM) {
          test.plus_sample = autoware_utils_geometry::Point2d{
            test.closest_point.x() + normal_x * probe_distance_m,
            test.closest_point.y() + normal_y * probe_distance_m};
          test.minus_sample = autoware_utils_geometry::Point2d{
            test.closest_point.x() - normal_x * probe_distance_m,
            test.closest_point.y() - normal_y * probe_distance_m};
          test.plus_sample_drivable = point_in_semantic_drivable_area(
            test.plus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
            hatched_road_markings, parking_lots);
          test.minus_sample_drivable = point_in_semantic_drivable_area(
            test.minus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
            hatched_road_markings, parking_lots);
          test.plus_sample_semantic_distance_m =
            test.plus_sample_drivable
              ? 0.0
              : distance_to_semantic_drivable_area(
                  test.plus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
                  hatched_road_markings, parking_lots);
          test.minus_sample_semantic_distance_m =
            test.minus_sample_drivable
              ? 0.0
              : distance_to_semantic_drivable_area(
                  test.minus_sample, road_lanelets, shoulder_lanelets, intersection_areas,
                  hatched_road_markings, parking_lots);
          if (test.plus_sample_drivable != test.minus_sample_drivable) {
            found_unambiguous_side = true;
            break;
          }
        }

        if (found_unambiguous_side) {
          const auto road_side_sample = test.plus_sample_drivable ? test.plus_sample : test.minus_sample;
          const double corner_side =
            (test.corner.x() - test.closest_point.x()) * normal_x +
            (test.corner.y() - test.closest_point.y()) * normal_y;
          const double road_side =
            (road_side_sample.x() - test.closest_point.x()) * normal_x +
            (road_side_sample.y() - test.closest_point.y()) * normal_y;
          const bool same_road_side = corner_side * road_side > kRoadBorderSideEpsilon;
          const bool bounded_gap =
            test.corner_between_ratio >= -0.05 && test.corner_between_ratio <= 1.05 &&
            test.corner_to_semantic_border_line_m <= kRoadBorderBetweenToleranceM;
          const bool across_border =
            test.border_tangent_alignment <= kRoadBorderMaxTangentAlignment;
          test.accepted =
            same_road_side && bounded_gap && across_border &&
            test.distance_m <= kRoadBorderMaxGapM &&
            test.semantic_to_border_distance_m <= kRoadBorderMaxSemanticToBorderM;
        }

        if (
          !best_rejected_test.has_value() ||
          test.distance_m < best_rejected_test->distance_m) {
          best_rejected_test = test;
        }
        if (
          test.accepted &&
          (!best_accepted_test.has_value() || test.distance_m < best_accepted_test->distance_m)) {
          best_accepted_test = test;
        }
      }
    }

    if (best_accepted_test.has_value()) {
      corner_drivable_by_road_border.at(index) = true;
      side_tests.push_back(*best_accepted_test);
    } else if (best_rejected_test.has_value()) {
      side_tests.push_back(*best_rejected_test);
    }
  }
  return corner_drivable_by_road_border;
}

bool point_within_lanelet_margin(
  const autoware_utils_geometry::Point2d & point, const lanelet::ConstLanelet & lanelet,
  const double margin_m)
{
  namespace bg = boost::geometry;
  const auto polygon = to_polygon_2d(lanelet.polygon2d().basicPolygon());
  return bg::distance(point, polygon) <= margin_m;
}

lanelet::ConstLanelets collect_local_route_consistent_lanelets(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  lanelet::ConstLanelets local_lanelets;
  if (!route_handler || !route_handler->isHandlerReady()) {
    return local_lanelets;
  }

  const auto map = route_handler->getLaneletMapPtr();
  const auto nearby_bbox = point_bounding_box(pose.position);
  std::unordered_set<lanelet::Id> nearby_road_lanelet_ids;
  std::unordered_set<lanelet::Id> nearby_shoulder_lanelet_ids;
  lanelet::ConstLanelets nearby_road_lanelets;
  lanelet::ConstLanelets nearby_shoulder_lanelets;
  for (const auto & lanelet : map->laneletLayer.search(nearby_bbox)) {
    if (route_handler->isRoadLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_road_lanelets, nearby_road_lanelet_ids);
    } else if (route_handler->isShoulderLanelet(lanelet)) {
      append_unique_lanelet(lanelet, nearby_shoulder_lanelets, nearby_shoulder_lanelet_ids);
    }
  }

  lanelet::ConstLanelets seed_lanelets;
  std::unordered_set<lanelet::Id> seed_ids;
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (route_handler->isRouteLanelet(lanelet)) {
      append_unique_lanelet(lanelet, seed_lanelets, seed_ids);
    }
  }
  lanelet::ConstLanelet closest_route_lanelet;
  if (route_handler->getClosestLaneletWithinRoute(pose, &closest_route_lanelet)) {
    append_unique_lanelet(closest_route_lanelet, seed_lanelets, seed_ids);
  }

  if (seed_lanelets.empty()) {
    return local_lanelets;
  }

  const double reference_yaw = lanelet::utils::getLaneletAngle(seed_lanelets.front(), pose.position);
  std::unordered_set<lanelet::Id> local_lanelet_ids;
  for (const auto & lanelet : nearby_road_lanelets) {
    const bool on_route = route_handler->isRouteLanelet(lanelet);
    const double lanelet_yaw = lanelet::utils::getLaneletAngle(lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(lanelet_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!on_route && !same_direction) {
      continue;
    }
    append_unique_lanelet(lanelet, local_lanelets, local_lanelet_ids);
  }

  for (const auto & shoulder_lanelet : nearby_shoulder_lanelets) {
    const double shoulder_yaw = lanelet::utils::getLaneletAngle(shoulder_lanelet, pose.position);
    const bool same_direction =
      std::abs(normalize_angle(shoulder_yaw - reference_yaw)) <= kDirectionSimilarityThresholdRad;
    if (!same_direction) {
      continue;
    }
    append_unique_lanelet(shoulder_lanelet, local_lanelets, local_lanelet_ids);
  }

  return local_lanelets;
}

std::vector<lanelet::ConstPolygon3d> collect_local_intersection_areas(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<lanelet::ConstPolygon3d> intersection_areas;
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return intersection_areas;
  }

  const auto map = route_handler->getLaneletMapPtr();
  std::unordered_set<lanelet::Id> seen_ids;
  for (const auto & polygon : map->polygonLayer.search(point_bounding_box(pose.position))) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "intersection_area") {
      continue;
    }
    append_unique_polygon(polygon, intersection_areas, seen_ids);
  }

  return intersection_areas;
}

bool detect_multiple_lanes(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets)
{
  if (footprint_points.empty() || road_lanelets.empty()) {
    return false;
  }

  std::size_t occupied_lanelets = 0;
  bool single_lane_contains_all_points = false;
  for (const auto & lanelet : road_lanelets) {
    std::size_t contained_points = 0;
    for (const auto & point : footprint_points) {
      if (point_in_lanelet(point, lanelet)) {
        ++contained_points;
      }
    }

    if (contained_points > 0U) {
      ++occupied_lanelets;
    }
    if (contained_points == footprint_points.size()) {
      single_lane_contains_all_points = true;
    }
  }

  return occupied_lanelets > 1U && !single_lane_contains_all_points;
}

std::vector<bool> evaluate_corner_drivable(
  const std::vector<autoware_utils_geometry::Point2d> & footprint_points,
  const lanelet::ConstLanelets & road_lanelets, const lanelet::ConstLanelets & shoulder_lanelets,
  const std::vector<lanelet::ConstPolygon3d> & intersection_areas,
  const std::vector<lanelet::ConstPolygon3d> & hatched_road_markings,
  const std::vector<lanelet::ConstPolygon3d> & parking_lots)
{
  std::vector<bool> corner_drivable(footprint_points.size(), false);

  for (std::size_t index = 0; index < footprint_points.size(); ++index) {
    const auto & point = footprint_points.at(index);
    for (const auto & lanelet : road_lanelets) {
      if (point_in_lanelet(point, lanelet)) {
        corner_drivable.at(index) = true;
        break;
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & lanelet : shoulder_lanelets) {
        if (point_in_lanelet(point, lanelet)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & intersection_area : intersection_areas) {
        if (point_in_polygon(point, intersection_area)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & hatched_road_marking : hatched_road_markings) {
        if (point_in_polygon(point, hatched_road_marking)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
    if (!corner_drivable.at(index)) {
      for (const auto & parking_lot : parking_lots) {
        if (point_in_polygon(point, parking_lot)) {
          corner_drivable.at(index) = true;
          break;
        }
      }
    }
  }

  return corner_drivable;
}

double planar_speed_mps(const geometry_msgs::msg::Twist & twist)
{
  return std::hypot(twist.linear.x, twist.linear.y);
}

double planar_displacement_m(
  const geometry_msgs::msg::Pose & lhs, const geometry_msgs::msg::Pose & rhs)
{
  return autoware_utils_geometry::calc_distance2d(lhs.position, rhs.position);
}

bool should_hold_long_object_yaw(
  const LoggedObjectState & previous_state, const LoggedObjectState & current_state,
  const double previous_yaw, const double current_yaw)
{
  constexpr double kLongObjectLengthThresholdM = 8.0;
  constexpr double kSlowObjectSpeedThresholdMps = 2.0;
  constexpr double kSmallDisplacementThresholdM = 0.75;
  constexpr double kLargeYawJumpThresholdRad = 10.0 * M_PI / 180.0;

  const double object_length = std::max(previous_state.shape.dimensions.x, current_state.shape.dimensions.x);
  if (object_length < kLongObjectLengthThresholdM) {
    return false;
  }

  const double max_speed =
    std::max(planar_speed_mps(previous_state.twist), planar_speed_mps(current_state.twist));
  if (max_speed > kSlowObjectSpeedThresholdMps) {
    return false;
  }

  const double displacement = planar_displacement_m(previous_state.pose, current_state.pose);
  if (displacement > kSmallDisplacementThresholdM) {
    return false;
  }

  const double yaw_delta = std::abs(normalize_angle(current_yaw - previous_yaw));
  return yaw_delta > kLargeYawJumpThresholdRad;
}

void canonicalize_bounding_box_yaws(LoggedObjectTrack & track)
{
  if (track.states.size() < 2U) {
    return;
  }

  bool has_reference_yaw = false;
  double reference_yaw = 0.0;
  const LoggedObjectState * previous_state = nullptr;
  for (auto & state : track.states) {
    if (state.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
      has_reference_yaw = false;
      previous_state = nullptr;
      continue;
    }

    const double raw_yaw = get_yaw(state.pose.orientation);
    double canonical_yaw =
      has_reference_yaw ? closest_pi_symmetric_yaw(reference_yaw, raw_yaw) : raw_yaw;
    if (
      has_reference_yaw && previous_state &&
      should_hold_long_object_yaw(*previous_state, state, reference_yaw, canonical_yaw)) {
      canonical_yaw = reference_yaw;
    }
    state.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(canonical_yaw);
    reference_yaw = canonical_yaw;
    has_reference_yaw = true;
    previous_state = &state;
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

std::optional<EgoAreaEvaluation> compute_ego_area_evaluation(
  const geometry_msgs::msg::Pose & pose, const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & designated_lanelets,
  const bool evaluate_intersection_context)
{
  if (!route_handler) {
    return std::nullopt;
  }
  if (!route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  auto road_lanelets = collect_candidate_road_lanelets(ego_polygon, route_handler, designated_lanelets);
  auto shoulder_lanelets = collect_candidate_shoulder_lanelets(ego_polygon, route_handler);
  for (const auto & lanelet : route_handler->getRoadLaneletsAtPose(pose)) {
    if (
      route_handler->isRoadLanelet(lanelet) && footprint_intersects_lanelet(ego_polygon, lanelet)) {
      const auto duplicate = std::any_of(
        road_lanelets.begin(), road_lanelets.end(),
        [&lanelet](const auto & candidate) { return candidate.id() == lanelet.id(); });
      if (!duplicate) {
        road_lanelets.push_back(lanelet);
      }
    }
  }
  const auto intersection_areas =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"intersection_area"});
  const auto hatched_road_markings =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"hatched_road_markings"});
  const auto parking_lots =
    collect_candidate_map_polygons(ego_polygon, route_handler, {"parking_lot"});
  auto road_border_lines = collect_candidate_road_border_lines(ego_polygon, route_handler);
  const auto points = footprint_vertices(ego_polygon);
  const auto semantic_corner_drivable = evaluate_corner_drivable(
    points, road_lanelets, shoulder_lanelets, intersection_areas, hatched_road_markings,
    parking_lots);
  auto corner_drivable = semantic_corner_drivable;
  std::vector<RoadBorderSideTest> road_border_side_tests;
  auto corner_drivable_by_road_border = evaluate_road_border_side_fallback(
    points, semantic_corner_drivable, road_lanelets, shoulder_lanelets, intersection_areas,
    hatched_road_markings, parking_lots, road_border_lines, road_border_side_tests);
  for (std::size_t index = 0; index < corner_drivable.size(); ++index) {
    if (!corner_drivable.at(index) && corner_drivable_by_road_border.at(index)) {
      corner_drivable.at(index) = true;
    }
  }

  EgoAreaEvaluation evaluation;
  evaluation.flags.multiple_lanes = detect_multiple_lanes(points, road_lanelets);
  evaluation.flags.non_drivable_area = std::any_of(
    corner_drivable.begin(), corner_drivable.end(), [](const bool value) { return !value; });
  if (evaluate_intersection_context) {
    const auto intersection_context = compute_driving_direction_local_context(pose, route_handler);
    evaluation.flags.intersection_context_available = intersection_context.has_value();
    evaluation.flags.in_intersection =
      intersection_context.has_value() && intersection_context->in_intersection;
  }
  evaluation.flags.road_border_fallback_used = false;
  for (std::size_t index = 0; index < corner_drivable.size(); ++index) {
    if (!semantic_corner_drivable.at(index) && corner_drivable_by_road_border.at(index)) {
      evaluation.flags.road_border_fallback_used = true;
      break;
    }
  }
  evaluation.flags.inside_road_border_envelope =
    std::all_of(corner_drivable.begin(), corner_drivable.end(), [](const bool value) { return value; });
  evaluation.footprint_points = points;
  evaluation.corner_drivable = corner_drivable;
  evaluation.corner_drivable_by_road_border = std::move(corner_drivable_by_road_border);
  evaluation.road_border_side_tests = std::move(road_border_side_tests);
  evaluation.road_lanelets = std::move(road_lanelets);
  evaluation.shoulder_lanelets = std::move(shoulder_lanelets);
  evaluation.intersection_areas = std::move(intersection_areas);
  evaluation.hatched_road_markings = std::move(hatched_road_markings);
  evaluation.parking_lots = std::move(parking_lots);
  evaluation.road_border_lines = std::move(road_border_lines);
  evaluation.designated_lanelet_count = designated_lanelets.size();
  return evaluation;
}

std::vector<TrajectoryFootprintEvaluation> evaluate_trajectory_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<RouteHandler> & route_handler,
  const lanelet::ConstLanelets * route_relevant_lanelets,
  const bool evaluate_intersection_context)
{
  std::vector<TrajectoryFootprintEvaluation> evaluations;
  if (!is_vehicle_info_valid(vehicle_info)) {
    return evaluations;
  }

  const auto local_footprint = vehicle_info.createFootprint(0.0);
  if (local_footprint.empty()) {
    return evaluations;
  }

  const auto local_designated_lanelets =
    route_relevant_lanelets == nullptr && route_handler && route_handler->isHandlerReady()
      ? collect_route_relevant_lanelets(trajectory, route_handler)
      : lanelet::ConstLanelets{};
  const auto & designated_lanelets =
    route_relevant_lanelets != nullptr ? *route_relevant_lanelets : local_designated_lanelets;

  evaluations.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    TrajectoryFootprintEvaluation evaluation;
    evaluation.ego_polygon = create_pose_footprint(point.pose, local_footprint);
    if (route_handler) {
      evaluation.ego_area_evaluation = compute_ego_area_evaluation(
        point.pose, evaluation.ego_polygon, route_handler, designated_lanelets,
        evaluate_intersection_context);
    }
    evaluations.push_back(std::move(evaluation));
  }

  return evaluations;
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
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_intersection;
}

bool is_pose_in_route_lane_polygon(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  const auto context = compute_driving_direction_local_context(pose, route_handler);
  return context.has_value() && context->in_route_lane_polygon;
}

std::optional<DrivingDirectionLocalContext> compute_driving_direction_local_context(
  const geometry_msgs::msg::Pose & pose, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isMapMsgReady()) {
    return std::nullopt;
  }

  const autoware_utils_geometry::Point2d search_point{pose.position.x, pose.position.y};
  DrivingDirectionLocalContext context;
  context.route_lanelets = collect_local_route_consistent_lanelets(pose, route_handler);
  context.intersection_areas = collect_local_intersection_areas(pose, route_handler);

  const bool in_route_lane_polygon_exact = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(),
    [&search_point](const auto & lanelet) { return point_in_lanelet(search_point, lanelet); });
  const bool in_route_lane_polygon_margin = std::any_of(
    context.route_lanelets.begin(), context.route_lanelets.end(), [&search_point](const auto & lanelet) {
      return point_within_lanelet_margin(search_point, lanelet, kAdmissibleLaneMarginM);
    });
  context.in_route_lane_polygon = in_route_lane_polygon_exact || in_route_lane_polygon_margin;
  context.in_lane_margin_only = !in_route_lane_polygon_exact && in_route_lane_polygon_margin;
  context.in_intersection = std::any_of(
    context.intersection_areas.begin(), context.intersection_areas.end(),
    [&search_point](const auto & polygon) { return point_in_polygon(search_point, polygon); });

  if (!context.in_intersection) {
    for (const auto & lanelet : context.route_lanelets) {
      if (
        autoware::experimental::lanelet2_utils::is_intersection_lanelet(lanelet) &&
        point_in_lanelet(search_point, lanelet)) {
        context.in_intersection = true;
        break;
      }
    }
  }
  return context;
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

bool is_unknown_classification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  return autoware::object_recognition_utils::getHighestProbLabel(classification) ==
         ObjectClassification::UNKNOWN;
}

std::vector<LoggedObjectTrack> build_logged_object_tracks(
  const std::vector<TimedTrackedObjects> & future_objects)
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
      state.pose = object.kinematics.pose_with_covariance.pose;
      state.twist = object.kinematics.twist_with_covariance.twist;
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
  object.pose =
    autoware_utils_geometry::calc_interpolated_pose(previous.pose, next.pose, ratio, false);
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
