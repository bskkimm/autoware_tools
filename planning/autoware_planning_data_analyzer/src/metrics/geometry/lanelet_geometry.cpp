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

#include "lanelet_geometry.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Utilities.h>

#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>

namespace autoware::planning_data_analyzer::metrics
{

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

void append_unique_line_string(
  const lanelet::ConstLineString3d & line_string,
  std::vector<lanelet::ConstLineString3d> & line_strings,
  std::unordered_set<lanelet::Id> & seen_ids)
{
  if (seen_ids.insert(line_string.id()).second) {
    line_strings.push_back(line_string);
  }
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

lanelet::BoundingBox2d make_bounding_box(
  const double min_x, const double min_y, const double max_x, const double max_y,
  const double margin_m)
{
  return lanelet::BoundingBox2d{
    lanelet::BasicPoint2d{min_x - margin_m, min_y - margin_m},
    lanelet::BasicPoint2d{max_x + margin_m, max_y + margin_m}};
}

double get_lanelet_angle(const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint3d & point)
{
  const auto & centerline = lanelet.centerline2d();
  if (centerline.size() < 2) return 0.0;

  double min_dist = std::numeric_limits<double>::max();
  double yaw = 0.0;
  const auto p2d = lanelet::utils::to2D(point);

  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    const auto & p1 = centerline[i];
    const auto & p2 = centerline[i + 1];
    const double dist = lanelet::geometry::distance2d(
      lanelet::BasicLineString2d{p1.basicPoint(), p2.basicPoint()}, p2d);
    if (dist < min_dist) {
      min_dist = dist;
      yaw = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
    }
  }
  return yaw;
}

double get_lateral_distance_to_centerline(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  const auto centerline = lanelet.centerline2d();
  const auto point =
    lanelet::utils::to2D(lanelet::BasicPoint3d(pose.position.x, pose.position.y, pose.position.z));
  return lanelet::geometry::signedDistance(centerline, point);
}

double get_arc_length(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & pose)
{
  if (lanelets.empty()) return 0.0;

  lanelet::BasicLineString2d combined_centerline;
  for (const auto & ll : lanelets) {
    for (const auto & p : ll.centerline2d()) {
      if (
        combined_centerline.empty() ||
        lanelet::geometry::distance2d(combined_centerline.back(), p.basicPoint()) > 1e-6) {
        combined_centerline.push_back(p.basicPoint());
      }
    }
  }

  if (combined_centerline.size() < 2) return 0.0;

  return lanelet::geometry::toArcCoordinates(
           combined_centerline,
           lanelet::utils::to2D(
             lanelet::BasicPoint3d(pose.position.x, pose.position.y, pose.position.z)))
    .length;
}

}  // namespace autoware::planning_data_analyzer::metrics
