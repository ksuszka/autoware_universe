// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "autoware/shape_estimation/model/convex_hull.hpp"

#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <vector>

namespace bg = boost::geometry;
using autoware_utils::Polygon2d;
using Point2d = autoware_utils::Point2d;

namespace autoware::shape_estimation
{
namespace model
{

bool ConvexHullShapeModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // calc centroid point for convex hull height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto & pcl_point : cluster) {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / static_cast<double>(cluster.size());
  centroid.y = centroid.y / static_cast<double>(cluster.size());
  centroid.z = centroid.z / static_cast<double>(cluster.size());

  // calc min and max z for convex hull height(z)
  float min_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  float max_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  for (const auto & point : cluster) {
    min_z = std::min(point.z, min_z);
    max_z = std::max(point.z, max_z);
  }

  Polygon2d bg_polygon;
  for (size_t i = 0; i < cluster.size(); ++i) {
    bg_polygon.outer().push_back(
      Point2d((cluster.at(i).x - centroid.x), (cluster.at(i).y - centroid.y)));
  }
  Polygon2d convex_polygon;
  bg::convex_hull(bg_polygon, convex_polygon);
  // Check if the polygon is clockwise, if not, inverse it since Autoware Universe uses
  // counter-clockwise
  if (autoware_utils::is_clockwise(convex_polygon)) {
    convex_polygon = autoware_utils::inverse_clockwise(convex_polygon);
  }
  Point2d polygon_centroid;
  bg::centroid(convex_polygon, polygon_centroid);

  for (size_t i = 0; i < convex_polygon.outer().size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<double>(convex_polygon.outer().at(i).x()) - polygon_centroid.x();
    point.y = static_cast<double>(convex_polygon.outer().at(i).y()) - polygon_centroid.y();
    point.z = 0.0;
    shape_output.footprint.points.push_back(point);
  }

  constexpr float ep = 0.001;
  shape_output.type = autoware_perception_msgs::msg::Shape::POLYGON;
  shape_output.dimensions.x = 0.0;
  shape_output.dimensions.y = 0.0;
  shape_output.dimensions.z = std::max((max_z - min_z), ep);
  pose_output.position.x = centroid.x + polygon_centroid.x();
  pose_output.position.y = centroid.y + polygon_centroid.y();
  pose_output.position.z = min_z + shape_output.dimensions.z * 0.5;
  pose_output.orientation.x = 0;
  pose_output.orientation.y = 0;
  pose_output.orientation.z = 0;
  pose_output.orientation.w = 1;
  return true;
}

}  // namespace model
}  // namespace autoware::shape_estimation
