// Copyright 2022 The Autoware Contributors
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

#include "objects_to_costmap_test_accessor.hpp"

#include <autoware/costmap_generator/utils/objects_to_costmap.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <cmath>
#include <memory>

namespace
{
geometry_msgs::msg::Point32 toPoint32(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point32 point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = pose.position.z;
  return point;
}
}  // namespace

namespace autoware::costmap_generator
{
using LABEL = autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

class ObjectsToCostMapTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  ~ObjectsToCostMapTest() override { rclcpp::shutdown(); }

  grid_map::GridMap construct_gridmap();

  [[nodiscard]] PredictedObject get_object(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 dimension,
    const bool box_type = true) const;
  [[nodiscard]] PredictedObject get_polygon_object_local_footprint(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 dimension) const;

public:
  double grid_resolution_ = 1;
  double grid_length_x_ = 21;
  double grid_length_y_ = 21;
  double grid_position_x_ = 0;
  double grid_position_y_ = 0;
};

grid_map::GridMap ObjectsToCostMapTest::construct_gridmap()
{
  grid_map::GridMap gm;

  // set gridmap size
  gm.setFrameId("map");
  gm.setGeometry(
    grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
    grid_map::Position(grid_position_x_, grid_position_y_));

  // set initial value
  gm.add("objects", 0.);

  // set car postion in map frame to center of grid
  grid_map::Position p;
  p.x() = 0;
  p.y() = 0;
  gm.setPosition(p);

  return gm;
}

PredictedObject ObjectsToCostMapTest::get_object(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 dimension,
  const bool box_type) const
{
  PredictedObject object;
  object.classification.push_back(LABEL{});
  object.classification.at(0).label = LABEL::CAR;
  object.classification.at(0).probability = 0.8;
  object.kinematics.initial_pose_with_covariance.pose = pose;
  object.shape.dimensions = dimension;
  if (box_type) {
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    return object;
  }

  object.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;

  object.shape.footprint.points.emplace_back(
    toPoint32(autoware_utils::calc_offset_pose(pose, -0.5 * dimension.x, -0.5 * dimension.y, 0.0)));

  object.shape.footprint.points.emplace_back(
    toPoint32(autoware_utils::calc_offset_pose(pose, -0.5 * dimension.x, 0.5 * dimension.y, 0.0)));

  object.shape.footprint.points.emplace_back(
    toPoint32(autoware_utils::calc_offset_pose(pose, 0.5 * dimension.x, 0.5 * dimension.y, 0.0)));

  object.shape.footprint.points.emplace_back(
    toPoint32(autoware_utils::calc_offset_pose(pose, 0.5 * dimension.x, -0.5 * dimension.y, 0.0)));

  return object;
}

PredictedObject ObjectsToCostMapTest::get_polygon_object_local_footprint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 dimension) const
{
  PredictedObject object;
  object.classification.push_back(LABEL{});
  object.classification.at(0).label = LABEL::CAR;
  object.classification.at(0).probability = 0.8;
  object.kinematics.initial_pose_with_covariance.pose = pose;
  object.shape.dimensions = dimension;
  object.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;

  const auto half_x = static_cast<float>(0.5 * dimension.x);
  const auto half_y = static_cast<float>(0.5 * dimension.y);

  const auto make_point32 = [](float x, float y, float z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  object.shape.footprint.points.emplace_back(make_point32(-half_x, -half_y, 0.0F));
  object.shape.footprint.points.emplace_back(make_point32(-half_x, half_y, 0.0F));
  object.shape.footprint.points.emplace_back(make_point32(half_x, half_y, 0.0F));
  object.shape.footprint.points.emplace_back(make_point32(half_x, -half_y, 0.0F));

  return object;
}

/*
grid_y
|
|     map_x---
|            |
|            |
|            map_y
|--------grid_x
*/
TEST_F(ObjectsToCostMapTest, TestMakeCostmapFromObjects_BoxType)
{
  auto objs = std::make_shared<PredictedObjects>();

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 1;
  obj_pose.position.y = 2;
  obj_pose.position.z = 1;
  obj_pose.orientation.x = 0;
  obj_pose.orientation.y = 0;
  obj_pose.orientation.z = 0;
  obj_pose.orientation.w = 1;

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 5;
  dimension.y = 3;
  dimension.z = 2;

  const auto object = get_object(obj_pose, dimension);
  objs->objects.push_back(object);

  grid_map::GridMap gridmap = construct_gridmap();
  ObjectsToCostmap objectsToCostmap;

  const double expand_polygon_size = 0.0;
  const double size_of_expansion_kernel = 1;  // do not expand for easy test check
  grid_map::Matrix objects_costmap = objectsToCostmap.makeCostmapFromObjects(
    gridmap, expand_polygon_size, size_of_expansion_kernel,
    ObjectsToCostmap::ObjectCostMode::ClassificationProbability, 1.0, {}, objs);

  // yaw = 0,so we can just calculate like this easily
  int expected_non_empty_cost_grid_num =
    (object.shape.dimensions.x * object.shape.dimensions.y) / grid_resolution_;

  // check if cost is correct
  int non_empty_cost_grid_num = 0;
  for (int i = 0; i < objects_costmap.rows(); i++) {
    for (int j = 0; j < objects_costmap.cols(); j++) {
      if (objects_costmap(i, j) == object.classification.at(0).probability) {
        non_empty_cost_grid_num += 1;
      }
    }
  }

  EXPECT_EQ(non_empty_cost_grid_num, expected_non_empty_cost_grid_num);

  float obj_center_x =
    grid_length_x_ / 2 - object.kinematics.initial_pose_with_covariance.pose.position.x;
  float obj_center_y =
    grid_length_y_ / 2 - object.kinematics.initial_pose_with_covariance.pose.position.y;
  float obj_left_x = obj_center_x - object.shape.dimensions.x / 2.;
  float obj_right_x = obj_center_x + object.shape.dimensions.x / 2.;
  float obj_bottom_y = obj_center_y - object.shape.dimensions.y / 2.;
  float obj_top_y = obj_center_y + object.shape.dimensions.y / 2.;
  int index_x_min = static_cast<int>(obj_left_x / grid_resolution_);
  int index_x_max = static_cast<int>(obj_right_x / grid_resolution_);
  int index_y_min = static_cast<int>(obj_bottom_y / grid_resolution_);
  int index_y_max = static_cast<int>(obj_top_y / grid_resolution_);
  for (int i = index_x_min; i < index_x_max; i++) {
    for (int j = index_y_min; j < index_y_max; j++) {
      EXPECT_DOUBLE_EQ(objects_costmap(i, j), object.classification.at(0).probability);
    }
  }
}

TEST_F(ObjectsToCostMapTest, TestMakeCostmapFromObjects_PolygonType)
{
  auto objs = std::make_shared<PredictedObjects>();

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 1;
  obj_pose.position.y = 2;
  obj_pose.position.z = 1;
  obj_pose.orientation.x = 0;
  obj_pose.orientation.y = 0;
  obj_pose.orientation.z = 0;
  obj_pose.orientation.w = 1;

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 5;
  dimension.y = 3;
  dimension.z = 2;

  const auto object = get_polygon_object_local_footprint(obj_pose, dimension);
  objs->objects.push_back(object);

  grid_map::GridMap gridmap = construct_gridmap();
  ObjectsToCostmap objectsToCostmap;

  const double expand_polygon_size = 0.0;
  const double size_of_expansion_kernel = 1;  // do not expand for easy test check
  grid_map::Matrix objects_costmap = objectsToCostmap.makeCostmapFromObjects(
    gridmap, expand_polygon_size, size_of_expansion_kernel,
    ObjectsToCostmap::ObjectCostMode::ClassificationProbability, 1.0, {}, objs);

  // yaw = 0,so we can just calculate like this easily
  int expected_non_empty_cost_grid_num =
    (object.shape.dimensions.x * object.shape.dimensions.y) / grid_resolution_;

  // check if cost is correct
  int non_empty_cost_grid_num = 0;
  for (int i = 0; i < objects_costmap.rows(); i++) {
    for (int j = 0; j < objects_costmap.cols(); j++) {
      if (objects_costmap(i, j) == object.classification.at(0).probability) {
        non_empty_cost_grid_num += 1;
      }
    }
  }

  EXPECT_EQ(non_empty_cost_grid_num, expected_non_empty_cost_grid_num);

  float obj_center_x =
    grid_length_x_ / 2 - object.kinematics.initial_pose_with_covariance.pose.position.x;
  float obj_center_y =
    grid_length_y_ / 2 - object.kinematics.initial_pose_with_covariance.pose.position.y;
  float obj_left_x = obj_center_x - object.shape.dimensions.x / 2.;
  float obj_right_x = obj_center_x + object.shape.dimensions.x / 2.;
  float obj_bottom_y = obj_center_y - object.shape.dimensions.y / 2.;
  float obj_top_y = obj_center_y + object.shape.dimensions.y / 2.;
  int index_x_min = static_cast<int>(obj_left_x / grid_resolution_);
  int index_x_max = static_cast<int>(obj_right_x / grid_resolution_);
  int index_y_min = static_cast<int>(obj_bottom_y / grid_resolution_);
  int index_y_max = static_cast<int>(obj_top_y / grid_resolution_);
  for (int i = index_x_min; i < index_x_max; i++) {
    for (int j = index_y_min; j < index_y_max; j++) {
      EXPECT_DOUBLE_EQ(objects_costmap(i, j), object.classification.at(0).probability);
    }
  }
}

TEST_F(ObjectsToCostMapTest, TestMakeCostmapFromObjects_PolygonTypeYaw90Rotation)
{
  auto objs = std::make_shared<PredictedObjects>();
  objs->header.frame_id = "map";

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 0.0;
  obj_pose.position.y = 0.0;
  obj_pose.position.z = 0.0;
  const double yaw = std::acos(-1.0) / 2.0;
  obj_pose.orientation.x = 0.0;
  obj_pose.orientation.y = 0.0;
  obj_pose.orientation.z = std::sin(yaw * 0.5);
  obj_pose.orientation.w = std::cos(yaw * 0.5);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 6.0;
  dimension.y = 1.0;
  dimension.z = 2.0;

  const auto object = get_polygon_object_local_footprint(obj_pose, dimension);
  objs->objects.push_back(object);

  grid_map::GridMap gridmap = construct_gridmap();
  ObjectsToCostmap objectsToCostmap;

  const double expand_polygon_size = 0.0;
  const double size_of_expansion_kernel = 1;  // do not expand for easy test check
  grid_map::Matrix objects_costmap = objectsToCostmap.makeCostmapFromObjects(
    gridmap, expand_polygon_size, size_of_expansion_kernel,
    ObjectsToCostmap::ObjectCostMode::ClassificationProbability, 1.0, {}, objs);

  grid_map::Index inside_index;
  ASSERT_TRUE(gridmap.getIndex(grid_map::Position(0.0, 2.0), inside_index));
  EXPECT_DOUBLE_EQ(
    objects_costmap(inside_index.x(), inside_index.y()), object.classification.at(0).probability);

  grid_map::Index outside_index;
  ASSERT_TRUE(gridmap.getIndex(grid_map::Position(2.0, 0.0), outside_index));
  EXPECT_DOUBLE_EQ(objects_costmap(outside_index.x(), outside_index.y()), 0.0);
}

TEST_F(ObjectsToCostMapTest, TestMakeCostmapFromObjects_PolygonTypeYaw45Rotation)
{
  auto objs = std::make_shared<PredictedObjects>();
  objs->header.frame_id = "map";

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = 0.0;
  obj_pose.position.y = 0.0;
  obj_pose.position.z = 0.0;
  const double yaw = std::acos(-1.0) / 4.0;
  obj_pose.orientation.x = 0.0;
  obj_pose.orientation.y = 0.0;
  obj_pose.orientation.z = std::sin(yaw * 0.5);
  obj_pose.orientation.w = std::cos(yaw * 0.5);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 6.0;
  dimension.y = 2.0;
  dimension.z = 2.0;

  const auto object = get_polygon_object_local_footprint(obj_pose, dimension);
  objs->objects.push_back(object);

  grid_map::GridMap gridmap = construct_gridmap();
  ObjectsToCostmap objectsToCostmap;

  const double expand_polygon_size = 0.0;
  const double size_of_expansion_kernel = 1;  // do not expand for easy test check
  grid_map::Matrix objects_costmap = objectsToCostmap.makeCostmapFromObjects(
    gridmap, expand_polygon_size, size_of_expansion_kernel,
    ObjectsToCostmap::ObjectCostMode::ClassificationProbability, 1.0, {}, objs);

  grid_map::Index inside_index;
  ASSERT_TRUE(gridmap.getIndex(grid_map::Position(1.0, 2.0), inside_index));
  EXPECT_DOUBLE_EQ(
    objects_costmap(inside_index.x(), inside_index.y()), object.classification.at(0).probability);

  grid_map::Index outside_index;
  ASSERT_TRUE(gridmap.getIndex(grid_map::Position(2.0, 0.0), outside_index));
  EXPECT_DOUBLE_EQ(objects_costmap(outside_index.x(), outside_index.y()), 0.0);
}

TEST_F(ObjectsToCostMapTest, TestMakePolygonFromObjectConvexHull_WithExpansion)
{
  // Test polygon object with expansion using expandPolygonUniform
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 2.0;
  dimension.y = 2.0;
  dimension.z = 1.0;

  auto object = get_polygon_object_local_footprint(pose, dimension);

  std_msgs::msg::Header header;
  header.frame_id = "map";

  ObjectsToCostmap obj2costmap;
  autoware::costmap_generator::test::ObjectsToCostmapTestAccessor accessor(obj2costmap);

  // Test with no expansion
  const double no_expansion = 0.0;
  auto polygon_no_expansion =
    accessor.makePolygonFromObjectConvexHull(header, object, no_expansion);

  // Test with expansion
  const double expansion_size = 0.5;
  auto polygon_with_expansion =
    accessor.makePolygonFromObjectConvexHull(header, object, expansion_size);

  // The expanded polygon should have more or equal vertices (due to boost::buffer rounding)
  EXPECT_GE(polygon_with_expansion.nVertices(), polygon_no_expansion.nVertices());

  // Calculate max distance from center for both polygons
  double max_dist_original = 0.0;
  for (size_t i = 0; i < polygon_no_expansion.nVertices(); ++i) {
    const auto & vertex = polygon_no_expansion.getVertex(i);
    double dist = std::hypot(vertex.x(), vertex.y());
    max_dist_original = std::max(max_dist_original, dist);
  }

  double max_dist_expanded = 0.0;
  for (size_t i = 0; i < polygon_with_expansion.nVertices(); ++i) {
    const auto & vertex = polygon_with_expansion.getVertex(i);
    double dist = std::hypot(vertex.x(), vertex.y());
    max_dist_expanded = std::max(max_dist_expanded, dist);
  }

  // Expanded polygon should have vertices further from center
  EXPECT_GT(max_dist_expanded, max_dist_original);
  EXPECT_NEAR(max_dist_expanded, max_dist_original + expansion_size, 0.0001);
}

TEST_F(ObjectsToCostMapTest, TestMakePolygonFromObjectConvexHull_NoExpansionWhenZero)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_4);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 4.0;
  dimension.y = 2.0;
  dimension.z = 1.5;

  auto object = get_polygon_object_local_footprint(pose, dimension);

  std_msgs::msg::Header header;
  header.frame_id = "map";

  ObjectsToCostmap obj2costmap;
  autoware::costmap_generator::test::ObjectsToCostmapTestAccessor accessor(obj2costmap);

  // Test that zero expansion doesn't modify the polygon
  const double zero_expansion = 0.0;
  auto polygon = accessor.makePolygonFromObjectConvexHull(header, object, zero_expansion);

  // Should have 5 vertices (corners of the footprint)
  EXPECT_EQ(polygon.nVertices(), 5);

  // Check that polygon is centered around the object position
  // Calculate centroid correctly by excluding the duplicate closing vertex if present
  double sum_x = 0.0, sum_y = 0.0;
  size_t count = polygon.nVertices();

  // Check if last vertex is duplicate of first (polygon closing point)
  if (count > 1) {
    const auto & first = polygon.getVertex(0);
    const auto & last = polygon.getVertex(count - 1);
    const double epsilon = 1e-6;
    if (std::abs(first.x() - last.x()) < epsilon && std::abs(first.y() - last.y()) < epsilon) {
      count--;  // Exclude duplicate closing vertex from centroid calculation
    }
  }

  for (size_t i = 0; i < count; ++i) {
    const auto & vertex = polygon.getVertex(i);
    sum_x += vertex.x();
    sum_y += vertex.y();
  }
  double center_x = sum_x / count;
  double center_y = sum_y / count;

  EXPECT_NEAR(center_x, pose.position.x, 0.1);
  EXPECT_NEAR(center_y, pose.position.y, 0.1);
}

TEST_F(ObjectsToCostMapTest, TestMakePolygonFromObjectConvexHull_LargeExpansion)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 1.0;
  dimension.y = 1.0;
  dimension.z = 1.0;

  auto object = get_polygon_object_local_footprint(pose, dimension);

  std_msgs::msg::Header header;
  header.frame_id = "map";

  ObjectsToCostmap obj2costmap;
  autoware::costmap_generator::test::ObjectsToCostmapTestAccessor accessor(obj2costmap);

  // Test with large expansion
  const double large_expansion = 2.0;
  auto polygon = accessor.makePolygonFromObjectConvexHull(header, object, large_expansion);

  // Should create a valid polygon
  EXPECT_GT(polygon.nVertices(), 0);

  // Check that all vertices are at significant distance from origin
  for (size_t i = 0; i < polygon.nVertices(); ++i) {
    const auto & vertex = polygon.getVertex(i);
    double dist = std::hypot(vertex.x(), vertex.y());
    // With original size ~0.7 (half diagonal of 1x1) + 2.0 expansion
    EXPECT_GT(dist, 1.0);
  }
}

TEST_F(ObjectsToCostMapTest, TestExpandPolygonUniform_InMakeCostmapFromObjects)
{
  // Integration test: verify that expandPolygonUniform is used correctly in full pipeline
  grid_map::GridMap gridmap = construct_gridmap();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 2.0;
  dimension.y = 2.0;
  dimension.z = 1.0;

  auto object = get_polygon_object_local_footprint(pose, dimension);

  auto objects = std::make_shared<PredictedObjects>();
  objects->header.frame_id = "map";
  objects->objects.push_back(object);

  ObjectsToCostmap obj2costmap;

  // Test with expansion
  const double expand_polygon_size = 2.0;
  const int64_t size_of_expansion_kernel = 0;

  const auto costmap_data = obj2costmap.makeCostmapFromObjects(
    gridmap, expand_polygon_size, size_of_expansion_kernel,
    ObjectsToCostmap::ObjectCostMode::ClassificationProbability, 1.0, {}, objects);

  // Count non-zero cells
  int occupied_cells = 0;
  for (int i = 0; i < costmap_data.rows(); ++i) {
    for (int j = 0; j < costmap_data.cols(); ++j) {
      if (costmap_data(i, j) > 0.0) {
        occupied_cells++;
      }
    }
  }

  // With expansion, should have more occupied cells than without
  EXPECT_GT(occupied_cells, 16);
  EXPECT_LT(occupied_cells, 30);  // Sanity check
}

TEST_F(ObjectsToCostMapTest, TestMakePolygonFromObjectConvexHull_NoExpansion_NotEmpty)
{
  // Regression test: ensure polygon with expand_polygon_size=0 is not empty
  // Tests the fix for boost::geometry::correct() being called when offset=0
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5.0;
  pose.position.y = 5.0;
  pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Vector3 dimension;
  dimension.x = 2.0;
  dimension.y = 2.0;
  dimension.z = 1.0;

  auto object = get_polygon_object_local_footprint(pose, dimension);

  std_msgs::msg::Header header;
  header.frame_id = "map";

  ObjectsToCostmap obj2costmap;
  autoware::costmap_generator::test::ObjectsToCostmapTestAccessor accessor(obj2costmap);

  // Test with NO expansion (expand_polygon_size = 0.0)
  const double no_expansion = 0.0;
  auto polygon = accessor.makePolygonFromObjectConvexHull(header, object, no_expansion);

  // Polygon should not be empty - it should have vertices from the footprint
  EXPECT_EQ(polygon.nVertices(), 5);  // exact 5 vertices for valid polygon

  // Verify polygon is properly formed and positioned around the object

  for (size_t i = 0; i < polygon.nVertices(); ++i) {
    const auto & vertex = polygon.getVertex(i);
    double dist_to_center = std::hypot(vertex.x() - pose.position.x, vertex.y() - pose.position.y);
    // Vertices should be exactly sqrt(2) from the center.
    if (dist_to_center < 2.0) {
      EXPECT_NEAR(dist_to_center, std::sqrt(2), 1e-6);
    }
  }
}
}  // namespace autoware::costmap_generator

// ─── Tests for labelFromString ────────────────────────────────────────────────

namespace autoware::costmap_generator
{
using LABEL = autoware_perception_msgs::msg::ObjectClassification;

TEST(LabelFromStringTest, KnownLabels)
{
  EXPECT_EQ(ObjectsToCostmap::labelFromString("unknown"), LABEL::UNKNOWN);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("car"), LABEL::CAR);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("truck"), LABEL::TRUCK);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("bus"), LABEL::BUS);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("trailer"), LABEL::TRAILER);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("motorcycle"), LABEL::MOTORCYCLE);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("bicycle"), LABEL::BICYCLE);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("pedestrian"), LABEL::PEDESTRIAN);
}

TEST(LabelFromStringTest, UnknownStringFallsBackToUnknown)
{
  EXPECT_EQ(ObjectsToCostmap::labelFromString(""), LABEL::UNKNOWN);
  EXPECT_EQ(ObjectsToCostmap::labelFromString("CAR"), LABEL::UNKNOWN);  // case-sensitive
  EXPECT_EQ(ObjectsToCostmap::labelFromString("not_a_type"), LABEL::UNKNOWN);
}

// ─── Tests for object-label filtering in makeCostmapFromObjects ───────────────

class ObjectLabelFilterTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  ~ObjectLabelFilterTest() override { rclcpp::shutdown(); }

  grid_map::GridMap make_gridmap()
  {
    grid_map::GridMap gm;
    gm.setFrameId("map");
    gm.setGeometry(grid_map::Length(21.0, 21.0), 1.0, grid_map::Position(0.0, 0.0));
    gm.add("objects", 0.0);
    return gm;
  }

  // Returns a bounding-box object centred at (0,0) with given label.
  static autoware_perception_msgs::msg::PredictedObjects::SharedPtr make_objects(
    uint8_t label, float probability = 0.9f)
  {
    auto objs = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
    objs->header.frame_id = "map";

    autoware_perception_msgs::msg::PredictedObject obj;
    autoware_perception_msgs::msg::ObjectClassification cls;
    cls.label = label;
    cls.probability = probability;
    obj.classification.push_back(cls);
    obj.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 4.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 1.5;

    objs->objects.push_back(obj);
    return objs;
  }

  static int count_nonzero(const grid_map::Matrix & m)
  {
    int n = 0;
    for (int i = 0; i < m.rows(); ++i)
      for (int j = 0; j < m.cols(); ++j)
        if (m(i, j) > 0.0f) ++n;
    return n;
  }
};

// Object whose label is NOT in the excluded set → cells should be filled.
TEST_F(ObjectLabelFilterTest, NonExcludedLabelProducesCost)
{
  const std::unordered_set<uint8_t> excluded = {LABEL::TRUCK};
  auto objs = make_objects(LABEL::CAR);

  ObjectsToCostmap obj2costmap;
  const auto costmap = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, excluded, objs);

  EXPECT_GT(count_nonzero(costmap), 0)
    << "CAR object should produce cost when only TRUCK is excluded";
}

// Object whose label IS in the excluded set → costmap stays empty.
TEST_F(ObjectLabelFilterTest, ExcludedLabelProducesNoCost)
{
  const std::unordered_set<uint8_t> excluded = {LABEL::UNKNOWN, LABEL::PEDESTRIAN};
  auto objs = make_objects(LABEL::UNKNOWN);

  ObjectsToCostmap obj2costmap;
  const auto costmap = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, excluded, objs);

  EXPECT_EQ(count_nonzero(costmap), 0)
    << "UNKNOWN object should be skipped when UNKNOWN is in excluded set";
}

// Empty excluded set means no filtering → all objects pass through.
TEST_F(ObjectLabelFilterTest, EmptyExcludedSetPassesAllObjects)
{
  const std::unordered_set<uint8_t> no_filter = {};
  auto objs = make_objects(LABEL::UNKNOWN);

  ObjectsToCostmap obj2costmap;
  const auto costmap = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, no_filter, objs);

  EXPECT_GT(count_nonzero(costmap), 0) << "Empty excluded set should pass all objects";
}

// Object with multiple classifications: dominant (highest-probability) label decides.
TEST_F(ObjectLabelFilterTest, DominantLabelDeterminesFiltering)
{
  auto objs = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  objs->header.frame_id = "map";

  autoware_perception_msgs::msg::PredictedObject obj;
  // Two classifications: PEDESTRIAN 0.3, UNKNOWN 0.8 → dominant is UNKNOWN
  autoware_perception_msgs::msg::ObjectClassification cls_ped;
  cls_ped.label = LABEL::PEDESTRIAN;
  cls_ped.probability = 0.3f;
  autoware_perception_msgs::msg::ObjectClassification cls_unk;
  cls_unk.label = LABEL::UNKNOWN;
  cls_unk.probability = 0.8f;
  obj.classification = {cls_ped, cls_unk};
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 4.0;
  obj.shape.dimensions.y = 2.0;
  obj.shape.dimensions.z = 1.5;
  objs->objects.push_back(obj);

  ObjectsToCostmap obj2costmap;

  // UNKNOWN excluded → dominant is UNKNOWN → object should be skipped
  const auto costmap_exclude_unknown = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, {LABEL::UNKNOWN}, objs);
  EXPECT_EQ(count_nonzero(costmap_exclude_unknown), 0)
    << "Dominant label is UNKNOWN; object must be skipped when UNKNOWN is excluded";

  // PEDESTRIAN excluded → dominant is UNKNOWN (not excluded) → object should produce cost
  const auto costmap_exclude_ped = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, {LABEL::PEDESTRIAN},
    objs);
  EXPECT_GT(count_nonzero(costmap_exclude_ped), 0)
    << "Dominant label is UNKNOWN; object must be included when only PEDESTRIAN is excluded";
}

// Object with empty classification vector → treated as UNKNOWN.
TEST_F(ObjectLabelFilterTest, EmptyClassificationTreatedAsUnknown)
{
  auto objs = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  objs->header.frame_id = "map";

  autoware_perception_msgs::msg::PredictedObject obj;
  // no classification entries
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = 4.0;
  obj.shape.dimensions.y = 2.0;
  obj.shape.dimensions.z = 1.5;
  objs->objects.push_back(obj);

  ObjectsToCostmap obj2costmap;

  // UNKNOWN excluded → object skipped
  const auto costmap_exclude_unknown = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, {LABEL::UNKNOWN}, objs);
  EXPECT_EQ(count_nonzero(costmap_exclude_unknown), 0)
    << "Empty-classification object is treated as UNKNOWN; must be skipped when UNKNOWN is "
       "excluded";

  // CAR excluded → UNKNOWN not excluded → object included
  const auto costmap_exclude_car = obj2costmap.makeCostmapFromObjects(
    make_gridmap(), 0.0, 1, ObjectsToCostmap::ObjectCostMode::Fixed, 1.0, {LABEL::CAR}, objs);
  EXPECT_GT(count_nonzero(costmap_exclude_car), 0)
    << "Empty-classification object is treated as UNKNOWN; must be included when UNKNOWN is not "
       "excluded";
}

}  // namespace autoware::costmap_generator
