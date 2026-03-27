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

#include <autoware/costmap_generator/utils/points_to_costmap.hpp>

#include <gtest/gtest.h>

#include <string>

namespace autoware::costmap_generator
{
using pointcloud = pcl::PointCloud<pcl::PointXYZ>;
class PointsToCostmapTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  ~PointsToCostmapTest() override { rclcpp::shutdown(); }

  grid_map::GridMap construct_gridmap();

public:
  const double grid_resolution_ = 1.0;
  const double grid_length_x_ = 20.0;
  const double grid_length_y_ = 20.0;
  const double grid_position_x_ = 0.0;
  const double grid_position_y_ = 0.0;
};

grid_map::GridMap PointsToCostmapTest::construct_gridmap()
{
  grid_map::GridMap gm;

  gm.setFrameId("map");
  // set gridmap size,resolution
  gm.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_);
  // center of grid to position p in map frame
  gm.setPosition(grid_map::Position(grid_position_x_, grid_position_y_));

  // set initial value
  gm.add("points", 0);

  return gm;
}

// grid_y
// |  map_x-------
// |            |
// |            |
// |            |
// |            map_y
// |__________________grid_x
TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_validPoints)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;

  in_sensor_points.width = 3;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = 1;

  in_sensor_points.points[1].x = 1.1;
  in_sensor_points.points[1].y = 1;
  in_sensor_points.points[1].z = 4;

  in_sensor_points.points[2].x = 1.4;
  in_sensor_points.points[2].y = 2;
  in_sensor_points.points[2].z = 2.7;

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 5.0;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        // std::cout << "i:"<< i <<",j:"<<j<< std::endl;
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 3);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_biggerThanMaximumHeightThres)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = 1;  // out of [maximum_height_thres,minimum_lidar_height_thres]

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_lessThanMinimumHeightThres)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  in_sensor_points.points[0].x = 0.7;
  in_sensor_points.points[0].y = 1;
  in_sensor_points.points[0].z = -0.1;  // out of [maximum_height_thres,minimum_lidar_height_thres]

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}

TEST_F(PointsToCostmapTest, TestMakeCostmapFromPoints_invalidPoints_outOfGrid)
{
  // construct pointcloud in map frame
  pointcloud in_sensor_points;
  in_sensor_points.width = 1;
  in_sensor_points.height = 1;
  in_sensor_points.is_dense = false;
  in_sensor_points.resize(in_sensor_points.width * in_sensor_points.height);

  // when we construct gridmap,we set grid map center to (0,0) in map frame
  // so it would be outside of grid map if absolute value of point.x bigger than half of
  // grid_length_x_
  in_sensor_points.points[0].x = 1 + grid_length_x_ / 2.0;
  in_sensor_points.points[0].y = 1 + grid_length_y_ / 2.0;
  in_sensor_points.points[0].z = 0.5;

  grid_map::GridMap gridmap = construct_gridmap();

  PointsToCostmap point2costmap;
  const double maximum_height_thres = 0.99;
  const double minimum_lidar_height_thres = 0.0;
  const double grid_min_value = 0.0;
  const double grid_max_value = 1.0;
  const std::string gridmap_layer_name = "points";
  grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, in_sensor_points);

  int nonempty_grid_cell_num = 0;
  for (int i = 0; i < costmap_data.rows(); i++) {
    for (int j = 0; j < costmap_data.cols(); j++) {
      if (costmap_data(i, j) == grid_max_value) {
        nonempty_grid_cell_num += 1;
      }
    }
  }

  EXPECT_EQ(nonempty_grid_cell_num, 0);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_CenteredGrid)
{
  // Test with grid centered at (0, 0) - matching grid_position
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Point at grid center should map to middle of grid
  pcl::PointXYZ center_point;
  center_point.x = 0.0;
  center_point.y = 0.0;
  center_point.z = 0.0;
  
  auto index = point2costmap.fetchGridIndexFromPoint(center_point);
  
  // For 20x20 grid with resolution 1.0, center should be at index (10, 10)
  EXPECT_EQ(index.x(), 10);
  EXPECT_EQ(index.y(), 10);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_OffsetGrid)
{
  // Test with grid centered at (5, 5) instead of (0, 0)
  grid_map::GridMap gridmap;
  gridmap.setFrameId("map");
  gridmap.setGeometry(grid_map::Length(20.0, 20.0), 1.0);
  gridmap.setPosition(grid_map::Position(5.0, 5.0));  // Offset center
  gridmap.add("points", 0);

  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Point at grid center (5, 5) should map to middle of grid
  pcl::PointXYZ center_point;
  center_point.x = 5.0;
  center_point.y = 5.0;
  center_point.z = 0.0;
  
  auto index = point2costmap.fetchGridIndexFromPoint(center_point);
  
  EXPECT_EQ(index.x(), 10);
  EXPECT_EQ(index.y(), 10);

  // Point at origin (0, 0) should be offset from center
  pcl::PointXYZ origin_point;
  origin_point.x = 0.0;
  origin_point.y = 0.0;
  origin_point.z = 0.0;
  
  auto origin_index = point2costmap.fetchGridIndexFromPoint(origin_point);
  
  // (0,0) is 5 meters from grid center (5,5), so 5 cells offset
  EXPECT_EQ(origin_index.x(), 15);
  EXPECT_EQ(origin_index.y(), 15);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_EdgePoints)
{
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Test points at grid edges
  // Grid spans from -10 to +10 in both axes (20m total, centered at 0)
  
  // Top-right corner (near +10, +10)
  pcl::PointXYZ top_right;
  top_right.x = 9.5;
  top_right.y = 9.5;
  top_right.z = 0.0;
  auto tr_index = point2costmap.fetchGridIndexFromPoint(top_right);
  EXPECT_EQ(tr_index.x(), 0);
  EXPECT_EQ(tr_index.y(), 0);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_EdgePoints2)
{
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Bottom-left corner (near -10, -10)
  pcl::PointXYZ bottom_left;
  bottom_left.x = -9.5;
  bottom_left.y = -9.5;
  bottom_left.z = 0.0;
  auto bl_index = point2costmap.fetchGridIndexFromPoint(bottom_left);
  EXPECT_EQ(bl_index.x(), 19);
  EXPECT_EQ(bl_index.y(), 19);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_MiddlePoint)
{
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Test negative coordinates
  pcl::PointXYZ neg_point;
  neg_point.x = -0.1;
  neg_point.y = -0.1;
  neg_point.z = 0.0;
  
  auto index = point2costmap.fetchGridIndexFromPoint(neg_point);
  // Verify it's valid and within grid bounds
  EXPECT_TRUE(point2costmap.isValidInd(index));
  EXPECT_EQ(index.x(), 10);
  EXPECT_EQ(index.y(), 10);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_NegativeCoordinates)
{
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Test negative coordinates
  pcl::PointXYZ neg_point;
  neg_point.x = -5.0;
  neg_point.y = -3.0;
  neg_point.z = 0.0;
  
  auto index = point2costmap.fetchGridIndexFromPoint(neg_point);
  // Verify it's valid and within grid bounds
  EXPECT_TRUE(point2costmap.isValidInd(index));
  EXPECT_EQ(index.x(), 15);
  EXPECT_EQ(index.y(), 13);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_DifferentResolution)
{
  // Test with different grid resolution
  grid_map::GridMap gridmap;
  gridmap.setFrameId("map");
  gridmap.setGeometry(grid_map::Length(20.0, 20.0), 0.5);  // 0.5m resolution
  gridmap.setPosition(grid_map::Position(0.0, 0.0));
  gridmap.add("points", 0);

  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Center point
  pcl::PointXYZ center_point;
  center_point.x = 0.0;
  center_point.y = 0.0;
  center_point.z = 0.0;
  
  auto index = point2costmap.fetchGridIndexFromPoint(center_point);
  
  // With 0.5m resolution, 20m grid = 40 cells, center at (20, 20)
  EXPECT_EQ(index.x(), 20);
  EXPECT_EQ(index.y(), 20);
}

TEST_F(PointsToCostmapTest, TestFetchGridIndexFromPoint_ConsistencyCheck)
{
  // Verify that offset calculation formula is consistent
  grid_map::GridMap gridmap = construct_gridmap();
  PointsToCostmap point2costmap;
  point2costmap.initGridmapParam(gridmap);

  // Test a known point mapping
  pcl::PointXYZ test_point;
  test_point.x = 2.0;
  test_point.y = 3.0;
  test_point.z = 0.0;
  
  auto index1 = point2costmap.fetchGridIndexFromPoint(test_point);
  
  // Calculate expected index manually using the formula
  const double grid_length_x = 20.0;
  const double grid_length_y = 20.0;
  const double grid_position_x = 0.0;
  const double grid_position_y = 0.0;
  const double grid_resolution = 1.0;
  
  const double origin_x_offset = grid_length_x / 2.0 - grid_position_x;  // 10.0
  const double origin_y_offset = grid_length_y / 2.0 - grid_position_y;  // 10.0
  
  double mapped_x = (grid_length_x - origin_x_offset - test_point.x) / grid_resolution;
  double mapped_y = (grid_length_y - origin_y_offset - test_point.y) / grid_resolution;
  
  int expected_x = static_cast<int>(std::floor(mapped_x));
  int expected_y = static_cast<int>(std::floor(mapped_y));
  
  EXPECT_EQ(index1.x(), expected_x);
  EXPECT_EQ(index1.y(), expected_y);
  
  // Verify the calculation: (20 - 10 - 2) / 1 = 8
  EXPECT_EQ(expected_x, 8);
  EXPECT_EQ(expected_y, 7);  // (20 - 10 - 3) / 1 = 7
}
}  // namespace autoware::costmap_generator
