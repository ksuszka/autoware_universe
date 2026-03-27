// Copyright 2024 TIER IV
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

#include <../src/lanelet2_plugins/utility_functions.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <Eigen/src/Geometry/Rotation2D.h>

#include <cmath>
#include <vector>

using autoware::mission_planner_universe::lanelet2::convert_linear_ring_to_polygon;
using autoware::mission_planner_universe::lanelet2::convertBasicPoint3dToPose;
using autoware::mission_planner_universe::lanelet2::convertCenterlineToPoints;
using autoware::mission_planner_universe::lanelet2::exists;
using autoware::mission_planner_universe::lanelet2::get_closest_centerline_pose;
using autoware::mission_planner_universe::lanelet2::insert_marker_array;
using autoware::mission_planner_universe::lanelet2::is_in_lane;
using autoware::mission_planner_universe::lanelet2::is_in_parking_lot;
using autoware::mission_planner_universe::lanelet2::is_in_parking_space;
using autoware::mission_planner_universe::lanelet2::project_goal_to_map;

using autoware::vehicle_info_utils::VehicleInfo;
using geometry_msgs::msg::Pose;

TEST(TestUtilityFunctions, convertLinearRingToPolygon)
{
  // clockwise
  {
    autoware_utils::LinearRing2d footprint;
    footprint.push_back({1.0, 1.0});
    footprint.push_back({1.0, -1.0});
    footprint.push_back({0.0, -1.0});
    footprint.push_back({-1.0, -1.0});
    footprint.push_back({-1.0, 1.0});
    footprint.push_back({0.0, 1.0});
    footprint.push_back({1.0, 1.0});
    autoware_utils::Polygon2d polygon = convert_linear_ring_to_polygon(footprint);

    ASSERT_EQ(polygon.outer().size(), footprint.size());
    for (std::size_t i = 0; i < footprint.size(); ++i) {
      EXPECT_DOUBLE_EQ(
        boost::geometry::get<0>(polygon.outer()[i]), boost::geometry::get<0>(footprint[i]));
      EXPECT_DOUBLE_EQ(
        boost::geometry::get<1>(polygon.outer()[i]), boost::geometry::get<1>(footprint[i]));
    }

    EXPECT_EQ(polygon.outer().front(), polygon.outer().back());

    const double area = boost::geometry::area(polygon);
    EXPECT_GT(area, 0.0);
  }

  // counterclockwise
  {
    autoware_utils::LinearRing2d footprint;
    footprint.push_back({1.0, 1.0});
    footprint.push_back({0.0, 1.0});
    footprint.push_back({-1.0, 1.0});
    footprint.push_back({-1.0, -1.0});
    footprint.push_back({0.0, -1.0});
    footprint.push_back({1.0, -1.0});
    footprint.push_back({1.0, 1.0});
    autoware_utils::Polygon2d polygon = convert_linear_ring_to_polygon(footprint);

    ASSERT_EQ(polygon.outer().size(), footprint.size());

    // polygon is always clockwise
    for (std::size_t i = 0; i < footprint.size(); ++i) {
      const std::size_t j = footprint.size() - i - 1;
      EXPECT_DOUBLE_EQ(
        boost::geometry::get<0>(polygon.outer()[i]), boost::geometry::get<0>(footprint[j]));
      EXPECT_DOUBLE_EQ(
        boost::geometry::get<1>(polygon.outer()[i]), boost::geometry::get<1>(footprint[j]));
    }

    const double area = boost::geometry::area(polygon);
    EXPECT_GT(area, 0.0);
  }
}

TEST(TestUtilityFunctions, convertCenterlineToPoints)
{
  lanelet::LineString3d left_bound;
  lanelet::LineString3d right_bound;
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});
  lanelet::Lanelet lanelet{lanelet::InvalId, left_bound, right_bound};

  lanelet::LineString3d centerline;
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, -1, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 0, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 1, 0});
  lanelet.setCenterline(centerline);

  const std::vector<geometry_msgs::msg::Point> points = convertCenterlineToPoints(lanelet);

  ASSERT_EQ(points.size(), centerline.size());
  for (std::size_t i = 0; i < centerline.size(); ++i) {
    EXPECT_DOUBLE_EQ(points[i].x, centerline[i].x());
    EXPECT_DOUBLE_EQ(points[i].y, centerline[i].y());
    EXPECT_DOUBLE_EQ(points[i].z, centerline[i].z());
  }
}

TEST(TestUtilityFunctions, insertMarkerArray)
{
  visualization_msgs::msg::MarkerArray a1;
  visualization_msgs::msg::MarkerArray a2;
  a1.markers.resize(1);
  a2.markers.resize(2);
  a1.markers[0].id = 0;
  a2.markers[0].id = 1;
  a2.markers[1].id = 2;

  insert_marker_array(&a1, a2);

  ASSERT_EQ(a1.markers.size(), 3);
  EXPECT_EQ(a1.markers[0].id, 0);
  EXPECT_EQ(a1.markers[1].id, 1);
  EXPECT_EQ(a1.markers[2].id, 2);
}

TEST(TestUtilityFunctions, convertBasicPoint3dToPose)
{
  {
    const lanelet::BasicPoint3d point(1.0, 2.0, 3.0);
    const double lane_yaw = 0.0;
    const Pose pose = convertBasicPoint3dToPose(point, lane_yaw);
    EXPECT_DOUBLE_EQ(pose.position.x, point.x());
    EXPECT_DOUBLE_EQ(pose.position.y, point.y());
    EXPECT_DOUBLE_EQ(pose.position.z, point.z());
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
  }

  {
    const lanelet::BasicPoint3d point(1.0, 2.0, 3.0);
    const double lane_yaw = M_PI_2;
    const Pose pose = convertBasicPoint3dToPose(point, lane_yaw);
    EXPECT_DOUBLE_EQ(pose.position.x, point.x());
    EXPECT_DOUBLE_EQ(pose.position.y, point.y());
    EXPECT_DOUBLE_EQ(pose.position.z, point.z());
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.7071067811865476);
    EXPECT_DOUBLE_EQ(pose.orientation.w, 0.7071067811865476);
  }
}

TEST(TestUtilityFunctions, is_in_lane)
{
  lanelet::LineString3d left_bound;
  lanelet::LineString3d right_bound;
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});
  lanelet::Lanelet lanelet{lanelet::InvalId, left_bound, right_bound};

  lanelet::LineString3d centerline;
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, -1, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 0, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 1, 0});
  lanelet.setCenterline(centerline);

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    EXPECT_TRUE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 1};
    EXPECT_TRUE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, -1};
    EXPECT_TRUE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 2};
    EXPECT_FALSE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, -2};
    EXPECT_FALSE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 2, 0};
    EXPECT_FALSE(is_in_lane(lanelet, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -2, 0};
    EXPECT_FALSE(is_in_lane(lanelet, point));
  }
}

TEST(TestUtilityFunctions, is_in_parking_lot)
{
  lanelet::Polygon3d parking_lot;
  parking_lot.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});
  parking_lot.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  parking_lot.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});
  parking_lot.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1});
  parking_lot.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    EXPECT_TRUE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 1, 0};
    EXPECT_TRUE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 1};
    EXPECT_TRUE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 2, 0};
    EXPECT_FALSE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 2};
    EXPECT_FALSE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -2, 0};
    EXPECT_FALSE(is_in_parking_lot({parking_lot}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, -2};
    EXPECT_FALSE(is_in_parking_lot({parking_lot}, point));
  }
}

TEST(TestUtilityFunctions, is_in_parking_space)
{
  lanelet::LineString3d parking_space;
  parking_space.push_back(lanelet::Point3d{lanelet::InvalId, -1, 0});
  parking_space.push_back(lanelet::Point3d{lanelet::InvalId, 1, 0});
  parking_space.setAttribute("width", 2.0);

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    EXPECT_TRUE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 1, 0};
    EXPECT_TRUE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -1, 0};
    EXPECT_TRUE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 2, 0};
    EXPECT_FALSE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 2};
    EXPECT_FALSE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -2, 0};
    EXPECT_FALSE(is_in_parking_space({parking_space}, point));
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, -2};
    EXPECT_FALSE(is_in_parking_space({parking_space}, point));
  }
}

TEST(TestUtilityFunctions, project_goal_to_map)
{
  const auto create_lane = [&](const double height) -> lanelet::Lanelet {
    lanelet::LineString3d left_bound;
    lanelet::LineString3d right_bound;
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1, height});
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, -1, height});
    left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1, height});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1, height});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, 1, height});
    right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1, height});
    const lanelet::Lanelet lanelet{lanelet::InvalId, left_bound, right_bound};
    return lanelet;
  };

  const auto check_height = [&](const double height) -> void {
    const auto lanelet = create_lane(height);
    lanelet::Point3d goal_point{lanelet::InvalId, 0, 0, height};
    EXPECT_DOUBLE_EQ(project_goal_to_map(lanelet, goal_point), height);
  };

  check_height(0.0);
  check_height(1.0);
  check_height(-1.0);
}


/**
 * @brief Creates straightforward lanelet with 3 points.
 * @param yaw Yaw direction of the lanelet.
 * @param start Position of the first point of the lanelet centerline.
 * @param length Length of the lanelet.
 * @param length Width of the lanelet.
 * @param points Number of points of the lanelet centerline.
 * @return The lanelet.
 */
lanelet::Lanelet create_straightforward_lanelet(
  double yaw=0.0,
  const Eigen::Vector3d start = {-1, 0, 0},
  double length = 2.0,
  double width = 1.0,
  size_t points = 3) {
  assert(points > 1);
  Eigen::Rotation2Dd rot(yaw);

  lanelet::LineString3d left_bound;
  lanelet::LineString3d right_bound;
  lanelet::LineString3d centerline;

  auto to_3d = [](Eigen::Vector2d v) -> Eigen::Vector3d { return {v.x(), v.y(), 0}; };
  auto forward = to_3d(rot * Eigen::Vector2d{length / (points - 1), 0});
  auto left = to_3d(rot * Eigen::Vector2d{0, -width});
  auto right = to_3d(rot * Eigen::Vector2d{0, +width});

  auto point = start;
  for(size_t i=0; i < points; i++, point += forward)
  {
    left_bound.push_back({lanelet::InvalId, point + left});
    right_bound.push_back({lanelet::InvalId, point + right});
    centerline.push_back({lanelet::InvalId, point});
  }

  lanelet::Lanelet lanelet{lanelet::InvalId, left_bound, right_bound};
  lanelet.setCenterline(centerline);

  return lanelet;
}

VehicleInfo create_vehicle_info(double left = 0.5, double right = 0.5) {
  VehicleInfo vehicle_info;
  vehicle_info.left_overhang_m = left;
  vehicle_info.right_overhang_m = right;
  return vehicle_info;
}

TEST(TestUtilityFunctions, get_closest_centerline_pose)
{
  auto lanelet = create_straightforward_lanelet();
  auto vehicle_info = create_vehicle_info();

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 1, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -1, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, -1.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 2, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, -2, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, -2.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 1};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, -1};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}


TEST(TestUtilityFunctions, get_closest_centerline_pose__two_lanelets)
{
  auto first_lanelet = create_straightforward_lanelet(0.0, {-1, 0});
  auto second_lanelet = create_straightforward_lanelet(0.0, {1, 0});
  auto vehicle_info = create_vehicle_info();

  auto get_centerline_pose = [&](lanelet::Point3d point) {
    return 
      get_closest_centerline_pose({first_lanelet, second_lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
  };

  {
    // From the first lanelet.
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    const Pose pose = get_centerline_pose(point);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }

  {
    // From the second lanelet.
    const lanelet::Point3d point{lanelet::InvalId, 2, 2};
    const Pose pose = get_centerline_pose(point);
    EXPECT_DOUBLE_EQ(pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}

TEST(TestUtilityFunctions, get_closest_centerline_pose__smaller_left_overhang)
{
  auto lanelet = create_straightforward_lanelet();
  auto vehicle_info = create_vehicle_info(0.2, 0.4);

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    // y = (right - left) / 2 = (0.4 - 0.2) / 2 = 0.1
    EXPECT_DOUBLE_EQ(pose.position.y, 0.1);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}

TEST(TestUtilityFunctions, get_closest_centerline_pose__smaller_right_overhang)
{
  auto lanelet = create_straightforward_lanelet();
  auto vehicle_info = create_vehicle_info(0.4, 0.2);

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    // y = (right - left) / 2 = (0.2 - 0.4) / 2 = -0.1
    EXPECT_DOUBLE_EQ(pose.position.y, -0.1);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}

TEST(TestUtilityFunctions, get_closest_centerline_pose__adjacent_to_centerline)
{
  auto lanelet = create_straightforward_lanelet();
  auto vehicle_info = create_vehicle_info();

  {
    const lanelet::Point3d point{lanelet::InvalId, 0, 0.5};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    // y axis should be adjacent to centerline y.
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}


TEST(TestUtilityFunctions, get_closest_centerline_pose__between_points)
{
  auto lanelet = create_straightforward_lanelet();
  auto vehicle_info = create_vehicle_info();

  {
    const lanelet::Point3d point{lanelet::InvalId, 0.5, 0.0};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    // Interpolated between two points: {0.0, 0.0} and {1.0, 0.0}.
    EXPECT_DOUBLE_EQ(pose.position.x, 0.5);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}

TEST(TestUtilityFunctions, get_closest_centerline_pose__between_points__horizontal)
{
  auto lanelet = create_straightforward_lanelet(M_PI / 2.0, {0, 0});
  auto vehicle_info = create_vehicle_info();

  {
    const lanelet::Point3d point{lanelet::InvalId, 0.5 + 0.1, 0.5};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    EXPECT_NEAR(pose.position.x, 0.0, 1e-12);
    // Interpolated between two points: {0.0, 0.0} and {sqrt(2)/2, 0.0}.
    EXPECT_NEAR(pose.position.y, 0.5, 1e-12);
    EXPECT_NEAR(pose.position.z, 0.0, 1e-12);
  }
}


TEST(TestUtilityFunctions, get_closest_centerline_pose__between_points__diagonal_lanelet)
{
  auto lanelet = create_straightforward_lanelet(M_PI / 4.0, {0, 0});
  auto vehicle_info = create_vehicle_info();
  const auto SQRT2_2 = M_SQRT2;

  {
    const lanelet::Point3d point{lanelet::InvalId, SQRT2_2 / 2 + 0.2, SQRT2_2 / 2 - 0.2};
    const Pose pose =
      get_closest_centerline_pose({lanelet}, convertBasicPoint3dToPose(point, 0.0), vehicle_info);
    // Interpolated between two points: {0.0, 0.0} and {sqrt(2)/2, 0.0}.
    EXPECT_DOUBLE_EQ(pose.position.x, SQRT2_2 / 2);
    EXPECT_DOUBLE_EQ(pose.position.y, SQRT2_2 / 2);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  }
}