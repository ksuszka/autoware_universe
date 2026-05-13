// Copyright 2025 TIER IV, Inc.
// Copyright (c) 2025 Autonomous Systems Sp. z o.o.
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

#include "object_splitter/object_splitter.hpp"

#include "utils/utils.hpp"

#include <autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp>
#include <autoware/shape_estimation/shape_estimator.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace detection_by_tracker
{

enum class Shape { BOX, L };

struct Range
{
  double start;
  double end;
};

struct TrackedObjects
{
  uint8_t label;
  Range x_range;
  Range y_range;
  double height;
  double existence_probability;
};

struct TestObjects
{
  Range x_range;
  Range y_range;
  double height;
  Shape shape;
};

struct ExpectedOutput
{
  std::map<uint8_t, int> label_counts;
};

struct MultipleObjectsTestParams
{
  std::string name;
  std::vector<TrackedObjects> tracked_objects;
  std::vector<TestObjects> test_objects;
  std::map<uint8_t, int> label_counts;
};

// Test scenarios
const std::vector<MultipleObjectsTestParams> kMultipleObjectsTestParams = {
  // Scenario 1: Two cars side by side, with two matching point clouds
  {"TwoCarsWithMatchingClouds",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::CAR, {8.0, 12.0}, {7.0, 9.0}, 1.5, 0.9}},
   // Test objects (point clouds)
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX}, {{8.0, 12.0}, {7.0, 9.0}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 2}, {Label::UNKNOWN, 0}}},

  // Scenario 2: Car and truck with matching point clouds
  {"CarAndTruckWithMatchingClouds",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::TRUCK, {16.5, 23.5}, {3.75, 6.25}, 3.0, 0.9}},
   // Test objects (point clouds)
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX}, {{16.5, 23.5}, {3.75, 6.25}, 3.0, Shape::BOX}},
   // Expected output
   {{Label::CAR, 1}, {Label::TRUCK, 1}, {Label::UNKNOWN, 0}}},

  // Scenario 3: One car with matching cloud and overlapping point cloud
  {"OneCarWithOverlappingCloud",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9}},
   // Test objects (point clouds) - second cloud is offset but within search area
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX},
    {{11.99 + 4.0 * 0.05 / 2, 14.0}, {4.0, 6.0}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 1}, {Label::UNKNOWN, 1}}},

  // Scenario 4: One car with matching cloud and not overlapping point cloud
  {"OneCarWithNotOverlappingCloud",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9}},
   // Test objects (point clouds) - second cloud beyond search area
   // 4.0 * 0.05 / 2 is the offset due to extend scale of CAR
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX},
    {{12.01 + 4.0 * 0.05 / 2, 14.0}, {4.0, 6.0}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 1}, {Label::UNKNOWN, 0}}},

  // Scenario 5: One car with larger, partially overlapping cloud in between
  {"OneCarWithLargerPartiallyOverlappingCloud",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9}},
   // Test objects (point clouds) - larger cloud that partially overlaps
   {{{9.0, 11.0}, {3.0, 7.0}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 1}, {Label::UNKNOWN, 2}}},

  // Scenario 6: One car with larger, overlapping cloud
  {"OneCarWithLargerOverlappingCloud",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9}},
   // Test objects (point clouds) - slightly larger cloud that fully overlaps
   {{{7.75, 12.25}, {3.75, 6.25}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 1}, {Label::UNKNOWN, 1}}},

  // Scenario 7: Two cars with overlapping cloud in between
  {"TwoCarsWithOverlappingCloudInBetween",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::CAR, {8.0, 12.0}, {7.0, 9.0}, 1.5, 0.9}},
   // Test objects (point clouds) - cloud in between the two cars
   {{{9.0, 11.0}, {5.5, 7.5}, 1.5, Shape::BOX}},
   // Expected output
   {{Label::CAR, 2}, {Label::UNKNOWN, 1}}},

  // Scenario 8: Pedestrian standing by a car with matching clouds
  {"PedestrianNearCarWithMatchingClouds",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::PEDESTRIAN, {11.2, 12.0}, {6.0, 6.8}, 1.7, 0.9}},
   // Test objects (point clouds)
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX}, {{11.2, 12.0}, {6.0, 6.8}, 1.7, Shape::BOX}},
   // Expected output - pedestrian is not processed du
   {{Label::CAR, 1}, {Label::PEDESTRIAN, 0}, {Label::UNKNOWN, 0}}},

  // Scenario 9: Unknown object near the car
  {"UnknownNearTheCar",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::PEDESTRIAN, {11.2, 12.0}, {6.0, 6.8}, 1.7, 0.9}},
   // Test objects (point clouds)
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::BOX}, {{11.2, 12.0}, {6.0, 6.8}, 1.7, Shape::BOX}},
   // Expected output - pedestrian is not processed due to size filter
   {{Label::CAR, 1}, {Label::PEDESTRIAN, 0}, {Label::UNKNOWN, 0}}},

  // Scenario 10: Two cars with L-shaped point clouds
  {"TwoCarsWithLShapedClouds",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.9},
    {Label::CAR, {8.0, 12.0}, {9.0, 11.0}, 1.5, 0.9}},
   // Test objects (L-shaped point clouds)
   {{{8.0, 12.0}, {4.0, 6.0}, 1.5, Shape::L}, {{8.0, 12.0}, {9.0, 11.0}, 1.5, Shape::L}},
   // Expected output
   {{Label::CAR, 2}, {Label::UNKNOWN, 0}}},

  // Scenario 11: Low probability tracked object (below threshold) with overlapping test object
  {"LowProbabilityTrackedObject",
   // Tracked objects
   {{Label::CAR, {8.0, 12.0}, {4.0, 6.0}, 1.5, 0.05}},  // Low probability
   // Test objects
   {{{8.0, 12.0}, {5.0, 7.0}, 1.5, Shape::BOX}},
   // Expected output - should ignore the tracked object due to low probability
   {{Label::UNKNOWN, 1}}},

  // Scenario 12: Group of pedestrians
  {"GroupOfPedestrians",
   // Tracked objects - various pedestrians with different positions
   {{Label::PEDESTRIAN, {9.65, 10.35}, {4.65, 5.35}, 1.4, 0.9},   // Standalone pedestrian
    {Label::PEDESTRIAN, {11.65, 12.35}, {4.65, 5.35}, 1.4, 0.9},  // Standalone pedestrian
    {Label::PEDESTRIAN, {9.4, 10.6}, {6.65, 7.35}, 1.5, 0.9},     // Large pedestrian (merged two?)
    {Label::PEDESTRIAN, {14.65, 15.35}, {4.65, 5.35}, 1.8, 0.9},  // Part of a group (close to next)
    {Label::PEDESTRIAN, {15.15, 15.85}, {4.65, 5.35}, 1.8, 0.9},  // Part of a group (close to
                                                                  // previous)
    {Label::PEDESTRIAN, {17.65, 18.35}, {4.65, 5.35}, 1.8, 0.9},  // Close to a large cluster
    {Label::PEDESTRIAN, {19.65, 20.35}, {4.65, 5.35}, 1.8, 0.9}}  // Far from all clusters
   ,
   // Test objects (point clouds)
   {
     {{9.65, 10.35}, {4.65, 5.35}, 1.8, Shape::BOX},    // Match first pedestrian
     {{11.8, 12.2}, {4.8, 5.0}, 1.5, Shape::BOX},       // Smaller cloud for 2nd pedestrian
     {{9.25, 10.75}, {6.625, 7.375}, 1.8, Shape::BOX},  // Large cloud overlapping large pedestrian:
                                                        // +2 unknown
     {{14.65, 15.85}, {4.6, 5.4}, 1.8, Shape::BOX},     // Large cloud covering both grouped
                                                        // pedestrians
     {{17.4, 18.6}, {4.4, 5.6}, 1.8, Shape::BOX},     // Larger than standard pedestrian: +1 unknown
     {{21.65, 22.35}, {4.65, 5.35}, 1.8, Shape::BOX}  // No corresponding tracked object
   },
   // Expected output
   // Some pedestrians should be detected properly, some clusters should be marked as unknown
   {{Label::PEDESTRIAN, 0}, {Label::UNKNOWN, 3}}},

  // Scenario 13: Bicycle with object at the corner
  {"BicycleWithObjectAtCorner",
   // Tracked objects
   {{Label::BICYCLE, {9.1, 10.9}, {4.8, 5.2}, 1.5, 0.9}},  // Bicycle
   // Test objects (point clouds)
   {
     {{9.4, 10.6}, {4.8, 5.2}, 1.5, Shape::BOX},      // Bicycle
     {{10.75, 11.25}, {4.95, 5.45}, 1.5, Shape::BOX}  // Object at the corner
   },
   // Expected output
   {{Label::BICYCLE, 0}, {Label::UNKNOWN, 1}}},

  // Scenario 14: Extremely long bicycle with object at the corner
  {"LongBicycleWithObjectAtCorner",
   // Tracked objects
   {{Label::BICYCLE, {8.0, 12.0}, {4.8, 5.2}, 1.5, 0.9}},  // Long Bicycle
   // Test objects (point clouds)
   {
     {{8.0, 12.0}, {4.8, 5.2}, 1.5, Shape::BOX},      // Long Bicycle
     {{11.95, 12.45}, {4.95, 5.45}, 1.5, Shape::BOX}  // Object at the corner
   },
   // Expected output: 0 unknown objects due to distance threshold
   {{Label::BICYCLE, 0}, {Label::UNKNOWN, 0}}},
};

class ObjectSplitterMultipleObjectsTest : public ::testing::TestWithParam<MultipleObjectsTestParams>
{
public:
  ObjectSplitterMultipleObjectsTest()
  : logger_(rclcpp::get_logger("object_splitter_multiple_objects_test"))
  {
  }

protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create test parameters
    max_search_distance_map_ = {{Label::UNKNOWN, 6}, {Label::CAR, 6},       {Label::TRUCK, 9},
                                {Label::BUS, 9},     {Label::TRAILER, 11},  {Label::MOTORCYCLE, 3},
                                {Label::BICYCLE, 2}, {Label::PEDESTRIAN, 2}};

    tracker_ignore_.UNKNOWN = true;
    tracker_ignore_.CAR = false;
    tracker_ignore_.TRUCK = false;
    tracker_ignore_.BUS = false;
    tracker_ignore_.TRAILER = false;
    tracker_ignore_.MOTORCYCLE = false;
    tracker_ignore_.BICYCLE = false;
    tracker_ignore_.PEDESTRIAN = false;

    shape_estimator_ = autoware::shape_estimation::ShapeEstimator(false, true);
    cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
      false, 1, 1000000000, 0.3, 0.2, 1);

    extend_scale_ = 1.05;
    buffer_distance_ = 0.1;
    existence_probability_threshold_ = 0.08;
    existence_probability_modifier_ = 0.5;

    logger_ = rclcpp::get_logger("object_splitter_multiple_objects_test");

    object_splitter_ = std::make_unique<ObjectSplitter>(
      max_search_distance_map_, tracker_ignore_, shape_estimator_, cluster_, extend_scale_,
      buffer_distance_, existence_probability_threshold_, existence_probability_modifier_,
      false, logger_);
  }

  // Helper methods to create test data
  autoware_perception_msgs::msg::DetectedObject createTestBBoxObject(
    uint8_t label_type, const Range & x_range, const Range & y_range, double height,
    double probability = 0.5)
  {
    autoware_perception_msgs::msg::DetectedObject object;

    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = label_type;
    classification.probability = 1.0;
    object.classification.push_back(classification);

    object.kinematics.pose_with_covariance.pose.position.x = (x_range.start + x_range.end) / 2.0;
    object.kinematics.pose_with_covariance.pose.position.y = (y_range.start + y_range.end) / 2.0;
    object.kinematics.pose_with_covariance.pose.position.z = 0.0;
    object.kinematics.pose_with_covariance.pose.orientation.x = 0.0;
    object.kinematics.pose_with_covariance.pose.orientation.y = 0.0;
    object.kinematics.pose_with_covariance.pose.orientation.z = 0.0;
    object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = x_range.end - x_range.start;
    object.shape.dimensions.y = y_range.end - y_range.start;
    object.shape.dimensions.z = height;

    object.existence_probability = probability;

    return object;
  }

  pcl::PointCloud<pcl::PointXYZ> createLShapedPointCloud(
    const Range & x_range, const Range & y_range, float height, int density = 40)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    float center_x = (x_range.start + x_range.end) / 2.0;
    float center_y = (y_range.start + y_range.end) / 2.0;
    float width = x_range.end - x_range.start;
    float length = y_range.end - y_range.start;

    density = std::max(density, 40);

    float dx = width / density;
    float dy = length / density;
    float dz = height / std::max(10, density / 4);

    // Create x part
    for (int i = 0; i < density; ++i) {
      for (int k = 0; k < static_cast<int>(height / dz); ++k) {
        pcl::PointXYZ point;
        point.x = x_range.start + i * dx;
        point.y = center_y;
        point.z = -height / 2 + k * dz;
        cloud.points.push_back(point);
      }
    }

    // Create y part
    for (int j = 0; j < density; ++j) {
      for (int k = 0; k < static_cast<int>(height / dz); ++k) {
        pcl::PointXYZ point;
        point.x = center_x;
        point.y = y_range.start + j * dy;
        point.z = -height / 2 + k * dz;
        cloud.points.push_back(point);
      }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> create3DBoxPointCloud(
    const Range & x_range, const Range & y_range, float height, int density = 20)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Calculate size manually
    float width = x_range.end - x_range.start;
    float length = y_range.end - y_range.start;

    density = std::max(density, 20);

    float dx = width / density;
    float dy = length / density;
    float dz = height / std::max(10, density / 4);

    // Top face
    for (int i = 0; i < density; ++i) {
      for (int j = 0; j < density; ++j) {
        pcl::PointXYZ point;
        point.x = x_range.start + i * dx;
        point.y = y_range.start + j * dy;
        point.z = height / 2;
        cloud.points.push_back(point);
      }
    }

    // Side faces
    for (int k = 0; k < static_cast<int>(height / dz); ++k) {
      for (int i = 0; i < density; ++i) {
        // Front and back faces
        pcl::PointXYZ point1, point2;
        point1.x = x_range.start + i * dx;
        point1.y = y_range.start;
        point1.z = -height / 2 + k * dz;

        point2.x = x_range.start + i * dx;
        point2.y = y_range.end;
        point2.z = -height / 2 + k * dz;

        cloud.points.push_back(point1);
        cloud.points.push_back(point2);
      }

      for (int j = 0; j < density; ++j) {
        // Left and right faces
        pcl::PointXYZ point1, point2;
        point1.x = x_range.start;
        point1.y = y_range.start + j * dy;
        point1.z = -height / 2 + k * dz;

        point2.x = x_range.end;
        point2.y = y_range.start + j * dy;
        point2.z = -height / 2 + k * dz;

        cloud.points.push_back(point1);
        cloud.points.push_back(point2);
      }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    return cloud;
  }

  tier4_perception_msgs::msg::DetectedObjectsWithFeature createClusterObjectsMsg(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
  {
    tier4_perception_msgs::msg::DetectedObjectsWithFeature object_with_feature_msg;
    object_with_feature_msg.header.frame_id = "base_link";
    object_with_feature_msg.header.stamp = rclcpp::Clock().now();

    for (const auto & cluster : clusters) {
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;

      // Set classification as UNKNOWN
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = Label::UNKNOWN;
      classification.probability = 1.0;
      feature_object.object.classification.push_back(classification);

      // Set cluster
      sensor_msgs::msg::PointCloud2 cluster_msg;
      pcl::toROSMsg(cluster, cluster_msg);
      cluster_msg.header.frame_id = "base_link";
      cluster_msg.header.stamp = rclcpp::Clock().now();
      feature_object.feature.cluster = cluster_msg;

      // Estimate shape
      shape_estimator_.estimateShapeAndPose(
        Label::UNKNOWN, cluster, boost::none, boost::none, boost::none, feature_object.object.shape,
        feature_object.object.kinematics.pose_with_covariance.pose);

      object_with_feature_msg.feature_objects.push_back(feature_object);
    }

    return object_with_feature_msg;
  }

  // Test parameters
  std::map<uint8_t, int> max_search_distance_map_;
  detection_by_tracker::utils::TrackerIgnoreLabel tracker_ignore_;
  autoware::shape_estimation::ShapeEstimator shape_estimator_{false, true};
  std::shared_ptr<autoware::euclidean_cluster::EuclideanClusterInterface> cluster_;
  double extend_scale_;
  double buffer_distance_;
  double existence_probability_threshold_;
  double existence_probability_modifier_;
  rclcpp::Logger logger_;

  std::unique_ptr<ObjectSplitter> object_splitter_;
};

TEST_P(ObjectSplitterMultipleObjectsTest, ProcessMultipleObjects)
{
  const auto & test_params = GetParam();

  // Create tracked objects
  autoware_perception_msgs::msg::DetectedObjects tracked_objects;
  tracked_objects.header.frame_id = "base_link";
  tracked_objects.header.stamp = rclcpp::Clock().now();

  for (const auto & tracked_obj_def : test_params.tracked_objects) {
    auto tracked_object = createTestBBoxObject(
      tracked_obj_def.label, tracked_obj_def.x_range, tracked_obj_def.y_range,
      tracked_obj_def.height, tracked_obj_def.existence_probability);
    tracked_objects.objects.push_back(tracked_object);
  }

  // Create test point clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  for (const auto & test_obj_def : test_params.test_objects) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (test_obj_def.shape == Shape::L) {
      cloud = createLShapedPointCloud(
        test_obj_def.x_range, test_obj_def.y_range, test_obj_def.height,
        std::max(
          40, static_cast<int>(60 * (test_obj_def.x_range.end - test_obj_def.x_range.start))));
    } else {
      cloud = create3DBoxPointCloud(
        test_obj_def.x_range, test_obj_def.y_range, test_obj_def.height,
        std::max(
          20, static_cast<int>(30 * (test_obj_def.x_range.end - test_obj_def.x_range.start))));
    }
    clusters.push_back(cloud);
  }

  // Create cluster objects message
  auto cluster_objects = createClusterObjectsMsg(clusters);

  // Process the objects
  autoware_perception_msgs::msg::DetectedObjects out_objects;
  object_splitter_->process(cluster_objects, tracked_objects, out_objects);

  // Calculate expected total count from label counts
  int expected_total_count = 0;
  for (const auto & [label, count] : test_params.label_counts) {
    expected_total_count += count;
  }

  // Check total count
  EXPECT_EQ(out_objects.objects.size(), expected_total_count) << "Test case: " << test_params.name;

  // Count objects by label
  std::map<uint8_t, int> actual_label_counts;
  for (const auto & object : out_objects.objects) {
    if (!object.classification.empty()) {
      uint8_t label = object.classification.front().label;
      actual_label_counts[label]++;
    }
  }

  // Check counts by label
  for (const auto & [label, expected_count] : test_params.label_counts) {
    EXPECT_EQ(actual_label_counts[label], expected_count)
      << "Test case: " << test_params.name << ", label: " << static_cast<int>(label)
      << ", expected: " << expected_count << ", actual: " << actual_label_counts[label];
  }
}

INSTANTIATE_TEST_SUITE_P(
  MultipleObjectsScenarios, ObjectSplitterMultipleObjectsTest,
  ::testing::ValuesIn(kMultipleObjectsTestParams),
  [](const testing::TestParamInfo<MultipleObjectsTestParams> & info) { return info.param.name; });

}  // namespace detection_by_tracker
}  // namespace autoware

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
