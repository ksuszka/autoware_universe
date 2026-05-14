// Copyright 2026 Tier IV, Inc.
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

#ifndef AUTOWARE__COSTMAP_GENERATOR__COSTMAP_DIFF_OVERLAY_HPP_
#define AUTOWARE__COSTMAP_GENERATOR__COSTMAP_DIFF_OVERLAY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>

namespace autoware::costmap_generator
{

class CostmapDiffOverlay
{
public:
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

  explicit CostmapDiffOverlay(rclcpp::Node & node, const std::chrono::seconds heatmap_window);

  void onCostmap(const OccupancyGrid::ConstSharedPtr & current);
  void reset();

private:
  rclcpp::Publisher<OccupancyGrid>::SharedPtr overlayed_costmap_pub_;
  OccupancyGrid::ConstSharedPtr previous_costmap_;
  std::vector<rclcpp::Time> changed_until_;
  std::vector<int8_t> change_intensity_;
  std::chrono::seconds heatmap_window_;

  void ensureBuffers(const OccupancyGrid & current);
  void updateChangedCells(const OccupancyGrid & current, const rclcpp::Time & now);
  void publishHeatmap(const OccupancyGrid & current, const rclcpp::Time & now);
  void remapPersistentState(const OccupancyGrid & current);
  static bool isSameGridShape(const OccupancyGrid & lhs, const OccupancyGrid & rhs);
  static bool isSameOrigin(const OccupancyGrid & lhs, const OccupancyGrid & rhs);
  static int8_t scaleDeltaToHeat(int delta);
};

}  // namespace autoware::costmap_generator

#endif  // AUTOWARE__COSTMAP_GENERATOR__COSTMAP_DIFF_OVERLAY_HPP_
