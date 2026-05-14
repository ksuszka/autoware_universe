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

#include "autoware/costmap_generator/costmap_diff_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autoware::costmap_generator
{

namespace
{
constexpr int8_t unknown_cost = -1;
constexpr int8_t free_cost = 0;
constexpr double epsilon = 1e-6;
constexpr int frequency_boost = 10;

constexpr double delta_log_base = 1.0;
constexpr double delta_log_max = 101.0;
constexpr int heat_min = 30;
constexpr int heat_span = 30;
constexpr int heat_max = 70;

constexpr double cellCenterCoord(
  const double origin, const size_t cell_idx, const double resolution)
{
  return origin + (static_cast<double>(cell_idx) + 0.5) * resolution;
}

constexpr bool isWithinGrid(
  const double world_x, const double world_y, const double min_x, const double min_y,
  const double max_x, const double max_y)
{
  return world_x >= min_x && world_x < max_x - epsilon && world_y >= min_y &&
         world_y < max_y - epsilon;
}

constexpr size_t toLinearIndex(const size_t x, const size_t y, const size_t width)
{
  return y * width + x;
}

constexpr int toCellCost(const int8_t cost)
{
  return static_cast<int>(static_cast<uint8_t>(cost));
}

constexpr int boostedHeat(const int base_heat, const bool already_active)
{
  const auto boosted = already_active ? base_heat + frequency_boost : base_heat;
  return std::clamp(boosted, 0, heat_max);
}

constexpr bool isHighlightedCost(const int8_t cost)
{
  return cost > free_cost;
}
}  // namespace

CostmapDiffOverlay::CostmapDiffOverlay(
  rclcpp::Node & node, const std::chrono::seconds heatmap_window)
: heatmap_window_{heatmap_window}
{
  rclcpp::QoS qos{1};
  qos.transient_local();
  overlayed_costmap_pub_ = node.create_publisher<OccupancyGrid>("~/overlayed_costmap", qos);
}

void CostmapDiffOverlay::onCostmap(const OccupancyGrid::ConstSharedPtr & current)
{
  if (!current) {
    return;
  }

  const auto now = rclcpp::Time{current->header.stamp};
  ensureBuffers(*current);
  updateChangedCells(*current, now);
  publishHeatmap(*current, now);
  previous_costmap_ = current;
}

void CostmapDiffOverlay::reset()
{
  previous_costmap_.reset();
  changed_until_.clear();
  change_intensity_.clear();
}

void CostmapDiffOverlay::ensureBuffers(const OccupancyGrid & current)
{
  const size_t cell_count = current.info.width * current.info.height;
  const bool shape_changed = previous_costmap_ && !isSameGridShape(*previous_costmap_, current);
  if (
    shape_changed || changed_until_.size() != cell_count ||
    change_intensity_.size() != cell_count) {
    changed_until_.assign(cell_count, rclcpp::Time{0, 0, RCL_ROS_TIME});
    change_intensity_.assign(cell_count, unknown_cost);
    return;
  }

  if (previous_costmap_ && !isSameOrigin(*previous_costmap_, current)) {
    remapPersistentState(current);
  }
}

bool CostmapDiffOverlay::isSameGridShape(const OccupancyGrid & lhs, const OccupancyGrid & rhs)
{
  const double lhs_resolution = static_cast<double>(lhs.info.resolution);
  const double rhs_resolution = static_cast<double>(rhs.info.resolution);

  return lhs.header.frame_id == rhs.header.frame_id && lhs.info.width == rhs.info.width &&
         lhs.info.height == rhs.info.height && std::abs(lhs_resolution - rhs_resolution) <= epsilon;
}

bool CostmapDiffOverlay::isSameOrigin(const OccupancyGrid & lhs, const OccupancyGrid & rhs)
{
  const double lhs_origin_x = static_cast<double>(lhs.info.origin.position.x);
  const double rhs_origin_x = static_cast<double>(rhs.info.origin.position.x);
  const double lhs_origin_y = static_cast<double>(lhs.info.origin.position.y);
  const double rhs_origin_y = static_cast<double>(rhs.info.origin.position.y);

  return std::abs(lhs_origin_x - rhs_origin_x) <= epsilon &&
         std::abs(lhs_origin_y - rhs_origin_y) <= epsilon;
}

void CostmapDiffOverlay::remapPersistentState(const OccupancyGrid & current)
{
  if (!previous_costmap_) {
    return;
  }

  const auto & previous = *previous_costmap_;
  const double resolution = static_cast<double>(current.info.resolution);
  const double prev_min_x = static_cast<double>(previous.info.origin.position.x);
  const double prev_min_y = static_cast<double>(previous.info.origin.position.y);
  const double prev_max_x = prev_min_x + static_cast<double>(previous.info.width) * resolution;
  const double prev_max_y = prev_min_y + static_cast<double>(previous.info.height) * resolution;

  std::vector<rclcpp::Time> remapped_until(changed_until_.size(), rclcpp::Time{0, 0, RCL_ROS_TIME});
  std::vector<int8_t> remapped_intensity(change_intensity_.size(), unknown_cost);

  for (size_t y = 0; y < current.info.height; ++y) {
    for (size_t x = 0; x < current.info.width; ++x) {
      const auto world_x =
        cellCenterCoord(static_cast<double>(current.info.origin.position.x), x, resolution);
      const auto world_y =
        cellCenterCoord(static_cast<double>(current.info.origin.position.y), y, resolution);

      if (!isWithinGrid(world_x, world_y, prev_min_x, prev_min_y, prev_max_x, prev_max_y)) {
        continue;
      }

      const auto prev_x = static_cast<size_t>((world_x - prev_min_x) / resolution);
      const auto prev_y = static_cast<size_t>((world_y - prev_min_y) / resolution);
      if (prev_x >= previous.info.width || prev_y >= previous.info.height) {
        continue;
      }

      const auto current_idx = toLinearIndex(x, y, current.info.width);
      const auto previous_idx = toLinearIndex(prev_x, prev_y, previous.info.width);
      remapped_until.at(current_idx) = changed_until_.at(previous_idx);
      remapped_intensity.at(current_idx) = change_intensity_.at(previous_idx);
    }
  }

  changed_until_ = std::move(remapped_until);
  change_intensity_ = std::move(remapped_intensity);
}

int8_t CostmapDiffOverlay::scaleDeltaToHeat(const int delta)
{
  // Logarithmic mapping: any change is visible, larger changes are brighter.
  static const auto log_max = std::log(delta_log_max);
  const auto t = std::log(delta_log_base + static_cast<double>(delta)) / log_max;
  const auto scaled = static_cast<int>(std::round(static_cast<double>(heat_min) + heat_span * t));
  return static_cast<int8_t>(std::clamp(scaled, 0, heat_max));
}

void CostmapDiffOverlay::updateChangedCells(const OccupancyGrid & current, const rclcpp::Time & now)
{
  if (!previous_costmap_) {
    return;
  }

  const auto & reference = *previous_costmap_;
  if (
    reference.header.frame_id != current.header.frame_id || reference.info.resolution <= 0.0 ||
    current.info.resolution <= 0.0) {
    return;
  }

  const double reference_resolution = static_cast<double>(reference.info.resolution);
  const double current_resolution = static_cast<double>(current.info.resolution);
  const double ref_min_x = static_cast<double>(reference.info.origin.position.x);
  const double ref_min_y = static_cast<double>(reference.info.origin.position.y);
  const double ref_max_x =
    ref_min_x + static_cast<double>(reference.info.width) * reference_resolution;
  const double ref_max_y =
    ref_min_y + static_cast<double>(reference.info.height) * reference_resolution;

  for (size_t y = 0; y < current.info.height; ++y) {
    for (size_t x = 0; x < current.info.width; ++x) {
      const auto current_idx = toLinearIndex(x, y, current.info.width);
      const int8_t current_cost = current.data.at(current_idx);
      if (current_cost == unknown_cost) {
        continue;
      }

      const auto world_x =
        cellCenterCoord(static_cast<double>(current.info.origin.position.x), x, current_resolution);
      const auto world_y =
        cellCenterCoord(static_cast<double>(current.info.origin.position.y), y, current_resolution);

      if (!isWithinGrid(world_x, world_y, ref_min_x, ref_min_y, ref_max_x, ref_max_y)) {
        // Ignore where previous and current working area do not overlap.
        continue;
      }

      const auto ref_x = static_cast<size_t>((world_x - ref_min_x) / reference_resolution);
      const auto ref_y = static_cast<size_t>((world_y - ref_min_y) / reference_resolution);
      if (ref_x >= reference.info.width || ref_y >= reference.info.height) {
        continue;
      }

      const auto ref_idx = toLinearIndex(ref_x, ref_y, reference.info.width);
      const int8_t ref_cost = reference.data.at(ref_idx);
      if (ref_cost == unknown_cost) {
        continue;
      }

      const auto current_cost_i = toCellCost(current_cost);
      const auto ref_cost_i = toCellCost(ref_cost);
      const auto delta = std::abs(current_cost_i - ref_cost_i);
      if (delta <= 0) {
        continue;
      }

      const bool already_active =
        changed_until_.at(current_idx) >= now && change_intensity_.at(current_idx) != unknown_cost;

      changed_until_.at(current_idx) = now + heatmap_window_;

      const auto base_heat = static_cast<int>(scaleDeltaToHeat(delta));
      change_intensity_.at(current_idx) =
        static_cast<int8_t>(boostedHeat(base_heat, already_active));
    }
  }
}

void CostmapDiffOverlay::publishHeatmap(const OccupancyGrid & current, const rclcpp::Time & now)
{
  if (!overlayed_costmap_pub_) {
    return;
  }

  auto heatmap = current;
  std::fill(heatmap.data.begin(), heatmap.data.end(), free_cost);

  for (size_t y = 0; y < current.info.height; ++y) {
    for (size_t x = 0; x < current.info.width; ++x) {
      const auto current_idx = toLinearIndex(x, y, current.info.width);
      const int8_t current_cost = current.data.at(current_idx);
      if (current_cost == unknown_cost) {
        // Ignore where current costmap does not exist.
        continue;
      }

      if (isHighlightedCost(current_cost)) {
        continue;
      }

      if (changed_until_.at(current_idx) < now) {
        change_intensity_.at(current_idx) = unknown_cost;
        continue;
      }

      heatmap.data.at(current_idx) = change_intensity_.at(current_idx);
    }
  }

  heatmap.header.stamp = current.header.stamp;
  overlayed_costmap_pub_->publish(heatmap);
}

}  // namespace autoware::costmap_generator
