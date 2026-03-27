// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__POSE_FMT_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__POSE_FMT_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.hpp>

#include <fmt/core.h>

namespace fmt
{
template <>
struct formatter<geometry_msgs::msg::Pose>
{
  constexpr auto parse(format_parse_context & ctx) { return ctx.begin(); }

  template <typename FormatContext>
  constexpr auto format(const geometry_msgs::msg::Pose & p, FormatContext & ctx)
  {
    return format_to(
      ctx.out(), "x:{}, y:{}, yaw:{}", p.position.x, p.position.y, tf2::getYaw(p.orientation));
  }
};
}  // namespace fmt

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__POSE_FMT_HPP_