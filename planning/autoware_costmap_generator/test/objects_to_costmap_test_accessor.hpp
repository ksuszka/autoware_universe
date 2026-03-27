// Copyright 2024 The Autoware Contributors
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

#ifndef OBJECTS_TO_COSTMAP_TEST_ACCESSOR_HPP_
#define OBJECTS_TO_COSTMAP_TEST_ACCESSOR_HPP_

#include <autoware/costmap_generator/utils/objects_to_costmap.hpp>

#include <grid_map_core/grid_map_core.hpp>

namespace autoware::costmap_generator::test
{

class ObjectsToCostmapTestAccessor
{
public:
  explicit ObjectsToCostmapTestAccessor(ObjectsToCostmap & obj) : obj_(obj) {}

  static grid_map::Polygon makePolygonFromObjectConvexHull(
    const std_msgs::msg::Header & header,
    const autoware_perception_msgs::msg::PredictedObject & in_object,
    const double expand_polygon_size)
  {
    return ObjectsToCostmap::makePolygonFromObjectConvexHull(header, in_object, expand_polygon_size);
  }

private:
  ObjectsToCostmap & obj_;
};

}  // namespace autoware::costmap_generator::test

#endif  // OBJECTS_TO_COSTMAP_TEST_ACCESSOR_HPP_
