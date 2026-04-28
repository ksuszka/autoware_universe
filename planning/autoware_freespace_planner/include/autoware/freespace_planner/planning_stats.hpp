// Copyright (c) 2026 Autonomous Systems Sp. z o.o.
// All rights reserved. Proprietary and confidential.
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

#ifndef AUTOWARE__FREESPACE_PLANNER__PLANNING_STATS_HPP_
#define AUTOWARE__FREESPACE_PLANNER__PLANNING_STATS_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace autoware::freespace_planner
{

struct PlanningStatsSummary
{
  using MillisecondsF = std::chrono::duration<double, std::milli>;

  uint64_t count = 0;
  MillisecondsF min = MillisecondsF::zero();
  MillisecondsF max = MillisecondsF::zero();
  MillisecondsF mean = MillisecondsF::zero();
  MillisecondsF stddev = MillisecondsF::zero();
};

class PlanningStatsCollector
{
public:
  using MillisecondsF = std::chrono::duration<double, std::milli>;

  void recordSample(const MillisecondsF & duration) noexcept;
  void reset() noexcept;
  uint64_t getCount() const noexcept { return count_; }
  PlanningStatsSummary getSummary() const noexcept;
  std::string formatSummary() const;

private:
  uint64_t count_ = 0;
  double mean_ms_ = 0.0;
  double M2_ms2_ = 0.0;
  double min_ms_ = 0.0;
  double max_ms_ = 0.0;
};

}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__PLANNING_STATS_HPP_
