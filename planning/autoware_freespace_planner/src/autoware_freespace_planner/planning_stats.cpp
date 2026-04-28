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

#include "autoware/freespace_planner/planning_stats.hpp"

#include <algorithm>
#include <cmath>

#include <fmt/format.h>

namespace autoware::freespace_planner
{

void PlanningStatsCollector::recordSample(const MillisecondsF & duration) noexcept
{
  const double duration_ms = duration.count();
  ++count_;

  const double delta = duration_ms - mean_ms_;
  mean_ms_ += delta / static_cast<double>(count_);
  const double delta2 = duration_ms - mean_ms_;
  M2_ms2_ += delta * delta2;

  if (count_ == 1U) {
    min_ms_ = duration_ms;
    max_ms_ = duration_ms;
  } else {
    min_ms_ = std::min(min_ms_, duration_ms);
    max_ms_ = std::max(max_ms_, duration_ms);
  }
}

void PlanningStatsCollector::reset() noexcept
{
  count_ = 0;
  mean_ms_ = 0.0;
  M2_ms2_ = 0.0;
  min_ms_ = 0.0;
  max_ms_ = 0.0;
}

PlanningStatsSummary PlanningStatsCollector::getSummary() const noexcept
{
  PlanningStatsSummary summary;
  summary.count = count_;

  if (count_ == 0U) {
    return summary;
  }

  summary.min = MillisecondsF{min_ms_};
  summary.max = MillisecondsF{max_ms_};
  summary.mean = MillisecondsF{mean_ms_};
  summary.stddev =
    (count_ >= 2U) ? MillisecondsF{std::sqrt(M2_ms2_ / static_cast<double>(count_ - 1U))}
                   : MillisecondsF::zero();
  return summary;
}

std::string PlanningStatsCollector::formatSummary() const
{
  const auto summary = getSummary();
  if (summary.count == 0U) {
    return "count=0";
  }

  return fmt::format(
    "count={} min={:.2f}ms max={:.2f}ms mean={:.2f}ms stddev={:.2f}ms", summary.count,
    summary.min.count(), summary.max.count(), summary.mean.count(), summary.stddev.count());
}

}  // namespace autoware::freespace_planner
