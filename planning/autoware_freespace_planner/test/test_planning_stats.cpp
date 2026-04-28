// Copyright 2024 TIER IV, Inc.
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

#include <gtest/gtest.h>

#include <chrono>

namespace autoware::freespace_planner
{
/// Test suite for PlanningStatsCollector
class TestPlanningStatsCollector : public ::testing::Test
{
protected:
  PlanningStatsCollector stats_;
};

/// Test empty stats returns correct initial values
TEST_F(TestPlanningStatsCollector, EmptyStats)
{
  EXPECT_EQ(stats_.getCount(), 0UL);
  EXPECT_EQ(stats_.formatSummary(), "count=0");
}

/// Test single sample statistics
TEST_F(TestPlanningStatsCollector, SingleSample)
{
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{10.5});
  EXPECT_EQ(stats_.getCount(), 1UL);
  const auto summary = stats_.formatSummary();
  EXPECT_NE(summary.find("count=1"), std::string::npos);
  EXPECT_NE(summary.find("min=10.50"), std::string::npos);
  EXPECT_NE(summary.find("max=10.50"), std::string::npos);
  EXPECT_NE(summary.find("mean=10.50"), std::string::npos);
}

/// Test multiple samples compute correct statistics
TEST_F(TestPlanningStatsCollector, MultipleSamples)
{
  // Add 4 samples: 10, 20, 30, 40 (mean = 25, stddev = 11.18)
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{10.0});
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{20.0});
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{30.0});
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{40.0});

  EXPECT_EQ(stats_.getCount(), 4UL);
  const auto summary = stats_.formatSummary();
  EXPECT_NE(summary.find("count=4"), std::string::npos);
  EXPECT_NE(summary.find("min=10.00"), std::string::npos);
  EXPECT_NE(summary.find("max=40.00"), std::string::npos);
  EXPECT_NE(summary.find("mean=25.00"), std::string::npos);
  // stddev of [10, 20, 30, 40] is sqrt(166.67) ≈ 12.91
  EXPECT_NE(summary.find("stddev=12"), std::string::npos);
}

/// Test reset clears statistics
TEST_F(TestPlanningStatsCollector, Reset)
{
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{100.0});
  EXPECT_EQ(stats_.getCount(), 1UL);

  stats_.reset();
  EXPECT_EQ(stats_.getCount(), 0UL);
  EXPECT_EQ(stats_.formatSummary(), "count=0");
}

/// Test two consecutive resets work correctly
TEST_F(TestPlanningStatsCollector, ConsecutiveRecordsAfterReset)
{
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{50.0});
  stats_.recordSample(PlanningStatsCollector::MillisecondsF{60.0});
  EXPECT_EQ(stats_.getCount(), 2UL);

  stats_.reset();
  EXPECT_EQ(stats_.getCount(), 0UL);

  stats_.recordSample(PlanningStatsCollector::MillisecondsF{100.0});
  EXPECT_EQ(stats_.getCount(), 1UL);
  const auto summary = stats_.formatSummary();
  EXPECT_NE(summary.find("mean=100.00"), std::string::npos);
}

}  // namespace autoware::freespace_planner
