#include "terrain_costmap/evidence_clearing.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

using terrain_costmap::EvidenceClearingParameters;
using terrain_costmap::makeEvidenceClearingRanges;
using terrain_costmap::Point;

constexpr double kPi = 3.14159265358979323846;

EvidenceClearingParameters geometry()
{
  EvidenceClearingParameters p;
  p.angle_min = 0.0;
  p.angle_increment = kPi / 4.0;
  p.bin_count = 8;
  return p;
}

Point polar(double range, double angle)
{
  return {range * std::cos(angle), range * std::sin(angle), 0.0};
}

bool allUnknown(const std::vector<float> & ranges)
{
  return std::all_of(ranges.begin(), ranges.end(), [](float range) {return std::isnan(range);});
}

TEST(EvidenceClearing, NoCurrentReturnsLeavesEveryDirectionUnknown)
{
  const auto p = geometry();
  EXPECT_TRUE(allUnknown(makeEvidenceClearingRanges({}, {}, p)));
  EXPECT_TRUE(allUnknown(makeEvidenceClearingRanges({}, {{1.0, 0.0, 0.5}}, p)));
  EXPECT_EQ(makeEvidenceClearingRanges({}, {}, p).size(), p.bin_count);
}

TEST(EvidenceClearing, OnlyActuallyObservedBinsProduceFiniteRays)
{
  const auto ranges = makeEvidenceClearingRanges({polar(3.0, 0.1)}, {}, geometry());
  ASSERT_EQ(ranges.size(), 8u);
  EXPECT_NEAR(ranges[0], 2.9, 1e-6);
  for (std::size_t index = 1; index < ranges.size(); ++index) {
    EXPECT_TRUE(std::isnan(ranges[index]));
  }
}

TEST(EvidenceClearing, RejectedTerrainReturnStillStopsTheRayAtANearWall)
{
  // The classifier rejected the near wall, so it is absent from obstacles.
  const auto ranges = makeEvidenceClearingRanges(
    {polar(4.0, 0.1), polar(0.7, 0.2)}, {}, geometry());
  EXPECT_NEAR(ranges[0], 0.6, 1e-6);
  EXPECT_LE(ranges[0], 0.7 - geometry().endpoint_margin);
}

TEST(EvidenceClearing, CurrentAndHistoricalObstaclesBothLimitObservedDirections)
{
  const auto ranges = makeEvidenceClearingRanges(
    {polar(4.0, 0.1), polar(3.0, 0.2), polar(0.9, 1.0)},
    {polar(2.0, 0.3), polar(1.5, 0.4), polar(2.0, 1.1), polar(0.8, 2.0)},
    geometry());
  EXPECT_NEAR(ranges[0], 1.4, 1e-6);
  EXPECT_NEAR(ranges[1], 0.8, 1e-6);
  EXPECT_TRUE(std::isnan(ranges[2]));
}

TEST(EvidenceClearing, NewFarReturnCanClearAnObstacleThatNoLongerHasEvidence)
{
  const auto before = makeEvidenceClearingRanges({polar(0.8, 0.1)}, {}, geometry());
  const auto after = makeEvidenceClearingRanges({polar(4.0, 0.1)}, {}, geometry());
  EXPECT_LT(before[0], 0.8);
  EXPECT_GT(after[0], 0.8);
  EXPECT_NEAR(after[0], 3.9, 1e-6);
}

TEST(EvidenceClearing, RayLimitsKeepMarginAtReturnsAndMaximumRange)
{
  auto p = geometry();
  p.range_min = 0.5;
  p.range_max = 4.0;
  p.endpoint_margin = 0.125;
  const auto ranges = makeEvidenceClearingRanges(
    {polar(0.4, 0.1), polar(2.0, 0.2), polar(0.625, 1.0),
      polar(4.0, 2.0), polar(20.0, 3.0)}, {}, p);
  EXPECT_TRUE(std::isnan(ranges[0]));  // A near return occludes the farther one.
  EXPECT_FLOAT_EQ(ranges[1], 0.5f);
  EXPECT_FLOAT_EQ(ranges[2], 3.875f);
  EXPECT_FLOAT_EQ(ranges[3], 3.875f);
}

TEST(EvidenceClearing, ObstaclesInsideMinimumRangeAlsoPreventClearing)
{
  const auto ranges = makeEvidenceClearingRanges(
    {polar(3.0, 0.1)}, {polar(0.2, 0.2)}, geometry());
  EXPECT_TRUE(allUnknown(ranges));
}

TEST(EvidenceClearing, SignedPiSeamUsesTheSameBinForReturnsAndObstacles)
{
  auto p = geometry();
  p.angle_min = -kPi;
  const auto ranges = makeEvidenceClearingRanges(
    {{-4.0, 0.0, 0.0}}, {{-0.8, -0.0, 0.0}}, p);
  EXPECT_NEAR(ranges[0], 0.7, 1e-6);
  EXPECT_TRUE(std::isnan(ranges.back()));
}

TEST(EvidenceClearing, PartialScanRejectsAnglesOutsideItsBins)
{
  auto p = geometry();
  p.bin_count = 2;
  const auto ranges = makeEvidenceClearingRanges(
    {polar(2.0, -0.01), polar(3.0, kPi / 4.0), polar(4.0, kPi / 2.0)}, {}, p);
  EXPECT_TRUE(std::isnan(ranges[0]));
  EXPECT_NEAR(ranges[1], 2.9, 1e-6);
}

TEST(EvidenceClearing, FloatScanGeometryDoesNotSplitAnExactBinBoundary)
{
  auto p = geometry();
  p.angle_min = static_cast<float>(-kPi);
  p.angle_increment = static_cast<float>(kPi / 360.0);
  p.bin_count = 720;
  const auto ranges = makeEvidenceClearingRanges(
    {{3.0, 0.0, 0.0}}, {{0.8, 0.0, 0.0}}, p);
  EXPECT_NEAR(ranges[360], 0.7, 1e-6);
  EXPECT_EQ(std::count_if(ranges.begin(), ranges.end(), [](float range) {
      return std::isfinite(range);
    }), 1);
}

TEST(EvidenceClearing, InvalidXYIsIgnoredAndZDoesNotAffectHorizontalEvidence)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto ranges = makeEvidenceClearingRanges(
    {{nan, 0.0, 0.0}, {0.0, inf, 0.0}, {2.0, 0.0, nan}},
    {{inf, 0.0, 0.0}}, geometry());
  EXPECT_NEAR(ranges[0], 1.9, 1e-6);
}

TEST(EvidenceClearing, FloatConversionNeverRoundsPastTheProtectedEndpoint)
{
  const auto p = geometry();
  const double hit = 1.23456789;
  const auto ranges = makeEvidenceClearingRanges({{hit, 0.0, 0.0}}, {}, p);
  EXPECT_LE(static_cast<double>(ranges[0]), hit - p.endpoint_margin);
  EXPECT_NEAR(ranges[0], hit - p.endpoint_margin, 1e-6);
}

TEST(EvidenceClearing, InvalidGeometryNeverProducesClearingEvidence)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto check = [](const EvidenceClearingParameters & p) {
      EXPECT_TRUE(allUnknown(makeEvidenceClearingRanges({{2.0, 0.0, 0.0}}, {}, p)));
    };
  auto p = geometry();
  p.range_min = -0.1;
  check(p);
  p = geometry(); p.range_min = nan; check(p);
  p = geometry(); p.range_max = p.range_min; check(p);
  p = geometry(); p.range_max = inf; check(p);
  p = geometry(); p.angle_min = nan; check(p);
  p = geometry(); p.angle_increment = 0.0; check(p);
  p = geometry(); p.angle_increment = -0.1; check(p);
  p = geometry(); p.angle_increment = inf; check(p);
  p = geometry(); p.bin_count = 10; check(p);
  p = geometry(); p.endpoint_margin = -0.1; check(p);
  p = geometry(); p.endpoint_margin = nan; check(p);
  p = geometry(); p.bin_count = 0;
  EXPECT_TRUE(makeEvidenceClearingRanges({{2.0, 0.0, 0.0}}, {}, p).empty());
}

}  // namespace
