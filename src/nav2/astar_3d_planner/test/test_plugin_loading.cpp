#include <gtest/gtest.h>

#include <memory>

#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

TEST(Astar3DPlugin, IsDiscoverableAndConstructible)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  ASSERT_TRUE(loader.isClassAvailable("astar_3d_planner/Astar3DPlanner"));
  auto planner =
    loader.createSharedInstance("astar_3d_planner/Astar3DPlanner");
  EXPECT_NE(planner, nullptr);
}
