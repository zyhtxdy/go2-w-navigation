#include <gtest/gtest.h>

#include "behaviortree_cpp_v3/bt_factory.h"

TEST(TerrainPathValidityBtPlugin, IsLoadableAndRegistersNode)
{
  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(TERRAIN_BT_PLUGIN_LIBRARY));
  EXPECT_EQ(factory.manifests().count("IsTerrainPathValid"), 1u);
}
