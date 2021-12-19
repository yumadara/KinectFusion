#include <gtest/gtest.h>

#include <Voxel.h>


/// @brief Basic test to check that data pipeline works
TEST(VoxelTest, CheckVoxelConstruction) {
	Voxel vox(3,4,5,0,0,0);
}


TEST(VoxelTest, CheckIsInBound) {
	Voxel vox(3,4,5,0,0,0);
    EXPECT_TRUE(vox.isInBound(0,0,0));
    EXPECT_TRUE(vox.isInBound(2,3,4));
    EXPECT_FALSE(vox.isInBound(2,3,5));
    EXPECT_FALSE(vox.isInBound(-1,3,5));
}

TEST(VoxelTest, CheckGetter) {
	Voxel vox(3,4,5,0,0,0);
    
    EXPECT_EQ(vox.getDistance(3.,3.,3.), -100.);
    EXPECT_EQ(vox.getWeight(3.,3.,3.), -1.);
}
TEST(VoxelTest, CheckSetter) {
	Voxel vox(3,4,5,0,0,0);
    vox.setDistance(3.,3.,3., 20.);
    vox.setWeight(3.,3.,3., 1.);
    EXPECT_EQ(vox.getDistance(3.,3.,3.), 20.);
    EXPECT_EQ(vox.getWeight(3.,3.,3.), 1.);
}
