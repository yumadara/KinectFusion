#include <gtest/gtest.h>

#include <Voxel.h>

namespace kinect_fusion {


TEST(VoxelTest, CheckVoxelConstruction) {
	Voxel vox(3,4,5,0,0,0);
}


TEST(VoxelTest, CheckIsInBound) {
	Voxel vox(3,4,5,0,0,0);
    EXPECT_TRUE(vox.isInBound(0.,0.,0.));
    EXPECT_TRUE(vox.isInBound(2* vox.minimumResolution,3* vox.minimumResolution,4* vox.minimumResolution));
    EXPECT_FALSE(vox.isInBound(2* vox.minimumResolution,3* vox.minimumResolution,5* vox.minimumResolution));
    EXPECT_FALSE(vox.isInBound(-1* vox.minimumResolution,3* vox.minimumResolution,5* vox.minimumResolution));
}

TEST(VoxelTest, CheckGetter) {
	Voxel vox(3,4,5,0,0,0);
    
    EXPECT_EQ(vox.getDistance(3.,3.,3.), -100.);
    EXPECT_EQ(vox.getWeight(3.,3.,3.), -1.);
}
TEST(VoxelTest, CheckSetter) {
	Voxel vox(3,4,5,2,3,4);
    vox.setDistance(-1.,3.,3., 20.);
    vox.setWeight(-1.,3.,3., 1.);
    EXPECT_EQ(vox.getDistance(-1.,3.,3.), 20.);
    EXPECT_EQ(vox.getWeight(-1.,3.,3.), 1.);
}
TEST(VoxelTest, CheckIsKnown){
    Voxel vox(3,4,5,0,0,0);
    EXPECT_TRUE(vox.isKnown(20.));
    EXPECT_FALSE(vox.isKnown(-30.));
    EXPECT_TRUE(vox.isKnown(30.));
}
TEST(VoxelTest, CheckOrdFromCont){
    Voxel vox(3,4,5,0,0,0);
    EXPECT_EQ(vox.ordFromCont(1.),0);
    EXPECT_EQ(vox.ordFromCont(-1.),-1);
    EXPECT_EQ(vox.ordFromCont(6.),1);
}
TEST(VoxelTest, CheckGetNormal) {
	Voxel vox(3,4,5,1,2,3);
    Vector3f targetNormal = Vector3f(0,0,0);
    EXPECT_NEAR((vox.getNormal(3.,3.,3.)-targetNormal).array().abs().sum(), 0., 0.0001);
    vox.setDistance(3.,3.,3., 0.);
    vox.setDistance(-1.,3.,3., -20.);
    vox.setDistance(6.,3.,3., 20.);
    vox.setDistance(3.,-1.,3., 0.);
    vox.setDistance(3.,6.,3., 0.);
    vox.setDistance(3.,3.,-1., 0.);
    vox.setDistance(3.,3.,6., 0.);
    

    EXPECT_NEAR(vox.getNormal(3.,3.,3.)(0)-4, 0., 0.0001);
    EXPECT_NEAR(vox.getNormal(3.,3.,3.)(1)-0, 0., 0.0001);
    EXPECT_NEAR(vox.getNormal(3.,3.,3.)(2)-0, 0., 0.0001);
}

} // namespace kinect_fusion
