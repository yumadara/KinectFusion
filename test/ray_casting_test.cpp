#include <gtest/gtest.h>

#include <ray_casting.h>
#include <type_definitions.h>
#include <camera.h>
#include <voxel.h>

namespace kinect_fusion {

/// @brief Basic test to check that data pipeline works
TEST(RayCastingTest, CheckCastingConstruction) {
    Camera camera;
    Voxel TSDF(3,4,5,0,0,0);
    RayCasting rayCastingRayCasting(TSDF, camera);
}


TEST(RayCastingTest, CheckFill_Pixel) {
	Camera camera;
    Voxel TSDF(100,100,200,50,50,0);
    for (int i =20;i<56;i++){
        float loc = 5.* i;
        float distance = 1;
        TSDF.setDistance(0,0,loc, distance);
        TSDF.setDistance(-1,0,loc, distance);
        TSDF.setDistance(6,0,loc, distance);
        TSDF.setDistance(0,-1,loc, distance);
        TSDF.setDistance(0,6,loc, distance);
        TSDF.setDistance(-1,-1,loc, distance);
        TSDF.setDistance(6,6,loc, distance);
        TSDF.setDistance(-1,6,loc, distance);
        TSDF.setDistance(6,-1,loc, distance);

        

    }
    for (int i =56;i<=96;i++){
        float loc = 5.* i;
        float distance = (TSDF.truncateDistance-(i-56)*5.)/ TSDF.truncateDistance;
        TSDF.setDistance(0,0,loc, distance);
        TSDF.setDistance(-1,0,loc, distance);
        TSDF.setDistance(6,0,loc, distance);
        TSDF.setDistance(0,-1,loc, distance);
        TSDF.setDistance(0,6,loc, distance);
        TSDF.setDistance(-1,-1,loc, distance);
        TSDF.setDistance(6,6,loc, distance);
        TSDF.setDistance(-1,6,loc, distance);
        TSDF.setDistance(6,-1,loc, distance);
        
    }
    for (int i =97;i<=137;i++){
        float loc = 5.* i;
        float distance = (-(i-96)*5./TSDF.truncateDistance);
        TSDF.setDistance(0,0,loc, distance);
        TSDF.setDistance(-1,0,loc, distance);
        TSDF.setDistance(6,0,loc, distance);
        TSDF.setDistance(0,-1,loc, distance);
        TSDF.setDistance(0,6,loc, distance);
        TSDF.setDistance(-1,-1,loc, distance);
        TSDF.setDistance(6,6,loc, distance);
        TSDF.setDistance(-1,6,loc, distance);
        TSDF.setDistance(6,-1,loc, distance);
        
    }
    
    RayCasting rayCastingRayCasting(TSDF, camera);
    rayCastingRayCasting.fill_pixel(0+ camera.originXInPixel, 0+ camera.originYInPixel);
    Map2Df depthMap = rayCastingRayCasting.getDepthMap();
    EXPECT_NEAR(depthMap.get(0+ camera.originYInPixel,0+ camera.originXInPixel)*1000, 480.,1.);
    
}

} // namespace kinect_fusion



