#pragma once
#include <Ray.h>
#include <Voxel.h>

#include <Camera.h>
#include <Surface.h>

namespace kinect_fusion {

class RayCasting {
    public:
    RayCasting(Voxel TSDF, Camera camera, float minDistance, float maxDistance){
        RayCasting(TSDF, camera);
        this->minDistance = minDistance;
        this->maxDistance = maxDistance;
    }
    RayCasting(Voxel TSDF, Camera camera){
        this->camera = camera;
        this->TSDF = TSDF;
        Surface surface(camera.getHeight(), camera.getWidth());
        this->surface = surface;
        this->minDistance = 400.;
        this->maxDistance = 8000.;
    }
    void do_work(){
        for (int j = 0;j< this->surface.getHeight();j++){
            for (int i = 0; i< this->surface.getWidth(); i++){
                fill_pixel(i,j);
            }
        }
    }
    bool fill_pixel(int XInPixel, int YInPixel){
        Vector3f direction = camera.inverseCalibrationMatrix * Vector3f(float(XInPixel), float(XInPixel), 1.);
        float stepLength =  TSDF.truncateDistance;
        
        Ray ray = Ray(stepLength, this->minDistance, this->maxDistance, direction);
        float currF = TSDF.defaultDistance;
        float lastF = TSDF.defaultDistance;
        while(ray.isInBound()){
            Vector3f currLocationCamera = ray.getCurrLocation();
            Vector3f LastLocationCamera = ray.getLastLocation();
            Vector3f currLocationWorld = camera.cameraToWorld(currLocationCamera);
            Vector3f LastLocationWorld = camera.cameraToWorld(LastLocationCamera);
            if (!TSDF.isInBound(currLocationWorld(0), currLocationWorld(1), currLocationWorld(2))){
                return false;
            }
            lastF = currF;
            currF = TSDF.getDistance(currLocationWorld(0), currLocationWorld(1), currLocationWorld(2));
            if (currF>=0 && lastF<=0 &&  TSDF.isKnown(lastF)){
                return false;
            }
            if (currF<=0 && lastF>=0){
                Vector3f vertexCamera = LastLocationCamera - (currLocationCamera-LastLocationCamera) * lastF /(currF -lastF);
                Vector3f normalWorld = TSDF.getNormal(LastLocationWorld(0), LastLocationWorld(1), LastLocationWorld(2));
                //Vector3f vertexCamera = camera.worldToCameraVector(vertexWorld);
                Vector3f normalCamera = camera.worldToCameraVector(normalWorld);
                surface.setNormal(XInPixel, YInPixel, normalCamera);
                surface.setVertex(XInPixel, YInPixel, vertexCamera);
                return true;
            }
            
            else{
                ray.step();
            }       
        }

    }
    Surface& getSurface() {
        return surface;
    }
    Camera& getCamera() {
        return camera;
    }

    private:
    Voxel TSDF;
    
    Camera camera;
    Surface surface;
    float minDistance;
    float maxDistance;
};

} // namespace kinect_fusion