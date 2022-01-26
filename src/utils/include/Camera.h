#pragma once

#include <cassert>

#include "Eigen.h"

namespace kinect_fusion {

class Camera {
    public:
    Camera(MatrixXf poseEstimation, Matrix3f CalibrationMatrix,
    int pictureHeightInPixel, int pictureWidthInPixel,
    int originXInPixel, int originYInPixel){
        Vector3f translation = poseEstimation.block(0,3,3,1);
        Matrix3f rotation = poseEstimation.block(0,0,3,3);

        init(translation,rotation, CalibrationMatrix,
        pictureHeightInPixel, pictureWidthInPixel,
        CalibrationMatrix(0, 2), CalibrationMatrix(1, 2));
    }

    Camera(){
        Vector3f translation = Vector3f(0.,0.,0.);
        Matrix3f rotation = Matrix3f::Identity();
        Matrix3f CalibrationMatrix = Matrix3f::Identity();
        std::size_t pictureHeightInPixel = 100;
        std::size_t pictureWidthInPixel = 100;
        std::size_t originXInPixel = 0;
        std::size_t originYInPixel = 0;

        init(translation,rotation, CalibrationMatrix,
        pictureHeightInPixel, pictureWidthInPixel,
        originXInPixel, originYInPixel);
        
    }
    void init(Vector3f translation, Matrix3f rotation, Matrix3f CalibrationMatrix,
        int pictureHeightInPixel, int pictureWidthInPixel,
        int originXInPixel, int originYInPixel) {
        assert (abs(rotation.determinant()-1) <0.01 );
        
        this->rotation = rotation;
        this->translation = translation;
        this->CalibrationMatrix = CalibrationMatrix;
        this->pictureHeightInPixel = pictureHeightInPixel;
        this->pictureWidthInPixel = pictureWidthInPixel;
        this->originXInPixel = CalibrationMatrix(0, 2);
        this->originYInPixel = CalibrationMatrix(1, 2);
    }
    bool rotationIsValid(Matrix3f rotation){
        return (rotation * rotation.transpose() - Matrix3f::Identity()).isZero();
    }
    Matrix3f getRotation(){
        return this->rotation;
    }
    //translate points from camera to world
    Vector3f cameraToWorld(Vector3f cameraFrame){
        return rotation * cameraFrame + translation;
    }
    //translate points from world to camera
    Vector3f worldToCamera(Vector3f worldFrame){
        return rotation.inverse() * (worldFrame - translation);
    }
    //translate vectors from camera to world
    Vector3f cameraToWorldVector(Vector3f cameraFrame){
        return rotation * cameraFrame;
    }
    //translate vectors from world to camera
    Vector3f worldToCameraVector(Vector3f cameraFrame){
        return rotation.inverse() * cameraFrame;
    }
    Vector3f getTranslation(){
        return this->translation;
    }
    inline std::size_t getHeight() {
        return pictureHeightInPixel;
    }
    inline std::size_t getWidth() {
        return pictureWidthInPixel;
    }


    std::size_t pictureHeightInPixel;
    std::size_t pictureWidthInPixel;
    std::size_t originXInPixel;
    std::size_t originYInPixel;
    Matrix3f CalibrationMatrix;
    private:
    Vector3f translation;
    Matrix3f rotation;
};

} // namespace kinect_fusion