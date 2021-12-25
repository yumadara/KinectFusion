#pragma once
#include <cassert>
#include "Eigen.h"


class Camera{
    public:
    Camera(Vector3f translation, Matrix3f rotation, Matrix3f inverseCalibrationMatrix,
    int pictureHeightInPixel, int pictureWidthInPixel,
    int originXInPixel, int originYInPixel){
        init(translation,rotation, inverseCalibrationMatrix,
        pictureHeightInPixel, pictureWidthInPixel,
        originXInPixel, originYInPixel);
        
    }
    Camera(){
        Vector3f translation = Vector3f(0.,0.,0.);
        Matrix3f rotation = Matrix3f::Identity();
        Matrix3f inverseCalibrationMatrix = Matrix3f::Identity();
        int pictureHeightInPixel = 100;
        int pictureWidthInPixel = 100;
        int originXInPixel = 0;
        int originYInPixel = 0;
        init(translation,rotation, inverseCalibrationMatrix,
        pictureHeightInPixel, pictureWidthInPixel,
        originXInPixel, originYInPixel);
        
    }
    void init(Vector3f translation, Matrix3f rotation, Matrix3f inverseCalibrationMatrix,
    int pictureHeightInPixel, int pictureWidthInPixel,
    int originXInPixel, int originYInPixel){
        assert (rotationIsValid(rotation));
        this->rotation = rotation;
        this->translation = translation;
        this->inverseCalibrationMatrix = inverseCalibrationMatrix;
        this->pictureHeightInPixel = pictureHeightInPixel;
        this->pictureWidthInPixel = pictureWidthInPixel;
        this->originXInPixel = originXInPixel;
        this->originYInPixel = originYInPixel;
    }
    bool rotationIsValid(Matrix3f rotation){
        return (rotation * rotation.transpose() - Matrix3f::Identity()).isZero();
    }
    Matrix3f getRotation(){
        return this->rotation;
    }
    Vector3f cameraToWorld(Vector3f cameraFrame){
        return rotation * cameraFrame + translation;
    }
    Vector3f getTranslation(){
        return this->translation;
    }
    int pictureHeightInPixel;
    int pictureWidthInPixel;
    int originXInPixel;
    int originYInPixel;
    Matrix3f inverseCalibrationMatrix;
    private:
    Vector3f translation;
    Matrix3f rotation;
    
    

    



};