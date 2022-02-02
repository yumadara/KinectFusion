#pragma once
#include "Eigen.h"

namespace kinect_fusion {

class Ray {
    public:
    Ray(float stepLength, float minDistance, float maxDistance, Vector3f direction){
        this->stepLength = stepLength;
        this->minDistance = minDistance;
        this->maxDistance = maxDistance;
        this->direction = direction / direction.norm();
        //this->currLocation = this->direction * minDistance;
        this->currLocation = this->direction * minDistance / (this->direction).norm();
        this->lastLocation = this->currLocation;
        

        
    }
    ~Ray(){}

    void step(){
        this->lastLocation = this->currLocation;
        this->currLocation = this->currLocation + this->direction * this->stepLength;



    }
    Vector3f getCurrLocation(){
        return this->currLocation;
    }
    Vector3f getLastLocation(){
        return this->lastLocation;
    }
    bool isInBound(){
        return this->currLocation.norm()<=this->maxDistance;
    }
    void setstepLength(float stepLength){
        this->stepLength = stepLength;
    }

    private:
    float stepLength;
    float minDistance;
    float maxDistance;
    Vector3f direction;
    Vector3f currLocation;
    Vector3f lastLocation;
};

} // namespace kinect_fusion