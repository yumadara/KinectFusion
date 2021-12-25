#pragma once


#include "Eigen.h"
#include <math.h> 
#include <algorithm> 

class Voxel {
public:
    

    Voxel(int num_x, int num_y, int num_z,
    int origin_x, int origin_y, int origin_z) {
    this->Distance =   new float[num_x * num_y * num_z];
    this->Weight =  new float[num_x * num_y * num_z];
    std::fill_n(this->Distance, num_x * num_y * num_z, defaultDistance); 
    std::fill_n(this->Weight,num_x * num_y * num_z , defaultWeight); 

    originX = origin_x;
    originY = origin_y;
    originZ = origin_z;
    numX = num_x;
    numY = num_y;
    numZ = num_z;
    }
    

    
    Voxel() {
        Voxel(3,4,5,0,0,0);
    }
    
    

    ~Voxel(void){
        delete [] Distance;
        delete [] Weight;
    }
    bool isInBoundInt(int ordX, int ordY, int ordZ){
        if (ordX + originX >= numX || ordX + originX < 0){
            return false;
        }
        if (ordY + originY >= numY || ordY + originY < 0){
            return false;
        }
        if (ordZ + originZ >= numZ || ordZ + originZ < 0){
            return false;
        }
        return true;
    }
    bool isInBound(float ordXf, float ordYf, float ordZf){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        return isInBoundInt(ordX, ordY, ordZ);
    }
    float getDistanceFromInt(int ordX, int ordY, int ordZ){
        if (! isInBoundInt(ordX, ordY, ordZ)){
            return defaultDistance;
        }
        return Distance[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY];
    }
    float getWeightFromInt(int ordX, int ordY, int ordZ){
        if (! isInBoundInt(ordX, ordY, ordZ)){
            return defaultWeight;
        }
        return Weight[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY];
    }
    int ordFromCont(float ordCont){
        
        float k  = ordCont/minimumResolution;
        int r = std::floor(ordCont/minimumResolution);
        return r;
    }
    float ContFromOrd(int ord){
        return ord*minimumResolution;
    }
    float getDistance(float ordXf, float ordYf, float ordZf){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        return getDistanceFromInt(ordX, ordY, ordZ);
    }
    Vector3f getNormal(float ordXf, float ordYf, float ordZf){
        Vector3f normal = Vector3f(0.,0.,0.);
        if(! isKnown(getDistance(ordXf, ordYf, ordZf))){
            return Vector3f(0.,0.,0.);
        }
        
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        float X_m1 = getDistanceFromInt(ordX-1, ordY, ordZ);
        float X_p1 = getDistanceFromInt(ordX+1, ordY, ordZ);
        if (isKnown(X_m1) && isKnown(X_p1)){
            normal(0) = (X_p1 - X_m1)/ 2./minimumResolution;
        }
        else {
            return Vector3f(0.,0.,0.);
        }
        float Y_m1 = getDistanceFromInt(ordX, ordY-1, ordZ);
        float Y_p1 = getDistanceFromInt(ordX, ordY+1, ordZ);
        if (isKnown(Y_m1) && isKnown(Y_p1)){
            normal(1) = (Y_p1 - Y_m1)/ 2./minimumResolution;
        }
        else {
            return Vector3f(0.,0.,0.);
        }
        float Z_m1 = getDistanceFromInt(ordX, ordY, ordZ-1);
        float Z_p1 = getDistanceFromInt(ordX, ordY, ordZ+1);
        if (isKnown(Z_m1) && isKnown(Z_p1)){
            normal(2) = (Z_p1 - Z_m1)/ 2./minimumResolution;
        }
        else {
            return Vector3f(0.,0.,0.);
        }
        return normal;



    }
    float getWeight(float ordXf, float ordYf, float ordZf){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        return getWeightFromInt(ordX, ordY, ordZ);
    }

    bool setDistance(float ordXf, float ordYf, float ordZf, float newDistance){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        if (! isInBoundInt(ordX, ordY, ordZ)){
            return false;
        }
        Distance[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY] = truncate(newDistance);
        return true;
    }
    bool setWeight(float ordXf, float ordYf, float ordZf, float newWeight){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        if (! isInBoundInt(ordX, ordY, ordZ)){
            return false;
        }
        Weight[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY] = newWeight;
        return true;
    }
    bool isKnown(float distanceValue){
        return distanceValue> -truncateDistance && distanceValue<= truncateDistance;
    }
    float truncate(float distance){
        if (isKnown(distance)){
            return distance;
        }
        if (distance<=-truncateDistance){
            return defaultDistance;
        }
        if (distance> truncateDistance){
            return truncateDistance;
        }
    }





    float defaultDistance = -100.;
    float defaultWeight = -1.;
    float truncateDistance = 30.;
    float minimumResolution = 5.;
    int originX;
    int originY;
    int originZ;
    int numX;
    int numY;
    int numZ;





private:
    float* Distance ;
    float* Weight;
    


};
 
