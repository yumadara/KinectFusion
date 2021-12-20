#pragma once


#include "Eigen.h"
#include <math.h> 
#include <algorithm> 

class Voxel {
public:
    

    Voxel(int num_x, int num_y, int num_z,
    int origin_x, int origin_y, int origin_z) {
    Distance =   new float[num_x * num_y * num_z];
    Weight =  new float[num_x * num_y * num_z];
    std::fill_n(Distance, num_x * num_y * num_z, defaultDistance); 
    std::fill_n(Weight,num_x * num_y * num_z , defaultWeight); 

    originX = origin_x;
    originY = origin_y;
    originZ = origin_z;
    numX = num_x;
    numY = num_y;
    numZ = num_z;
    

    }

    ~Voxel(void){
        delete [] Distance;
        delete [] Weight;
    }
    bool isInBound(int ordX, int ordY, int ordZ){
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
    float getDistanceFromInt(int ordX, int ordY, int ordZ){
        if (! isInBound(ordX, ordY, ordZ)){
            return defaultDistance;
        }
        return Distance[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY];
    }
    float getWeightFromInt(int ordX, int ordY, int ordZ){
        if (! isInBound(ordX, ordY, ordZ)){
            return defaultWeight;
        }
        return Weight[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY];
    }
    int ordFromCont(float ordCont){
        return floor(ordCont/minimumResolution);
    }
    float getDistance(float ordXf, float ordYf, float ordZf){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        return getDistanceFromInt(ordX, ordY, ordZ);
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
        if (! isInBound(ordX, ordY, ordZ)){
            return false;
        }
        Distance[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY] = newDistance;
        return true;
    }
    bool setWeight(float ordXf, float ordYf, float ordZf, float newWeight){
        int ordX = ordFromCont(ordXf);
        int ordY = ordFromCont(ordYf);
        int ordZ = ordFromCont(ordZf);
        if (! isInBound(ordX, ordY, ordZ)){
            return false;
        }
        Weight[(ordX + originX) + (ordY + originY) * numX + (ordZ + originZ) * numX * numY] = newWeight;
        return true;
    }









private:
    float* Distance ;
    float* Weight;
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



};
 
