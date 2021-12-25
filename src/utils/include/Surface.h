#pragma once
#include <cassert>
#include "Eigen.h"
#include <Camera.h>
class Surface{
    public:
    Surface(Camera camera){
        this->camera = camera;
        Vector3f defaultValue = Vector3f::Zero();
        vertexMap.resize(camera.pictureHeightInPixel * camera.pictureWidthInPixel);
        normalMap.resize(camera.pictureHeightInPixel * camera.pictureWidthInPixel);
        
        
        
        std::fill(vertexMap.begin(), vertexMap.end(), defaultValue);
        std::fill(normalMap.begin(), normalMap.end(), defaultValue);


    }
    Surface(){
       
    }

    void setVertex(int X, int Y, Vector3f value){
        this->vertexMap[Y * camera.pictureWidthInPixel + X ] = value;
    }
    void setNormal(int X, int Y, Vector3f value){
        this->normalMap[Y  * camera.pictureWidthInPixel + X ] = value;
    }
    Vector3f getVertex(int X, int Y){
        return this->vertexMap[Y * camera.pictureWidthInPixel + X ];
    }
    Vector3f getNormal(int X, int Y){
        return this->normalMap[Y * camera.pictureWidthInPixel + X ];
    }
    int getHeight(){
        return this->camera.pictureHeightInPixel;
    }

    int getWidth(){
        return this->camera.pictureWidthInPixel;
    }

    std::vector<Vector3f> getVertexMap(){
        return this-> vertexMap;
    }
    std::vector<Vector3f> getNormalMap(){
        return this-> normalMap;
    }


    private:
    

    std::vector<Vector3f> vertexMap;
    std::vector<Vector3f> normalMap;
    Camera camera;

};