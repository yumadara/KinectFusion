#include <iostream>
#include "Eigen.h"
#include"Voxel.h"
#include"virtual_sensor.h"
#include"type_definitions.h"
#include <chrono>
#include <cassert>

namespace kinect_fusion {
    //camera coordinate -> pixel coordinate
    Vector2f cameraToScreen(const Vector3f& p_k, Matrix3f depthIntrinsics)
    {
        Vector3f q = depthIntrinsics*p_k;
        Vector2f x = Vector2f(q.x()/q.z(), q.y()/q.z());
        return x;    
    }
    //input:(x, y)
    //output: (x, y, 1)
    VectorXf homogenisation(const VectorXf& dehomo_coord)
    {
        VectorXf homo_coord(dehomo_coord.size()+1);
        homo_coord.segment(0, dehomo_coord.size()) << dehomo_coord;
        homo_coord[dehomo_coord.size()] = 1;
        return homo_coord;

    }
    //quation (9)
    float SDF_truncation(float& x, const float& mu, const float& default_sdf)
    {   
        int sign = x>0? 1:-1;
        if(x > -mu)
        {
            return fmin(1, abs(x/mu))*sign;
        }
        else
            return default_sdf;
    }
    //equation (8)
    Vector2i globalToScreen(const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p_g ,float x_z_limit, float y_z_limit)
    {
        Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
        Vector4f p = homogenisation(p_g);
        Vector4f p_k = depthExtrinsicsInv*p;
        if (p_k.z()<0){
            return Vector2i(MINF,MINF);
        }
        Vector3f p_p;
        p_p << p_k.x(), p_k.y(), p_k.z();
        p_p = depthIntrinsics*p_p;
        Vector2i pixel_coord;
        pixel_coord << floor(p_p.x()/p_p.z()), floor(p_p.y()/p_p.z());
        return pixel_coord;
    }
    //equation (7)
    float Lamda(const Matrix3f& depthIntrinsics, const Vector2i& pixel_coord)
    {
        Matrix3f K_Inv = depthIntrinsics.inverse();
        Vector3f homo_pixel_coord = homogenisation(pixel_coord.cast<float>());
        Vector3f temp = K_Inv*homo_pixel_coord;
        float lamda = temp.norm();
        return lamda;

    }
    //equation （6) for each point
    float SDF_k_i(const float mu, float lamda, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p, const float depth_k_i, const float& default_sdf)
    {
        Vector3f translation;
        translation << depthExtrinsics.topRightCorner(3,1);
        float sdf_k_i;
        float Eta =depth_k_i*1000 - (translation*1000 - p).norm()/lamda ;//change of unit from m to mm
        sdf_k_i = SDF_truncation(Eta, mu, default_sdf);
        return sdf_k_i;
    }
    //
    //void update_volument(VirtualSensor& sensor, Voxel& volument, const float defaut_dst )
    //{
    void update_volument(VirtualSensor& sensor, Voxel& volument, const Matrix4f depthExtrinsics )
    {
        // Vector3i orig = volument.getOrig();
        // std::cout<<"test====="<<std::endl;
        // const unsigned int k = 3;//TODO:??
        // unsigned int i = 0;
            const float defaut_dst = volument.getDefaultDist();//TODO: get wrong intial value
        // while (sensor.processNextFrame() && i <= k) {
            float y_z_limit = sensor.get_tan_y_z();
            float x_z_limit = sensor.get_tan_x_z();
            Map2Df depthMap_k_i{sensor.getDepth()};
            //float* depthMap_k_i = sensor.getDepth().data();;
            const Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
            //const Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();
            // const Matrix4f depthExtrinsics = depthExtrinsics;
            const unsigned int width = sensor.getDepthImageWidth();
            const unsigned int height = sensor.getDepthImageHeight();
            const float mu = volument.truncateDistance;//TODO: right? wirte a config file
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            for (int voxel_xi=0; voxel_xi<volument.numX; voxel_xi++)
            // for (int voxel_xi=75; voxel_xi<125; voxel_xi++)
            {
                end = std::chrono::steady_clock::now();
                std::cout<<"x:"<<voxel_xi<<std::endl;
                std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
                begin = end;
                for(int voxel_yi=0; voxel_yi<volument.numY; voxel_yi++)
                // for(int voxel_yi=75; voxel_yi<125; voxel_yi++)
                {
                   
                    for(int voxel_zi=0; voxel_zi<volument.numZ; voxel_zi++)
                    {
                        Vector3f p_g = volument.ContFromOrd(Vector3i(voxel_xi, voxel_yi, voxel_zi));
                        
                        
                        
                        
                        
                        
                        Vector2i x = globalToScreen(depthIntrinsics, depthExtrinsics, p_g, x_z_limit, y_z_limit);
                        if(x.x()>width || x.y()>height || x.x()<0 || x.y()<0)
                        {
                            continue;
                        }
                        
                        float depth_k_i = depthMap_k_i.get( x.y(), x.x());
                        if (!(depth_k_i>0)){
                            continue;
                        }
                        const float lamda = Lamda(depthIntrinsics, x);
                        float SDF_k_i_n = SDF_k_i(mu, lamda, depthIntrinsics, depthExtrinsics, p_g, depth_k_i, defaut_dst);
                        float F_k_i_j = volument.getDistance(p_g.x(), p_g.y(), p_g.z());
                        float new_dist;
                        if (F_k_i_j !=defaut_dst && SDF_k_i_n!=defaut_dst)
                        {
                            new_dist = (F_k_i_j + SDF_k_i_n)/2;
                            volument.setDistance(p_g.x(), p_g.y(), p_g.z(), new_dist);
                        }    
                        else if (SDF_k_i_n!=defaut_dst)
                        {
                            new_dist = SDF_k_i_n;
                            if (new_dist<0 && new_dist>-1){
                                //std::cout<<"GOTBACK, "<<new_dist<<std::endl;
                            }
                            else{
                                //std::cout<<"GOTFRONT, "<<new_dist<<std::endl;
                            }
                            volument.setDistance(p_g.x(), p_g.y(), p_g.z(), new_dist);
                            assert (volument.getDistance(p_g.x(), p_g.y(), p_g.z()) == new_dist);
                        }
                            
                        

                    }
                }
               
            }
        // i++;
        // }
        
    }

} // namespace kinect_fusion
