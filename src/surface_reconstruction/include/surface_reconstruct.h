#include <iostream>
#include "Eigen.h"
#include"Voxel.h"
#include"virtual_sensor.h"
#include"type_definitions.h"
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
        int sign = x/abs(x);
        if(x > -mu)
        {
            return fmin(1, x/mu)*sign;
        }
        else
            return default_sdf;
    }
    //equation (8)
    Vector2i globalToScreen(const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p_g )
    {
        Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
        Vector4f p = homogenisation(p_g);
        Vector4f p_k = depthExtrinsicsInv*p;
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
        float Eta = (translation - p).norm()/lamda - depth_k_i;
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
            Map2Df depthMap_k_i{sensor.getDepth()};
            //float* depthMap_k_i = sensor.getDepth().data();;
            // const Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
            //const Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();
            // const Matrix4f depthExtrinsics = depthExtrinsics;
            const unsigned int width = sensor.getDepthImageWidth();
            const unsigned int height = sensor.getDepthImageHeight();
            const float mu = 10.0;//TODO: right? wirte a config file
            for (int voxel_xi=0; voxel_xi<volument.numX; voxel_xi++)
            {
                for(int voxel_yi=0; voxel_yi<volument.numY; voxel_yi++)
                {
                    for(int voxel_zi=0; voxel_zi<volument.numZ; voxel_zi++)
                    {
                        float p_x =  volument.ContFromOrd(voxel_xi);
                        float p_y =  volument.ContFromOrd(voxel_yi);
                        float p_z =  volument.ContFromOrd(voxel_zi);
                        Vector3f p_g(p_x, p_y, p_z);
                        Vector2i x = globalToScreen(depthIntrinsics, depthExtrinsics, p_g);
                        if(x.x()>height || x.y()>width || x.x()<0 || x.y()<0)
                        {
                            continue;
                        }
                        float depth_k_i = depthMap_k_i.get(x.x(), x.y());
                        const float lamda = Lamda(depthIntrinsics, x);
                        float SDF_k_i_n = SDF_k_i(mu, lamda, depthIntrinsics, depthExtrinsics, p_g, depth_k_i, defaut_dst);
                        float F_k_i_j = volument.getDistance(p_x, p_y, p_z);
                        float new_dist;
                        if (F_k_i_j != defaut_dst and SDF_k_i_n!=defaut_dst)
                        {
                            new_dist = (F_k_i_j + SDF_k_i_n)/2;
                            volument.setDistance(p_x, p_y, p_z, new_dist);
                        }    
                        else if (SDF_k_i_n!=defaut_dst)
                        {
                            new_dist = SDF_k_i_n;
                            volument.setDistance(p_x, p_y, p_z, new_dist);
                        }
                            
                        else
                            ;

                    }
                }
               
            }
        // i++;
        // }
        
    }

} // namespace kinect_fusion
