#include <iostream>
// #include<algorithm>
#include "Eigen.h"
// #include"data_frame.h"
#include"point_cloud.h"
#include"Voxel.h"
#include"virtual_sensor.h"
#include"type_definitions.h"
namespace kinect_fusion {
    
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
        homo_coord.segment(0, dehomo_coord.size()) << dehomo_coord;//TODO
        homo_coord[dehomo_coord.size()] = 1;
        return homo_coord;

    }
    
    float SDF_truncation(float& x, const float& mu, const float& default_sdf)//quation (9)
    {   
        int sign = x/abs(x);
        if(x > -mu)
        {
            return fmin(1, x/mu)*sign;
        }
        else
            return default_sdf;
    }
    Vector2i globalToScreen(const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p_g )//equation (8)
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
    float Lamda(const Matrix3f& depthIntrinsics, const Vector2i& pixel_coord)//equation (7)
    {
        Matrix3f K_Inv = depthIntrinsics.inverse();
        Vector3f homo_pixel_coord = homogenisation(pixel_coord.cast<float>());
        Vector3f temp = K_Inv*homo_pixel_coord;
        float lamda = temp.norm();
        return lamda;

    }
     //p: global point x,y,z
    float SDF_k_i(const float mu, float lamda, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p, const float depth_k_i, const float& default_sdf)
    {
        Vector3f translation;
        translation << depthExtrinsics.topRightCorner(3,1);
        float sdf_k_i;
        float Eta = (translation - p).norm()/lamda - depth_k_i;
        sdf_k_i = SDF_truncation(Eta, mu, default_sdf);
        return sdf_k_i;//equation （6) for each point
    }

    //equation (6)
    // std::vector<float> SDF_k(const float defaut_dst, const unsigned width, const unsigned height, const float mu, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const float* depth_k, const std::vector<Vector3f>& vertex_k)
    // {
    //     unsigned int length = vertex_k.size();
    //     // float* sdf_k;
    //     std::vector<float> sdf_k(length);
        
    //     for (unsigned int i=0; i<length; i++){
    //         // if(i==1208)
    //         //         std::cout << "BUG" << std::endl;
    //         Vector2i x = globalToScreen(depthIntrinsics, depthExtrinsics, vertex_k[i]);
    //         const float lamda = Lamda(depthIntrinsics, x);
    //         unsigned int indx = x.x()*width+ x.y();
    //         if (indx<length){
    //             float depth_k_i = depth_k[indx];
    //             if(depth_k_i == MINF)
    //                 sdf_k[i] = defaut_dst; //defaut volum
    //             else
    //                 sdf_k[i] = SDF_k_i(mu, lamda, depthIntrinsics, depthExtrinsics, vertex_k[i], depth_k_i);
    //         }
    //         else
    //             sdf_k[i] = defaut_dst;//defaut volum
    //     }
    //     return sdf_k;
    // }
    
    // Voxel volument(Distance=NULL, Weight=0)I need a function to initial Distance and Weight 
    void update_volument(VirtualSensor& sensor, Voxel& volument)
    {
        // Vector3i orig = volument.getOrig();
        const unsigned int k = 3;//TODO:??
        unsigned int i = 0;
        const float defaut_dst = volument.getDefaultDist();
        while (sensor.processNextFrame() && i <= k) {
            Map2Df depthMap_k_i{sensor.getDepth()};
            //float* depthMap_k_i = sensor.getDepth().data();;
            const Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
            const Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();
            const unsigned int width = sensor.getDepthImageWidth();
            const unsigned int height = sensor.getDepthImageHeight();
            
            PointCloud PointCloud_k_i{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(),  sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};
            std::vector<Vector3f> vertex_k_i = PointCloud_k_i.getPoints();
            const float mu = 0.1;//
            // intersect_table table_i;
            //std::vector<float> F_Rk_i = SDF_k(defaut_dst, width, height, mu, depthIntrinsics, depthExtrinsics, depthMap_k_i, vertex_k_i);
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
                        if (F_k_i_j != defaut_dst and SDF_k_i_n!=defaut_dst)//TODO:get default value
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
            // for(unsigned int j=0; j<vertex_k_i.size(); j++)
            // {               
            //     //volument.setDistance( vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), F_Rk_i[j]);
            //     //volument.setWeight(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), 1);
            //     //TODO:
            //     //std::vector<Matrix3f> intersect_points;//all intersect_points in global coordinate along this ray
            //     // getIntersect(volument, vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), camera_coord.x(), camera_coord.y(), camera_coord.z(), intersect_points);
            //     Vector2i x = globalToScreen(depthIntrinsics, depthExtrinsics, vertex_k_i[j]);
            //     const float lamda = Lamda(depthIntrinsics, x);
            //     for(unsigned int n=0; n<intersect_points.size(); n++)
            //     {
            //         float SDF_k_i_n = SDF_k_i(mu, lamda, depthIntrinsics, depthExtrinsics, intersect_points[n], depthMap_k_i[j], defaut_dst);
            //         float F_k_i_j = volument.getDistance(intersect_points[n].x(), intersect_points[n].y(), intersect_points[n].z());
            //         float new_dist;
            //         if (F_k_i_j != defaut_dst and SDF_k_i_n!=defaut_dst)//TODO:get default value
            //         {
            //             new_dist = (F_k_i_j + SDF_k_i_n)/2;
            //             volument.setDistance(intersect_points[n].x(), intersect_points[n].y(), intersect_points[n].z(), new_dist);
            //         }    
            //         else if (SDF_k_i_n!=defaut_dst)
            //         {
            //             new_dist = SDF_k_i_n;
            //             volument.setDistance(intersect_points[n].x(), intersect_points[n].y(), intersect_points[n].z(), new_dist);
            //         }
                        
            //         else
            //             ;
            //     }
            //     intersect_points.clear();
            //     //table_i.push_back(intersect_points);
            //     // float F_k_i_j = volument.getDistance(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z());
            //     // float new_dist;
            //     // if (F_k_i_j != defaut_dst)//TODO:get default value
            //     //     new_dist = (F_k_i_j + F_Rk_i[j])/2;
            //     // else
            //     //     new_dist = F_k_i_j;
            //     // volument.setDistance(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), new_dist);
            // }
            
            // // F_Rk_i.clear();
            // delete [] depthMap_k_i;

        i++;
        }
        
    }

} // namespace kinect_fusion
