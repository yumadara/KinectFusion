#include <iostream>
// #include<algorithm>
#include "Eigen.h"
// #include"data_frame.h"
#include"point_cloud.h"
#include"Voxel.h"
#include"virtual_sensor.h"
    // some comments
    // ?????
   //????
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
    
    float SDF_truncation(const float defaut_dst, float& x, const float& mu)//quation (9)
    {   
        int sign = x/abs(x);
        if(x > -mu)
        {
            return fmin(1, x/mu)*sign;//TODO:!
        }
        else
            return defaut_dst;//TODO
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
    float SDF_k_i(const float defaut_dst, const float mu, float lamda, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const Vector3f& p, const float depth_k_i)
    {
        Vector3f translation;
        translation << depthExtrinsics.topRightCorner(3,1);
        float sdf_k_i;
        float Eta = (translation - p).norm()/lamda - depth_k_i;
        sdf_k_i = SDF_truncation(defaut_dst, Eta, mu);
        return sdf_k_i;//equation （6) for each point
    }

    //equation (6)
    std::vector<float> SDF_k(const float defaut_dst, const unsigned width, const unsigned height, const float mu, const Matrix3f& depthIntrinsics, const Matrix4f& depthExtrinsics, const float* depth_k, const std::vector<Vector3f>& vertex_k)
    {
        unsigned int length = vertex_k.size();
        // float* sdf_k;
        std::vector<float> sdf_k(length);
        
        for (unsigned int i=0; i<length; i++){
            // if(i==1208)
            //         std::cout << "BUG" << std::endl;
            Vector2i x = globalToScreen(depthIntrinsics, depthExtrinsics, vertex_k[i]);
            const float lamda = Lamda(depthIntrinsics, x);
            unsigned int indx = x.x()*width+ x.y();
            if (indx<length){
                float depth_k_i = depth_k[indx];
                if(depth_k_i == MINF)
                    sdf_k[i] = defaut_dst; //defaut volum
                else
                    sdf_k[i] = SDF_k_i(defaut_dst, mu, lamda, depthIntrinsics, depthExtrinsics, vertex_k[i], depth_k_i);
            }
            else
                sdf_k[i] = defaut_dst;//defaut volum
        }
        return sdf_k;
    }
    namespace kinect_fusion{
    // Voxel volument(Distance=NULL, Weight=0)I need a function to initial Distance and Weight 
    void update_volument(kinect_fusion::VirtualSensor& sensor, kinect_fusion::Voxel& volument)
    {
        // Vector3i orig = volument.getOrig();
        const unsigned int k = 3;//TODO:??
        unsigned int i = 0;
        const float defaut_dst = volument.getDefaultDist();
        while (sensor.processNextFrame() && i <= k) {
            float* depthMap_k_i = sensor.getDepth().data();
            const Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
            const Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();
            const unsigned width = sensor.getDepthImageWidth();
            const unsigned height = sensor.getDepthImageHeight();
            
            PointCloud PointCloud_k_i{depthMap_k_i, depthIntrinsics, depthExtrinsics, width, height};
            std::vector<Vector3f> vertex_k_i = PointCloud_k_i.getPoints();
            const float mu = 0.1;//TODO
            
            std::vector<float> F_Rk_i = SDF_k(defaut_dst, width, height, mu, depthIntrinsics, depthExtrinsics, depthMap_k_i, vertex_k_i);
            for(unsigned int j=0; j<vertex_k_i.size(); j++)
            {               
                //volument.setDistance( vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), F_Rk_i[j]);
                volument.setWeight(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), 1);
                float F_k_i_j = volument.getDistance(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z());
                float new_dist;
                if (F_k_i_j != defaut_dst)//TODO
                    new_dist = (F_k_i_j + F_Rk_i[j])/2;
                else
                    new_dist = F_k_i_j;
                volument.setDistance(vertex_k_i[j].x(), vertex_k_i[j].y(), vertex_k_i[j].z(), new_dist);
            }
            F_Rk_i.clear();
            delete [] depthMap_k_i;

        i++;
        }
        
    }

}