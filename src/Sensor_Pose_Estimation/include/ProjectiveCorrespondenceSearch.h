#include <math.h>

#include "Eigen.h"
#include <data_frame.h>
#include <Surface.h>
#include <iostream>

namespace kinect_fusion {

class projectiveCorrespondence {
public:
	/// <summary>
	/// 
	/// </summary>
	/// <param name="targetMap"> targetMap in camera space of frame k 
	/// <param name="previousTransformation"></param> previous transformation also in camera space 
	/// <param name="currentTransformation"></param> current transformation also in camera space
	/// <param name="camera_intrinsics"></param> corressponding camera intrinsics
	projectiveCorrespondence(const Map2DVector3f& targetVertexMap,
		const Map2DVector3f& targetNormalMap,
		const Eigen::MatrixXf& previousTransformation,
		const Eigen::MatrixXf& currentTransformation,
		const Eigen::Matrix3f& camera_intrinsics)
	{
		m_targetVertexMap = targetVertexMap;
		m_targetNormalMap = targetNormalMap;
		//m_modelPoints = surfacePoints;
		m_previousTransformation = previousTransformation;
		m_currentTransformation = currentTransformation;
		m_cameraIntrinsics = camera_intrinsics;
	}
	int matchPoint()
	{
		// match target point with correspondence point
		for (int i = 0; i != m_targetVertexMap.size(); i++)
		{
			// find correspondence point in m_modelpoints
			Vector3f vertex = m_targetVertexMap.get(i);
			Vector3f normal = m_targetNormalMap.get(i);
			if (!isnan(vertex[0]) &&  !isnan(vertex[1]) && !isnan(vertex[2])
				&& !isnan(normal[0]) && !isnan(normal[1]) && !isnan(normal[2])&&
				vertex[0] != MINF && vertex[1] != MINF && vertex[2] != MINF &&
				normal[0] != MINF && normal[1]!=MINF && normal[2] != MINF)
			{
				//std::cout << "input target point  " << m_targetVertexMap.get(i) << std::endl;
				//std::cout << "target normal  " << m_targetNormalMap.get(i) << std::endl;
				//std::cout << "target index " << i << std::endl;
				int correspondenceIndexInSurface = findCorrespondence(m_targetVertexMap.get(i));
				if (correspondenceIndexInSurface != -1) // found correspondence
				{

					//assert(i == correspondenceIndexInSurface);
					match.insert(std::pair<int, int>(i, correspondenceIndexInSurface));
					
				}
			}
			
		}
		return 1;
	}
	/// <summary>
	/// 
	/// </summary>
	/// <param name="targetPoint" Target point is kth frame vertex in camera space
	/// <returns></returns>
	int findCorrespondence(const Eigen::Vector3f& targetPoint)
	{

		//point(3) = 1;
		Eigen::MatrixXf Transformation = m_previousTransformation.inverse()* m_currentTransformation;
		Eigen::Matrix3f intrinsics = m_cameraIntrinsics; // should be k-1 th frame intrinsic
		unsigned int depthHeight = m_targetVertexMap.getHeight();
		unsigned int depthWidth = m_targetVertexMap.getWidth();
		//std::cout << "intrinsics " << intrinsics << std::endl;
		//std::cout << "target point" << targetPoint << std::endl;
		//std::cout << "previous transformation " << m_previousTransformation << std::endl;
		//std::cout << "current transformation " << m_currentTransformation << std::endl;
		//std::cout << "result is " << intrinsics * (Transformation.block(0, 0, 3, 3) * targetPoint + Transformation.block(0, 3, 3, 1)) << std::endl;
		Vector3f point = (Transformation.block(0, 0, 3, 3) * targetPoint + Transformation.block(0, 3, 3, 1));// k -1 frame 
			//Vector3f image_plane_point = intrinsics*point;
			//std::cout << "image plane point " << image_plane_point << std::endl;
		//std::cout << "source point" << point << std::endl;
		float imagePlaneCoordinateX = (intrinsics(0, 0) * point.x() / point.z() + intrinsics(0, 2));
		float imagePlaneCoordinateY = (intrinsics(1, 1) * point.y() / point.z() + intrinsics(1, 2));
		//std::cout << "image X " << imagePlaneCoordinateX << std::endl;
		//std::cout << "image Y " << imagePlaneCoordinateY << std::endl;

		if (imagePlaneCoordinateX > depthWidth || imagePlaneCoordinateY > depthHeight|| imagePlaneCoordinateX < 0 || imagePlaneCoordinateY<0)
		{
			return -1;
		}
		//std::cout << "source index " << imagePlaneCoordinateY * depthWidth + imagePlaneCoordinateX << std::endl;
		return round(imagePlaneCoordinateY * depthWidth + imagePlaneCoordinateX);
	}

	std::map<int, int> getMatch()
	{
		return match;
	}
private:
	
	//std::vector<Eigen::Vector3f> m_surfacePoints;
	std::map<int, int> match;
	Eigen::MatrixXf m_previousTransformation; //T_{g,k-1}
	Eigen::MatrixXf m_currentTransformation; //T_{g,k}, initial setting will be T_{g, k-1}
	Map2DVector3f m_targetVertexMap; // surface of k-th frame 
	Map2DVector3f m_targetNormalMap;
	Eigen::MatrixXf m_cameraIntrinsics;
	int m_height;
	int m_width;
};

} // namespace kinect_fusion