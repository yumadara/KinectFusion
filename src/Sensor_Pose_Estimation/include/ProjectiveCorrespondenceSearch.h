#include <math.h>

#include "Eigen.h"
#include <data_frame.h>
#include <surface.h>


namespace kinect_fusion {

class projectiveCorrespondence {
public:
	/// <summary>
	/// 
	/// </summary>
	/// <param name="targetMap"> targetMap in camera space of frame k 
	/// <param name="previousTransformation"></param> previous transformation also in camera space 
	/// <param name="currentTransformation"></param> current transformation also in camera space
	/// <param name="camera_intrinsics"></param> 
	projectiveCorrespondence(const Map2DVector3f& targetMap,
		const Eigen::MatrixXf& previousTransformation,
		const Eigen::MatrixXf& currentTransformation,
		const Eigen::Matrix3f& camera_intrinsics)
		
	{
		m_targetMap = targetMap;
		//m_modelPoints = surfacePoints;
		m_previousTransformation = previousTransformation;
		m_currentTransformation = currentTransformation;
		m_cameraIntrinsics = camera_intrinsics;
	}
	int matchPoint()
	{
		// match target point with correspondence point
		for (int i = 0; i != m_targetMap.size(); i++)
		{
			// find correspondence point in m_modelpoints
			if (m_targetMap.get(i)[0] != MINF && m_targetMap.get(i)[1] != MINF && m_targetMap.get(i)[2] != MINF)
			{
				int correspondenceIndexInSurface = findCorrespondence(m_targetMap.get(i));
				if (correspondenceIndexInSurface != -1) // found correspondence
				{
					match.insert(std::pair<int, int>(i, correspondenceIndexInSurface));
				}
			}
			
		}
		return 1;
	}

	int findCorrespondence(const Eigen::Vector3f& targetPoint)
	{
		Eigen::Vector3f point;
		point(0) = targetPoint[0];
		point(1) = targetPoint[1];
		point(2) = targetPoint[2];
		//point(3) = 1;
		Eigen::MatrixXf Transformation = m_previousTransformation.inverse()* m_currentTransformation;
		Eigen::Matrix3f intrinsics = m_cameraIntrinsics; // should be k-1 th frame intrinsic
		unsigned int depthHeight = m_targetMap.getHeight();
		unsigned int depthWidth = m_targetMap.getWidth();
		
		int imagePlaneCoordinateX = round(((intrinsics * (Transformation.block(0,0,3,3) * point + Transformation.block(0,3,3,1)))(0)));
		int imagePlaneCoordinateY = round(((intrinsics * (Transformation.block(0, 0, 3, 3) * point + Transformation.block(0, 3, 3, 1))))(1));
		if (imagePlaneCoordinateX > depthWidth || imagePlaneCoordinateY > depthHeight)
		{
			return -1;
		}
		return imagePlaneCoordinateY * depthWidth + imagePlaneCoordinateX;
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
	Map2DVector3f m_targetMap; // surface of k-th frame 
	Eigen::MatrixXf m_cameraIntrinsics;
	int m_height;
	int m_width;
};

} // namespace kinect_fusion