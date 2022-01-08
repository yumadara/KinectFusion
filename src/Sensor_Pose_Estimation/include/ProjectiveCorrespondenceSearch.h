#include <math.h>

#include "Eigen.h"

#include <surface.h>
#include <virtual_sensor.h>

namespace kinect_fusion {

class projectiveCorrespondence {
public:
	projectiveCorrespondence(const std::vector<Eigen::Vector3f>& targetPoints, 
		const Eigen::MatrixXf& previousTransformation,
		const Eigen::MatrixXf& currentTransformation,
		const VirtualSensor virtualSensor)
	{
		m_targetPoints = targetPoints;
		//m_modelPoints = surfacePoints;
		m_previousTransformation = previousTransformation;
		m_currentTransformation = currentTransformation;
		m_virtualSensor = virtualSensor;
	}
	int matchPoint()
	{
		// match target point with correspondence point
		for (int i = 0; i != m_targetPoints.size(); i++)
		{
			// find correspondence point in m_modelpoints
			if (m_targetPoints[i][0] != MINF && m_targetPoints[i][1] != MINF && m_targetPoints[i][2] != MINF)
			{
				int correspondenceIndexInSurface = findCorrespondence(m_targetPoints[i]);
				if (correspondenceIndexInSurface != -1) // found correspondence
				{
					match.push_back(correspondenceIndexInSurface);
				}
				else // 
				{
					match.push_back(-1);
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
		Eigen::Matrix3f intrinsics = m_virtualSensor.getColorIntrinsics(); // should be k-1 th frame intrinsic
		unsigned int depthHeight = m_virtualSensor.getDepthImageHeight();
		unsigned int depthWidth = m_virtualSensor.getDepthImageWidth();
		
		int imagePlaneCoordinateX = round(((intrinsics * (Transformation.block(0,0,3,3) * point + Transformation.block(0,3,3,1)))(0)));
		int imagePlaneCoordinateY = round(((intrinsics * (Transformation.block(0, 0, 3, 3) * point + Transformation.block(0, 3, 3, 1))))(1));
		if (imagePlaneCoordinateX > depthWidth || imagePlaneCoordinateY > depthHeight)
		{
			return -1;
		}
		return imagePlaneCoordinateY * depthWidth + imagePlaneCoordinateX;
	}
	std::vector<int> getMatch()
	{
		return match;
	}

	std::vector<Eigen::Vector3f> getTargetPoint()
	{
		return m_targetPoints;
	}
private:
	std::vector<Eigen::Vector3f> m_targetPoints;
	//std::vector<Eigen::Vector3f> m_surfacePoints;
	std::vector<int> match;
	Eigen::MatrixXf m_previousTransformation; //T_{g,k-1}
	Eigen::MatrixXf m_currentTransformation; //T_{g,k}
	Surface m_surface; // surface of last frame 
	VirtualSensor m_virtualSensor;
};

} // namespace kinect_fusion