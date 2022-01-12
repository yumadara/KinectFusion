#include "ProjectiveCorrespondenceSearch.h"
#include "Optimizer.h"

#include <Surface.h>

extern std::map<int, int> iteration_num_with_level = map_list_of(2, 4) (1, 5) (0, 10);

namespace kinect_fusion {

	class PoseEstimator {
	public:
		PoseEstimator(FrameData& currentFrameData, FrameData& lastFrameData, Eigen::MatrixXf& lastTransformation  )
		{
            m_currentFrameData = currentFrameData;
            m_lastFrameData = lastFrameData;
			m_currentFrameSurface = currentFrameData.getSurface();
			m_lastFrameSurface = lastFrameData.getSurface();
			m_lastTransformation = lastTransformation;
		}
        Eigen::Vector3f TransformVertex(Eigen::Vector3f& vertex, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0, 3, 3, 1);
        }
        Eigen::Vector3f TransformNormal(Eigen::Vector3f& normal, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0, 0, 3, 3) * normal;
        }
        /// <summary>
        /// 
        /// 
        /// <param name="sourceNormals" is transformed to global frame, which is N^{g}_{k-1} 
        /// <param name="targetNormals" is transformed to global frame, which is R^{z}_{g,k}*Nk(u)
        /// <param name="sourceVertices" is transformed to global frame, which is V^{g}_{k-1}
        /// <param name="targetVertices" is transformed to global frame, which is T^{z}_{g,k}*Vk(u)
        /// <param name="matches"></param>
        map<int, int> pruneCorrespondences(const Map2DVector3f& sourceNormals,
            const Map2DVector3f& targetNormals,
            const Map2DVector3f& sourceVertices,
            const Map2DVector3f& targetVertices,
            std::map<int, int>& match) {
            
            map<int, int>::iterator it;
            for (it = match.begin(); it != match.end();it++) {
                int index_target = it->first;
                int index_source = it->second;
                
                const auto& sourceNormal = sourceNormals.get[index_source];
                const auto& targetNormal = targetNormals.get[index_target];

                const auto& sourceVertex = sourceVertices.get[index_source];
                const auto& targetVertex = targetVertices.get[index_target];

                // TODO: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
                float cosin = (sourceNormal.x() * targetNormal.x() + sourceNormal.y() * targetNormal.y() + sourceNormal.z() * targetNormal.z()) / (sourceNormal.norm() * targetNormal.norm());
                float dis = (sourceVertex - targetVertex).norm();
                if ( cosin > epsilon_theta || dis > epsilon_d ) {
                    match.erase(index_target);
                }  }
            return match;
            }
        
        

        Matrix4f calculateIncremental(const Map2DVector3f& targetVertex,
            const Map2DVector3f& sourceVertex,
            const Map2DVector3f& sourceNormals,
            std::map<int, int>& match) 
        {
            MatrixXf A = MatrixXf::Zero(6, 6);
            float b = 0;
            map<int, int>::iterator it;
            for (it = match.begin(); it != match.end(); it++) {
                int index_target = it->first;
                int index_source = it->second;
                Eigen::Vector3f targetPoint = targetVertex.get(index_target);
                Eigen::Vector3f sourcePoint = sourceVertex.get(index_source);
                Eigen::Vector3f sourceNormal = sourceNormals.get(index_source);
                MatrixXf  G(3, 6);
                G(0, 0) = 0;
                G(1, 1) = 0;
                G(2, 2) = 0;
                G(0, 1) = -targetPoint[2];
                G(0, 2) = targetPoint[1];
                G(1, 2) = -targetPoint[0];
                G(1, 0) = targetPoint[2];
                G(2, 0) = -targetPoint[1];
                G(2, 1) = targetPoint[0];
                G(0, 3) = 1;
                G(1, 4) = 1;
                G(2, 5) = 1;

                Eigen::MatrixXf tempA(6, 6);
                Eigen::VectorXf tempbias(6);
                tempA = G.transpose() * targetNormal * targetNormal.transpose() * G;
                tempbias = G.transpose() * targetNormal * targetNormal.transpose() * (sourcePoint - targetPoint);
                A = A + tempA;
                b = b + tempbias;
            }

            VectorXf x = A.ldlt().solve(b);
            result(0, 0) = 1;
            result(1, 1) = 1;
            result(2, 2) = 1;
            result(0, 3) = x[3];
            result(1, 3) = x[4];
            result(2, 3) = x[5];
            result(1, 2) = x[0];
            result(2, 1) = -x[0];
            result(0, 2) = -x[1];
            result(2, 0) = x[1];
            result(0, 1) = x[2];
            result(1, 0) = -x[2];

            return result;
        }
		
        void frame2frameEstimation(Eigen::MatrixXf inputTransformationMatrix, Level level)
        {
            Map2DVector3f currentFrameNormal = m_currentFrameSurface.getNormalMap();
            Map2DVector3f currentFrameVertex = m_currentFrameSurface.getVertexMap();

            Map2DVector3f lastFrameNormal = m_lastFrameSurface.getNormalMap();
            Map2DVector3f lastFrameVertex = m_lastFrameSurface.getVertexMap();

            for (int i = 0; i != iteration_num_with_level[getIndex(level)]; i++)
            {
                projectiveCorrespondence pc(currentFrameVertex, m_lastTransformation, m_currentTransformation, m_lastFrameData.getCameraIntrinsics(level));
                pc.matchPoint(currentFrameVertex);
                std::map<int, int> match = pc.getMatch();

                PoseEstimator pose_estimator(FrameData & currentFrameData, FrameData & lastFrameData, Eigen::MatrixXf & lastTransformation);
                Map2DVector3f lastFrameNormal_transformed = TransformNormal(lastFrameNormal, m_lastTransformation);
                Map2DVector3f lastFrameVertex_transformed = TransformVertex(lastFrameVertex, m_lastTransformation);

                Map2DVector3f currentFrameNormal_transformed = TransformNormal(currentFrameNormal, m_currentTransformation);
                Map2DVector3f currentVertex_transformed = TransformVertex(currentFrameNormal, m_currentTransformation);

                match = pose_estimator.pruneCorrespondences(lastFrameNormal_transformed, currentFrameNormal_transformed, lastFrameVertex_transformed, currentVertex_transformed, match);
                Matrix4f incremental = calculateIncremental(currentVertex_transformed, lastFrameVertex_transformed, lastFrameNormal_transformed, match);           
                m_currentTransformation = incremental * m_currentTransformation;
            }
        }

        Eigen::MatrixXf getCurrentTransformation()
        {
            return m_currentTransformation;
        }

        
	private:
        FrameData m_currentFrameData;
        FrameData m_lastFrameData;
		Surface m_currentFrameSurface;
		Surface m_lastFrameSurface;
		Eigen::MatrixXf m_lastTransformation; // k-1 frame pose estimation
        static Eigen::MatrixXf m_currentTransformation;
	}
}