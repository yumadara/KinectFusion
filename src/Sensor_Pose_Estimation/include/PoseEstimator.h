#include "ProjectiveCorrespondenceSearch.h"
extern float epsilon_theta = 0.5;
extern float epsilon_d = 0.01;
#include <virtual_sensor.h>
#include <data_frame.h>
//extern std::map<int, int> iteration_num_with_level = map_list_of(2, 4) (1, 5) (0, 10);

namespace kinect_fusion {

	class PoseEstimator {
	public:
		PoseEstimator(FrameData& currentFrameData, FrameData& lastFrameData, Eigen::MatrixXf& lastTransformation , Eigen::MatrixXf& currentTransformation)
		{
            
            m_currentFrameData = currentFrameData;
            m_lastFrameData = lastFrameData;
			m_currentFrameSurface = currentFrameData.getSurface();
			m_lastFrameSurface = lastFrameData.getSurface();
			m_lastTransformation = lastTransformation;
            m_currentTransformation = currentTransformation;
		}
        Vector3f TransformVertex(Vector3f vertex, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0,3,3,1);

        }
        Vector3f TransformNormal(Vector3f normal, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0,0,3,3)* normal + Transformation.block(0,3,3,1);
        }
        /// <summary>
        /// 
        /// 
        /// <param name="sourceNormals" is transformed to global frame, which is N^{g}_{k-1} 
        /// <param name="targetNormals" is transformed to global frame, which is R^{z}_{g,k}*Nk(u)
        /// <param name="sourceVertices" is transformed to global frame, which is V^{g}_{k-1}
        /// <param name="targetVertices" is transformed to global frame, which is T^{z}_{g,k}*Vk(u)
        /// <param name="matches"></param>
        Matrix4f pruneCorrespondences(const Map2DVector3f& sourceNormals,
            const Map2DVector3f& targetNormals,
            const Map2DVector3f& sourceVertices,
            const Map2DVector3f& targetVertices,
            std::map<int, int>& match) {
            int num = 0;
            int valid_num = 0;
            std::map<int, int>::iterator it;

            MatrixXf A = MatrixXf::Zero(6, 6);
            Eigen::VectorXf b = VectorXf::Zero(6);

            for (it = match.begin(); it != match.end();it++) {
                int index_target = it->first;
                int index_source = it->second;
            
                Vector3f sourceNormal = sourceNormals.get(index_source);
                Vector3f targetNormal = targetNormals.get(index_target);

                Vector3f sourceVertex = sourceVertices.get(index_source);
                Vector3f targetVertex = targetVertices.get(index_target);
                
                if (!isnan(sourceVertex[0]) && !isnan( sourceVertex[1] ) && !isnan( sourceVertex[2] )&& !isnan(-sourceNormal[0]) && !isnan(-sourceNormal[1]) && !isnan(-sourceNormal[2])
                    && sourceVertex[0] !=MINF && sourceVertex[1] != MINF&& sourceVertex[2] != MINF && sourceNormal[0] != MINF&& sourceNormal[1] != MINF&& sourceNormal[2] != MINF )
                {
                    //std::cout << "target point  " << targetVertex << std::endl;
                    //std::cout << "source point " << sourceVertex << std::endl;

                    //std::cout << "target normal  " << targetNormal << std::endl;
                    //std::cout << "source normal " << sourceNormal << std::endl;

                    float cosin = (sourceNormal.x() * targetNormal.x() + sourceNormal.y() * targetNormal.y() + sourceNormal.z() * targetNormal.z()) / (sourceNormal.norm() * targetNormal.norm());
                    float dis = (sourceVertex - targetVertex).norm();
                    if (cosin > epsilon_theta || dis > epsilon_d ) {
                        //std::cout << "too far" << std::endl;
                        num++;
                        //std::cout << "match size" <<match.size()<< std::endl;
                        //match.erase(it);
                        //std::cout << "after erasing, match size" << match.size() << std::endl;
                    }
                    else
                    {
                        valid_num++;
                        Vector3f transformedSourceVertex = TransformVertex(sourceVertex, m_currentTransformation);
                        Vector3f transformedTargetVertex = TransformVertex(targetVertex, m_currentTransformation);

                        Vector3f transformedSourceNormal = TransformNormal(sourceNormal, m_currentTransformation);
                        Vector3f transformedTargetNormal = TransformNormal(targetNormal, m_currentTransformation);

                        MatrixXf  G(3, 6);
                        G(0, 0) = 0;
                        G(1, 1) = 0;
                        G(2, 2) = 0;
                        G(0, 1) = -transformedTargetVertex[2];
                        G(0, 2) = transformedTargetVertex[1];
                        G(1, 2) = -transformedTargetVertex[0];
                        G(1, 0) = transformedTargetVertex[2];
                        G(2, 0) = -transformedTargetVertex[1];
                        G(2, 1) = transformedTargetVertex[0];
                        G(0, 3) = 1;
                        G(1, 4) = 1;
                        G(2, 5) = 1;

                        Eigen::MatrixXf tempA(6, 6);
                        Eigen::VectorXf tempbias(6);
                        tempA = G.transpose() * transformedSourceNormal * transformedSourceNormal.transpose() * G;
                        tempbias = G.transpose() * transformedSourceNormal * transformedSourceNormal.transpose() * (transformedSourceVertex - transformedTargetVertex);
                        //std::cout << "temp A  " << std::endl << tempA << std::endl;
                        //std::cout << "temp bias " << std::endl << tempbias << std::endl;
                        A = A + tempA;
                        b = b + tempbias;
                    }
                }
                else
                {
                    //std::cout << "source invalid" << std::endl;
                    //std::cout << "match size" << match.size()<< std::endl;
                    //match.erase(it);
                    //std::cout << "after erasing, match size" << match.size() << std::endl;
                    num++;
                }
              
                }
            std::cout << "and valid num is " << valid_num << std::endl;
            Matrix4f result;
            std::cout << "A " <<A << std::endl;
            std::cout << "b " << b << std::endl;
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
            result(3, 3) = 1;
            result(3, 1) = 0;
            result(3, 2) = 0;
            result(3, 0) = 0;
            return result;
            
            }
        
        

        //Matrix4f calculateIncremental(const Map2DVector3f& targetVertex,
        //    const Map2DVector3f& sourceVertex,
        //    const Map2DVector3f& sourceNormals,
        //    std::map<int, int>& match) 
        //{
        //    std::cout << "now match size" << match.size() << std::endl;
        //    MatrixXf A = MatrixXf::Zero(6, 6);
        //    Eigen::VectorXf b= VectorXf::Zero(6);
        //    std::map<int, int>::iterator it;
        //    for (it = match.begin(); it != match.end(); it++) {
        //        int index_target = it->first;
        //        int index_source = it->second;
        //        Eigen::Vector3f targetPoint = targetVertex.get(index_target);
        //        Eigen::Vector3f sourcePoint = sourceVertex.get(index_source);
        //        Eigen::Vector3f sourceNormal = sourceNormals.get(index_source);

        //        std::cout << "target point  " << targetPoint << std::endl;
        //        std::cout << "source point " << sourcePoint << std::endl;
        //        std::cout << "source normal" << sourceNormal << std::endl;
        //        
        //        MatrixXf  G(3, 6);
        //        G(0, 0) = 0;
        //        G(1, 1) = 0;
        //        G(2, 2) = 0;
        //        G(0, 1) = -targetPoint[2];
        //        G(0, 2) = targetPoint[1];
        //        G(1, 2) = -targetPoint[0];
        //        G(1, 0) = targetPoint[2];
        //        G(2, 0) = -targetPoint[1];
        //        G(2, 1) = targetPoint[0];
        //        G(0, 3) = 1;
        //        G(1, 4) = 1;
        //        G(2, 5) = 1;

        //        Eigen::MatrixXf tempA(6, 6);
        //        Eigen::VectorXf tempbias(6);
        //        tempA = G.transpose() * sourceNormal * sourceNormal.transpose() * G;
        //        tempbias = G.transpose() * sourceNormal * sourceNormal.transpose() * (sourcePoint - targetPoint);
        //        //std::cout << "temp A  " << std::endl << tempA << std::endl;
        //        //std::cout << "temp bias " << std::endl << tempbias << std::endl;
        //        A = A + tempA;
        //        b = b + tempbias;
        //    }
        //    Matrix4f result;
        //    //td::cout << "A size" <<A.size()<< std::endl;
        //    //std::cout << "b size" << b.size() << std::end;
        //    VectorXf x = A.ldlt().solve(b);
        //    result(0, 0) = 1;
        //    result(1, 1) = 1;
        //    result(2, 2) = 1;
        //    result(0, 3) = x[3];
        //    result(1, 3) = x[4];
        //    result(2, 3) = x[5];
        //    result(1, 2) = x[0];
        //    result(2, 1) = -x[0];
        //    result(0, 2) = -x[1];
        //    result(2, 0) = x[1];
        //    result(0, 1) = x[2];
        //    result(1, 0) = -x[2];
        //    result(3, 3) = 1;
        //    std::cout << "RESULT" << result << std::endl;
        //    return result;
        //}
		
        /*void frame2frameEstimation(Eigen::MatrixXf inputTransformationMatrix, Level level)
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
        }*/

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
        Eigen::MatrixXf m_currentTransformation;
    };
}