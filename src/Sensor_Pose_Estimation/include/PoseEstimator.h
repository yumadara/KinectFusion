#include "ProjectiveCorrespondenceSearch.h"
extern float epsilon_theta = 0.939;
extern float epsilon_d = 0.01;
#include <virtual_sensor.h>
#include <data_frame.h>
#include <fstream>
extern std::map<kinect_fusion::Level, int> iteration_num_with_level = { {kinect_fusion::Level::First, 4}, {kinect_fusion::Level::Second,5}, {kinect_fusion::Level::Third,10} };

namespace kinect_fusion {

	class PoseEstimator {
	public:
		PoseEstimator(FrameData& currentFrameData, FrameData& lastFrameData, Eigen::MatrixXf& lastTransformation , Eigen::MatrixXf& currentTransformation)
		{
            
            m_currentFrameData = currentFrameData;
            m_lastFrameData = lastFrameData;


			m_lastTransformation = lastTransformation;
            m_currentTransformation = currentTransformation;
            //std::cout << "current frame data second level in constructor" << currentFrameData.getSurface(kinect_fusion::Level::Second).getVertexMap().size() << std::endl;
            //std::cout << "current frame data second level in constructor" << m_currentFrameData.getSurface(kinect_fusion::Level::Second).getVertexMap().size() << std::endl;
		}
        Vector3f TransformVertex(Vector3f vertex, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0, 0, 3, 3) * vertex + Transformation.block(0,3,3,1);

        }
        Vector3f TransformNormal(Vector3f normal, Eigen::MatrixXf Transformation)
        {
            return Transformation.block(0,0,3,3)* normal;
        }

        Map2DVector3f TransformVertexMap(Map2DVector3f& vertexMap, Eigen::MatrixXf Transformation)
        {
            Map2DVector3f result = vertexMap;
            for (int i = 0; i != vertexMap.size(); i++)
            {
                result.set(i,Transformation.block(0, 0, 3, 3) * vertexMap.get(i) + Transformation.block(0, 3, 3, 1));
            }
            return result;
        }

        Map2DVector3f TransformNormalMap(Map2DVector3f& normalMap, Eigen::MatrixXf Transformation)
        {
            Map2DVector3f result = normalMap;
            for (int i = 0; i != normalMap.size(); i++)
            {
                result.set(i, Transformation.block(0, 0, 3, 3) * normalMap.get(i)) ;
            }
            return result;
        }

        /// <summary>
        /// 
        /// 
        /// <param name="sourceNormals" is transformed to global frame, which is N^{g}_{k-1} 
        /// <param name="targetNormals" is transformed to global frame, which is R^{z}_{g,k}*Nk(u)
        /// <param name="sourceVertices" is transformed to global frame, which is V^{g}_{k-1}
        /// <param name="targetVertices" is transformed to global frame, which is T^{z}_{g,k}*Vk(u)
        /// <param name="matches"></param>
    std::map<int, int> pruneCorrespondences(const Map2DVector3f& sourceNormals,
        const Map2DVector3f& targetNormals,
        const Map2DVector3f& sourceVertices,
        const Map2DVector3f& targetVertices,
        std::map<int, int>& match) {
        //std::cout << "in prunecorrespondences " << match.size() << std::endl;
        std::map<int, int> result_map;
        
        int valid_num = 0;
        std::map<int, int>::iterator it;


        float average_dis = 0;
        float loss = 0;

        int num_dis = 0;
        

        //myfile.open("./test.txt");
        for (it = match.begin(); it != match.end(); it++) {
            //std::cout << "now match size in prune" << match.size() << std::endl;
            int index_target = it->first;
            int index_source = it->second;

            Vector3f sourceNormal = sourceNormals.get(index_source);
            Vector3f targetNormal = targetNormals.get(index_target);

            Vector3f sourceVertex = sourceVertices.get(index_source);
            Vector3f targetVertex = targetVertices.get(index_target);

            //std::cout << "target Vertex " << targetVertex << std::endl;
            //std::cout << "source Vertex" << sourceVertex << std::endl;

            

            if (!isnan(sourceVertex[0]) && !isnan(sourceVertex[1]) && !isnan(sourceVertex[2]) && !isnan(-sourceNormal[0]) && !isnan(-sourceNormal[1]) && !isnan(-sourceNormal[2])
                && sourceVertex[0] != MINF && sourceVertex[1] != MINF && sourceVertex[2] != MINF && sourceNormal[0] != MINF && sourceNormal[1] != MINF && sourceNormal[2] != MINF &&
                !isnan(targetVertex[0]) && !isnan(targetVertex[1]) && !isnan(targetVertex[2]) && !isnan(-targetNormal[0]) && !isnan(-targetNormal[1]) && !isnan(-targetNormal[2])
                && targetVertex[0] != MINF && targetVertex[1] != MINF && targetVertex[2] != MINF && targetNormal[0] != MINF && targetNormal[1] != MINF && targetNormal[2] != MINF)
            {
                float cosin = (sourceNormal.x() * targetNormal.x() + sourceNormal.y() * targetNormal.y() + sourceNormal.z() * targetNormal.z()) / 
                    (sourceNormal.norm() * targetNormal.norm() + std::numeric_limits<float>::epsilon() );
                float dis = (sourceVertex - targetVertex).norm();
                



                if (cosin >= epsilon_theta && dis <= epsilon_d)
                {
                    //std::cout << "cosin " << cosin << std::endl;
                    //std::cout << "dis" << dis << std::endl;
                    valid_num++;
                    //std::cout << "after pruning, index target " << index_target << std::endl;
                    //std::cout << "after pruning, source target " << index_source << std::endl;
                    result_map.insert(std::pair<int, int>(index_target, index_source));
                    num_dis++;
                    //average_dis += (targetVertex - sourceVertex).norm();
                    loss += abs((targetVertex - sourceVertex).transpose() * sourceNormal);
                }
               
            }
            
        }
        std::cout << "valid num" << valid_num << std::endl;

        //std::cout << " Average distance " << average_dis/num_dis << std::endl;
        std::cout << " Average loss " << loss / num_dis << std::endl;

        return result_map;
    }
        
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="sourceNormals" is transformed to global frame, which is N^{g}_{k-1} 
        /// <param name="sourceVertices" is transformed to global frame, which is V^{g}_{k-1}
        /// <param name="targetVertices" is transformed to global frame, which is T^{z}_{g,k}*Vk(u)
        /// match is correspondence match after pruning
        /// <returns></returns>
        Matrix4f calculateIncremental(const Map2DVector3f& targetVertex,
            const Map2DVector3f& sourceVertex,
            const Map2DVector3f& sourceNormals,
            std::map<int, int> match)
        {
            std::cout << "now match size" << match.size() << std::endl;
            MatrixXf A = MatrixXf::Zero(6, 6);
            Eigen::VectorXf b= VectorXf::Zero(6);
            std::map<int, int>::iterator it;
            
            for (it = match.begin(); it != match.end(); it++) {
                int index_target = it->first;
                int index_source = it->second;
                
                //std::cout << "index_target while calculate " << index_target << std::endl;
                //std::cout << "index_source while calculate " << index_source << std::endl;


                Eigen::Vector3f targetPoint = targetVertex.get(index_target);
                Eigen::Vector3f sourcePoint = sourceVertex.get(index_source);
                Eigen::Vector3f sourceNormal = sourceNormals.get(index_source);
                //std::cout << "target point  " << targetPoint << std::endl;
                //std::cout << "source point " << sourcePoint << std::endl;
                //std::cout << "source normal" << sourceNormal << std::endl;
                
                //assert(abs((targetPoint - sourcePoint).norm()) < 0.01);

                MatrixXf  G = MatrixXf::Zero(3,6);
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

                //std::cout << "G is " << G << std::endl;

                Eigen::MatrixXf tempA = MatrixXf::Zero(6, 6);
                Eigen::VectorXf tempbias = VectorXf::Zero(6);
                tempA = G.transpose() * sourceNormal * sourceNormal.transpose() * G;
                tempbias = G.transpose() * sourceNormal * sourceNormal.transpose() * (sourcePoint - targetPoint);

                A = A + tempA;
                b = b + tempbias;

                
            }
            Matrix4f result;
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
            
            std::cout << "RESULT" << result << std::endl;
            return result;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="inputTransformationMatrix" is the T g, k-1 
        /// <returns T g,k
        Eigen::MatrixXf frame2frameEstimation(Eigen::MatrixXf& inputTransformationMatrix)
        {
            Eigen::MatrixXf tempInputTransformation = inputTransformationMatrix;
            //for (Level level : LEVELS)
            //{

            //    int index = m_currentFrameData.getIndex(level); // index = 0, 1, 2
            //    std::cout << "INEDX" << index << std::endl;
            //    std::cout << "level iteration number " << iteration_num_with_level[level] << std::endl;
            //    Map2DVector3f currentFrameNormal = this->m_currentFrameData.getSurface(2 - index).getNormalMap();
            //    Map2DVector3f currentFrameVertex = this->m_currentFrameData.getSurface(2 - index).getVertexMap();

            //    Map2DVector3f lastFrameNormal = this->m_lastFrameData.getSurface(2 - index).getNormalMap();
            //    Map2DVector3f lastFrameVertex = this->m_lastFrameData.getSurface(2 - index).getVertexMap();

            //    for (int i = 0; i != iteration_num_with_level[level]; i++)
            //    {
            //        //std::cout << "now we are at iteration " << i << std::endl;
            //        //std::cout << "temp Input Transformation " << tempInputTransformation << std::endl;
            //        projectiveCorrespondence pc(currentFrameVertex, currentFrameNormal, inputTransformationMatrix, tempInputTransformation, m_lastFrameData.getCameraIntrinsics(2 - index));
            //        pc.matchPoint();
            //        std::map<int, int> match = pc.getMatch();
            //        std::cout << "correspondence mapping " << match.size() << std::endl;

            //        Map2DVector3f lastFrameNormal_transformed = this->TransformNormalMap(lastFrameNormal, inputTransformationMatrix);
            //        Map2DVector3f lastFrameVertex_transformed = this->TransformVertexMap(lastFrameVertex, inputTransformationMatrix);

            //        Map2DVector3f currentFrameNormal_transformed = this->TransformNormalMap(currentFrameNormal, tempInputTransformation);
            //        Map2DVector3f currentVertex_transformed = this->TransformVertexMap(currentFrameVertex, tempInputTransformation);

            //        std::map<int, int> new_match = this->pruneCorrespondences(lastFrameNormal_transformed, currentFrameNormal_transformed, lastFrameVertex_transformed, currentVertex_transformed, match);
            //        std::cout << "after pruning the new match" << new_match.size() << std::endl;
            //        Matrix4f incremental = calculateIncremental(currentVertex_transformed, lastFrameVertex_transformed, lastFrameNormal_transformed, new_match);

            //        tempInputTransformation = incremental * tempInputTransformation;
            //        std::cout << "/nlevel " << index << "Iteration " << i << " Now tempInputTransformation " << tempInputTransformation << std::endl;
            //    }
            //}

            Level level = Level::First;
            int index = m_currentFrameData.getIndex(level); // index = 0, 1, 2
            std::cout << "INEDX" << index << std::endl;
            //std::cout << "level iteration number " << iteration_num_with_level[level] << std::endl;
            Map2DVector3f currentFrameNormal = this->m_currentFrameData.getSurface( index).getNormalMap();
            Map2DVector3f currentFrameVertex = this->m_currentFrameData.getSurface(index).getVertexMap();

            Map2DVector3f lastFrameNormal = this->m_lastFrameData.getSurface( index).getNormalMap();
            Map2DVector3f lastFrameVertex = this->m_lastFrameData.getSurface( index).getVertexMap();

            //std::cout << "height of image" << m_currentFrameData.getSurface(2-index).getHeight() << std::endl;


            for (int i = 0; i != iteration_num_with_level[Level::Third]; i++)
            {
                //std::cout << "now we are at iteration " << i << std::endl;
                projectiveCorrespondence pc(currentFrameVertex, currentFrameNormal, inputTransformationMatrix, tempInputTransformation, m_lastFrameData.getCameraIntrinsics(index));
                pc.matchPoint();
                std::map<int, int> match = pc.getMatch();
                std::cout << "correspondence mapping " << match.size() << std::endl;

                Map2DVector3f lastFrameNormal_transformed = this->TransformNormalMap(lastFrameNormal, inputTransformationMatrix);
                Map2DVector3f lastFrameVertex_transformed = this->TransformVertexMap(lastFrameVertex, inputTransformationMatrix);

                Map2DVector3f currentFrameNormal_transformed = this->TransformNormalMap(currentFrameNormal, tempInputTransformation);
                Map2DVector3f currentVertex_transformed = this->TransformVertexMap(currentFrameVertex, tempInputTransformation);

                std::map<int, int> new_match = this->pruneCorrespondences(lastFrameNormal_transformed, currentFrameNormal_transformed, lastFrameVertex_transformed, currentVertex_transformed, match);
                std::cout << "after pruning the new match" << new_match.size() << std::endl;
                Matrix4f incremental = calculateIncremental(currentVertex_transformed, lastFrameVertex_transformed, lastFrameNormal_transformed, new_match);

                tempInputTransformation = incremental * tempInputTransformation;
                std::cout << "level " << index << "Iteration " << i << " Now tempInputTransformation " << std::endl;
                std::cout << tempInputTransformation << std::endl;
            }

            return tempInputTransformation;
        }

        Eigen::MatrixXf getCurrentTransformation()
        {
            return m_currentTransformation;
        }

        
	private:
        FrameData m_currentFrameData;
        FrameData m_lastFrameData;
		
		Eigen::MatrixXf m_lastTransformation; // k-1 frame pose estimation
        Eigen::MatrixXf m_currentTransformation;
    };
}