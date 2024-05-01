/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{

//pMap�������ĵ�ͼ��nIterations�������Ż������Ĵ���=10��pbStopFlag��ʾ�Ƿ���Ҫֹͣȫ���Ż�(��������)
//nLoopKF�ǵ�ǰ֡�����ţ�bRobustĬ��ֵ��false
//���������ڻػ������б�����
void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


//vpKFs�ǵ�ͼ�����еĹؼ�֡��vpMP�ǵ�ͼ�����еĵ�ͼ��
//nIterations�Ƿ������Ż������Ĳ��裬pbStopFlag��ʾ�Ƿ���Ҫֹͣȫ���Ż�(��������)
//nLoopKF�ǵ�ǰ֡�����ţ�bRobustĬ��ֵ��false
//����������������:����=�ؼ�֡��λ��+��ͼ��
//ȫ���Ż�������ע��ͼ���͹ؼ�֮֡����ͶӰ��ϵ����������֡��֮֡�����Ż���ϵ�������Ż�����Զ���ɹؼ�֡��λ�˺͵�ͼ�㹹�ɵ�
void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();//���Է���������-ʹ�õ���dense_schur

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);//��������=BlockSolver< BlockSolverTraits<6, 3> >

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);//����Ӧ����g2o�ı�־λ��ʹ���Ż�ֹͣ

    long unsigned int maxKFid = 0;//��¼���ߵĹؼ�֡����

    // Set KeyFrame vertices
    //������ͼ�е����йؼ�֡�����ؼ�֡����Ϊg2o�Ķ���
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    //������ͼ�е����еĵ�ͼ�㣬��g2o���ӵ�ͼ�㲢����g2o�����ӱ�
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;//��ͼ����g2o����������=��ͼ������+���߹ؼ�֡����+1
        vPoint->setId(id);
		//����һ��Ҫע��!!!!!!!!!!!!!!!!!!!!!!g2o�����ȱ�Ե����ͼ���ڽ����Է�������ֻ����������λ�ˣ�����slam14���е�264ҳ
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);//����ͼ�����ӵ�g2o�Ķ�����

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();//������ǰ��ͼ�������йؼ�֡

        int nEdges = 0;//��¼��g2o���ӵı���
        //SET EDGES
        //��g2o�����Ӻʹ˵�ͼ���йصıߣ�ע��ȫ���Ż�������ע��ͼ���͹ؼ�֮֡����ͶӰ��ϵ����������֡��֮֡�����Ż���ϵ
        //����������ǰ��ͼ�������йؼ�֡
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;//�ĸ��ؼ�֡������������ͼ��
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];//��ǰ��ͼ���ڱ������Ĺؼ�֡ͼ���е�������(������������)

            if(pKF->mvuRight[mit->second]<0)//����������ͼ��ֻ������ͼ���б�����
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();//ע�������ıߺ������ı��ǲ�һ���ģ��˴��ı���

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//���ɵ�ͼ���Ϳ����˵�ͼ���Ĺؼ�֡����
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);//����ֵ�ǵ�ǰ��ͼ���ڱ������Ĺؼ�֡ͼ���е���������������
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else//����������ͼ������ͼ����Ҳ������
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();//ע�������ıߺ������ı��ǲ�һ���ģ��˴��ı���˫Ŀ��

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data
    //ʹ��ȫ���Ż��õ��Ľ������µ�ͼ���͹ؼ�֡��λ��
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)//������ǰ֡��������0
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

//���ص�����Ч���Ż�����
//������������Ҫ������:��ǰ֡��ͼ��(mvpMapPoints)��λ����֪���Һ͵�ǰ֡��ͼ���ϵ�ƥ����ϵҲ�Ѿ��������Ż���ǰ֡��λ��
//����һԪ�ߣ�ֻ�Ե�ǰ֡��λ�˽����Ż����Ż���ͼ����λ��
//�������Ż�֮�������ٽ���ǰ֡��λ�˷�����������ǰ֡��Щ��ͼ������ֵ�㣬�����µ�ǰ֡��mvbOutlier��ͬʱ���ö�Ӧ�ߵĵȼ������ٴν����Ż�
//���˷���һ������4�Σ������Ż���������Ч�ߵĸ���С��10����ֹͣ�´��Ż�ֱ������������
//����������tracking�߳��е�TrackReferenceKeyFrame������TrackWithMotionModel�����е��ã�Ŀ�ľ���ͨ����һ֡�Ż��õ���ǰ֡��λ��
int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
	//����һ��Ҫע����!!!���������������һ����˫Ŀ������������ά����3���ڶ�����ֻ��Ŀ����������ά����2��������������ͳһʹ�õ���6,3�Ŀ�������
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;//�������Ż�ά��Ϊ6������ֵά��Ϊ3��
    
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);//��һ���㲻����Ϊ�̶������ĵ㣬��ζ�Ų����Ż�
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;//�����洢�Ķ��Ǳ�
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);
    //��ѯ��ǰ֡�е����е�ͼ��
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);//���õĺ˺��������Ǹ�����5.991

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;//��ʼ��ʱ��Ϊ��ͼ�����еĵ�ͼ�㶼����outlier�����ܹ�����g2o�������Ż�

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                //����ͼ��ͶӰ����������ϵ�µ�����ƽ�档�Ż�����ֻ��pose����ͼ��λ�ù̶�����һ��Ԫ��
                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];//��������ͼ���б����Ĳ���Խ���������ߵ�Ȩ��ԽС��������Խ����Ҫ
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);//����7.815

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)//������ʼ��ƥ������С��3
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};//�Ҹо�����ֵ��������Ϊ�趨��
    const int its[4]={10,10,10,10};//����ÿ���Ż��Ĵ���    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);//��ʼ�����Ż�!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        optimizer.optimize(its[it]);

        nBad=0;
        //������˫Ŀ��ִ��������forѭ��
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        //������˫Ŀ��ִ��������forѭ��
        //�Ż�֮����Щ��ͼ������ֵ�㣬��Ҫ������Ӧ�ıߵĵȼ�Ȼ���ٽ����Ż�
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();//error����Omega*error, �����������ܴ���˵���˱ߵ�ֵ�������ߺܲ�����

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);//���ñߵļ�����ʲô��˼????????????
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);//���ú˺���
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

//�����Ĳ���pKF�ǵ�ǰ֡
//pbStopFlag�Ƿ�ֹͣ��pMap�����ĵ�ͼ
//���Ż��Ĳ����Ǳ���֡(=��ǰ֡+��ǰ֡�Ĺ���֡)�ͱ��ص�ͼ��
//��ʹ�÷������Ż�(��һ�ε�������5��)�õ�һ������Ȼ������Щ�����Ƚϴ��ı����õȼ�Ϊ1Ȼ���ٽ���һ���Ż�(�ڶ��ε���������10��)������֮ǰ�����Ƚϴ��ıߵĶ�Ӧ�����������Ӹ������ݽṹ��ɾ��
//�Ż����ɺ������µı��ص�ͼ���ͱ���֡��λ��
//��������ֻ��locamapping�߳��б����ã���Ҫ���������Ż�����֡��λ�˺ͱ��صĵ�ͼ��
void Optimizer:: LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    
    list<KeyFrame*> lLocalKeyFrames;//�洢���ǵ�ǰ֡�͹���֡�����Ƕ��嵱ǰ֡+��ǰ֡�Ĺ���֡=����֡

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();//��ǰ֡�Ĺ���֡
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;//�洢���Ǳ���֡�ĵ�ͼ��
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;//����֡�ĵ�ͼ��
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    //��������֡��ͼ���������ؼ�֡(����������֡)�����Ż������ǲ��Ż���Щ�ؼ�֡����Ϊ�̶��Ĳ�����
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();//�������Ż�ά��Ϊ6������ֵά��Ϊ3

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    //���������Ż������Ӵ��Ż����㣬����=����֡��λ��
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }
 
    // Set Fixed KeyFrame vertices
    //�������Ի������ӹ̶����㣬����=��������֡��ͼ���������ؼ�֡
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;//���ڴ洢�Ż��ı�
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;//���ڴ洢��Ϊ�����ı���֡
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;//���ڴ洢��Ϊ�����ı��ص�ͼ��
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);//������huber�˺����Ĳ���������slam14��256ҳ

    //�������ص�ͼ��=����֡�����ĵ�ͼ��
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {   
        //�����ص�ͼ�����뵽�������Ż��Ķ�����
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;//��ͼ���Ķ��������ڹؼ�֡�ı���֮��
        vPoint->setId(id);
        vPoint->setMarginalized(true);//����һ��Ҫע��!!!!!!!!!!!!!!!!!!!!!!g2o�����ȱ�Ե����ͼ���ڽ����Է�������ֻ����������λ��
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        //��������������ͼ�������йؼ�֡
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//��ͼ���Ķ���
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));//����λ�˵Ķ���
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation ���������Ż��м����ߣ�����ֵ�ǵ�ͼ��������֡����������
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;//�۲�ֵ�ǵ�ͼ���ڱ���֡��ͶӰ��������
                    //���������Ż��м�����!!
                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
                    //����������������=��ͼ��+����������ͼ���Ĺؼ�֡
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));//��ͼ���Ķ�������
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));//����������ͼ���Ĺؼ�֡����
                    e->setMeasurement(obs);
		      //�ߵ���Ϣ�����ɵ�ͼ���ڹؼ�֡��Ӧ���������Ľ�������������
		      //��������Խ��������ϵ���ͻ�ԽС����ζ�ſ��ŶȽ���
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();//��ʼ��һ���Ż�!!!!!!!!
    optimizer.optimize(5);

    bool bDoMore= true;

   //�ٴ��ж��Ƿ�ֹͣ�Ż�
    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    //�������Ƚϴ��ı����óɵȼ�1��Ȼ���ٽ���һ���Ż�
    if(bDoMore)
    {

	    // Check inlier observations
	    //�����ڵ�Ŀ��˫Ŀ����ֱ�Ӻ���
	    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
	    {
	        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
	        MapPoint* pMP = vpMapPointEdgeMono[i];

	        if(pMP->isBad())
	            continue;

	        if(e->chi2()>5.991 || !e->isDepthPositive())
	        {
	            e->setLevel(1);
	        }

	        e->setRobustKernel(0);
	    }

	     //������˫Ŀ���������еı�
	    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
	    {
	        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
	        MapPoint* pMP = vpMapPointEdgeStereo[i];

	        if(pMP->isBad())
	            continue;

	        if(e->chi2()>7.815 || !e->isDepthPositive())//���������ߵ�����̫�󣬻��ߵ�ͼ�㲻������֡�ķ�Χ��
	        {
	            e->setLevel(1);//??????????????????????���������ı�����Ϊ�ȼ�1��ɶ��˼
	        }

	        e->setRobustKernel(0);//���ﲻ��ʹ�ú˺���
	    }

    	   // Optimize again without the outliers

          optimizer.initializeOptimization(0);//�ٽ���һ���Ż�
          optimizer.optimize(10);

    }

    //�������Ż�����֮�󣬻�����һЩ�������Ƚϴ�������Ҫ�������߶�Ӧ�Ķ���(��ͼ��+�ؼ�֡)ɾ����
    //����ɾ���ı߱�����vToErase
    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations  
    //�����ڵ�Ŀ��˫Ŀ����ֱ�Ӻ���
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }
   //������˫Ŀ
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;//Ҫɾ���ı߶�Ӧ�Ĺؼ�֡
            MapPoint* pMPi = vToErase[i].second;//Ҫɾ���ı߶�Ӧ�ĵ�ͼ��
            pKFi->EraseMapPointMatch(pMPi);//��������ͼ���������ؼ�֡��ɾ����Ҳ����˵�����ؼ�֡û�й۲⵽������ͼ��
            pMPi->EraseObservation(pKFi);//�������ؼ�֡��������ͼ����ɾ����Ҳ����˵������ͼ��û�б������ؼ�֡�۲⵽
        }
    }

    // Recover optimized data
    //�Ż����ɺ������µı��ص�ͼ���ͱ���֡��λ��

    //Keyframes
    
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}



//pMp�ǵ�ͼ��pLoopKF���뵱ǰ֡��ƥ�������űջ���ѡ֡,pCurKF�ǵ�ǰ�ؼ�֡��
//CorrectedSim3�洢���ǵ�����֮���ĵ�ǰ�ؼ�֡�Ĺ���֡�͵�ǰ�ؼ�֡����������ϵ�µ�λ��
//NonCorrectedSim3�ǵ���֮ǰ��ǰ�ؼ�֡�Ĺ���֡�͵�ǰ�ؼ�֡����������ϵ�µ�λ��
//LoopConnections�ǻػ�֡=�����ػ��ںϺ��²����Ĺ�ϵ
//bFixScale=��˫Ŀ����������true
//����������������:����=��ͼ�е����йؼ�֡(���Ż����űջ���ѡ֡��λ��)
//��=�ػ�֮֡��������λ�˱仯,��С������֮���ؼ�֡��λ�˱仯�����йؼ�֡����Ȩ�ش���100�Ĺ���֡��λ�˱仯
//��������ֻ�ڻػ��߳��б����ã���Ҫ���Ż���ͼ�����йؼ�֡��λ��
//�����Ż�����Ϊ20�Σ�ʹ��LM����������������ʹ�õ���LinearSolverEigen��ϡ��cholesky����ʹ�õ���g2o�Դ���EdgeSim3�ߣ�������û�ж����ſ˱Ⱥ�����Ҳ����ζ����ʹ�õ�����ֵ�ſ˱Ƚ��е�����
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);//���򿪵�������
    //7��ÿ���������Ż�ά�ȣ�3������ֵά��
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);//���������ڳ�ʼ��leverberg���㷨����
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();//��ͼ�����еĹؼ�֡
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();//��ͼ�����еĵ�ͼ��

    const unsigned int nMaxKFid = pMap->GetMaxKFid();//��ǰ��ͼ�йؼ�֡����������ֵ

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);//�����ǹؼ�֡�ڵ�ͼ�е����ţ������ǹؼ�֡��λ�ˣ���������֡�Ǳջ��ؼ�֡�������乲��֡�����õ�������λ�������µ�ǰ֡�ؼ�֡��λ��
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);//�����Ǳ����������Ż����ĸ�������
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);//�����ǹؼ�֡�ڵ�ͼ�е����ţ������ǹؼ�֡��λ�˶�Ӧ�Ķ���

    const int minFeat = 100;//�ڹ���ͼ��Ȩ�ش���minFeat�ı߲Ż����뵽g2o���Ż�����

    // Set KeyFrame vertices
    //������ͼ�е����йؼ�֡��Ŀ����ʹ�ùؼ�֡��λ�˵õ�g2o�Ķ���
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();//����һ�����͵�sim3��g2o����

        const int nIDi = pKF->mnId;//�ؼ�֡������

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())//�ж������ؼ�֡�Ƿ��ڻػ��б�������λ�ˣ������ҵ�����ʹ�õ�������λ�������¶�����λ��
        {
            vScw[nIDi] = it->second;//�������ĵ�ǰ֡�Ĺ���֡��λ�˸���vScw
            VSim3->setEstimate(it->second);//Ӧ��������sim3�����ĳ�ֵ
        }
        else//û���ҵ� û�е����� ��ʹ��ԭ����λ�������¶���λ��
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);//��ʹ������֡��λ�������¶�����λ��
        }

        if(pKF==pLoopKF)//����������ͼ�еĹؼ�֡�����űջ���ѡ�ؼ�֡���򲻶��������������Ż�
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;//�̶�sim3�еĳ߶�

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }

    //�������ǻػ�֡�в����ıߣ�һ�������������㹹��
    //��¼��������������Ӧ�Ĺؼ�֡�����ţ�����С������һ����ǰ��
    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    //�����ػ�֡�������ػ�֡����������һ������֡�Ͷ�������֡�Ĺ�ϵ���뵽�Ż��ı���!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;//��ǰ֡�Ĺ��ӹؼ�֡
        const long unsigned int nIDi = pKF->mnId;//��ǰ֡�Ĺ��ӹؼ�֡������
        const set<KeyFrame*> &spConnections = mit->second;//��ǰ֡�Ķ������ӹؼ�֡
        const g2o::Sim3 Siw = vScw[nIDi];//�õ���ǰ֡����֡��λ��
        const g2o::Sim3 Swi = Siw.inverse();
        //������ǰ֡�Ķ������ӹؼ�֡
        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];//�������ӹؼ�֡��λ��
            const g2o::Sim3 Sji = Sjw * Swi;//��������֡��һ������֡������λ�˱仯

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;//��Ϣ������һ����λ�������Ҹо����������Ż�??????????????????

            optimizer.addEdge(e);//��g2o�����ӵı�

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    //������ͼ�е����йؼ�֡
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;
        //�ڵ�ǰ֡�Ĺ���֡��Ѱ�ҵ�ͼ�������ؼ�֡
        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())//�ҵ���
            Swi = (iti->second).inverse();//û�лػ������Ĺؼ�֡λ��
        else
            Swi = vScw[nIDi].inverse();

	    //������֡��Ӧ����С�������еı߼��뵽g2o��!!!!!!!!!!!!!!!!!!!!!!!!!!
        KeyFrame* pParentKF = pKF->GetParent();//�õ���ͼ�ؼ�֡�ĸ��ؼ�֡

        // Spanning tree edge
        if(pParentKF)//����������ͼ�ؼ�֡���ڸ��ؼ�֡
        {
            int nIDj = pParentKF->mnId;//���ؼ�֡������

            g2o::Sim3 Sjw;
             //�ڵ�ǰ֡�Ĺ���֡��Ѱ���������ؼ�֡
            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;//�õ�������ͼ�ؼ�֡���丸�ؼ�֮֡��������λ�˱仯

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges �����űջ���ѡ�ؼ�֡������λ�˼��뵽λ���Ż���
        //��������ͼ�ؼ�֡��Ӧ�ıջ��ؼ�֡���γɵı߼��뵽g2o��!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //����������֤��������
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();//�õ�������ͼ�ؼ�֡��Ӧ�ıջ��ؼ�֡
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)//pKFһ���ǵ�ǰ֡
            {
                g2o::Sim3 Slw;
                //�ڵ�ǰ֡�Ĺ���֡��Ѱ�ҵ�ͼ�ؼ�֡��Ӧ�ıջ��ؼ�֡
                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);//pLKF=���űջ���ѡ֡

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;//�õ���ǰ��ͼ֡�Ͷ�Ӧ�ıջ��ؼ�֡������λ�˱仯
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        //�����뵱ǰ�ؼ�֡�����Ĺ��ӹؼ�֡(Ȩ����Ҫ������ֵ)
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);//���õ�ǰ�ؼ�֡����ͼȨ�ش���minFeat�Ĺؼ�֡
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))//Ϊ�˷�ֹ�ظ��ӱ�
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))//����ҲӦ���Ƿ�ֹ�ظ��ӱ�
                        continue;

                    g2o::Sim3 Snw;
					
                    //�ڵ�ǰ֡�Ĺ���֡��Ѱ�ҹ���֡
                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];//��ǰ��ͼ����֡��λ��

                    g2o::Sim3 Sni = Snw * Swi;//��ǰ��ͼ֡���乲��֡������λ�˱仯

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!�Ѿ������˱ߣ���ʼ�Ż�!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    //�ٱ���һ����ͼ�е����йؼ�֡�������Ƿ������Ż��õ��ĸ���������λ��ȥ���µ�ͼ�ؼ�֡��λ��
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    //������ͼ���е�ͼ��
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;//Ӧ�����ж�������ͼ�����ĸ��ؼ�֡�и���
        //
        if(pMP->mnCorrectedByKF==pCurKF->mnId)//�õ���ͼ���ڱջ�����������Ϊ��һ����ǰ֡����������������������֡���ڵ�ǰ֡
        {
            nIDr = pMP->mnCorrectedReference;//��ǰ������Ҫ�������ĵ�ͼ�����ڵ�ǰ֡�е��ĸ�����֡�У�����������֡���Ÿ�ֵ��nIDr
        }
        else//������ΪʲôҪ�õ�ͼ���Ĳο��ؼ�֡������???????????
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];//�õ��ؼ�֡��λ��
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];//�����������Ż�֮���Ĺؼ�֡��λ��

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));//�õ�У�����ĵ�ͼ��λ��=ԭ����ͼ����λ��*������ͼ���ο��ؼ�֡֡λ�˵ĵ���

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

//pKF1�ǵ�ǰ�ؼ�֡��pKF2�Ǳջ���ѡ֡��vpMatches1�Ǿ���ransac���̺�searchbysim3����ǰ֡�ͱջ���ѡ֡ƥ���ĵ�ͼ�㣬�����ǵ�ǰ֡ͼ���������������ţ������Ǻ�ѡ�ջ��ؼ�֡����Ӧƥ���ĵ�ͼ��(����һ��������������������)
//th2�������趨ĳЩ����������ֵ�����������ߵ������th2��
//g2oS12��һ�����������������������������ǵ�ǰ֡�ͱջ���ѡ֡����ransac���̼����õ�������λ�˱仯��������������ͨ���������Ż��õ�������λ�˱仯������
//���ص����α�����ͨ���Ż�ɸѡ��ʣ����inliner��ͼ��
//ͨ���Ż��ķ�ʽ�õ����Ž�Ȼ��ɸѡ��Щ�����ϴ��ıߣ�����Щ�߶�Ӧ�Ķ�����vpMatches1��ɾ��������Щ�������ıߴӷ������Ż���ɾ�������ٽ���һ�η������Ż�����ɸѡ�������ϴ��ıߣ�������Щ�߶�Ӧ�Ķ�����vpMatches1��ɾ��
//���Ż��Ķ���:��ǰ֡�ͱջ���ѡ֡������λ�ˣ��̶�����:��ǰ֡��ͼ���ڵ�ǰ֡����ϵ�µĵ�����;�Ѿ�ƥ���ĵ�ͼ���ڱջ���ѡ֡����ϵ�µ�����
//��=�뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ���ڱջ���ѡ֡����ϵ�µ�λ��+��ǰ֡�ͱջ���ѡ֡������λ��
//��=����ǰ֡��ͼ���ڵ�ǰ֡����ϵ�µ�λ��+��ǰ֡�ͱջ���ѡ֡������λ��
//�Ҹо�����������icpô
//��������ֻ�ڻػ��߳��б����ã���Ҫ���Ǽ��㵱ǰ�ؼ�֡����ѡ֡������λ��
int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);//��ransac�õ�������λ������Ϊ�Ż��ĳ�ֵ
    vSim3->setId(0);
    vSim3->setFixed(false);//��˼�����������Ǳ�����Ҫ���Ż�
    vSim3->_principle_point1[0] = K1.at<float>(0,2);//cx
    vSim3->_principle_point1[1] = K1.at<float>(1,2);//cy
    vSim3->_focal_length1[0] = K1.at<float>(0,0);//fx
    vSim3->_focal_length1[1] = K1.at<float>(1,1);//fy
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);//����ǰ֡�ͱջ���ѡ�ؼ�֡������λ�˼��뵽������!!!!!!!!!!!!!!!!!!!!!!!!!!11

    // Set MapPoint vertices
    const int N = vpMatches1.size();//inliner��ͼ������
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();//��ǰ֡�ĵ�ͼ��
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;//�������Ǳջ���ѡ�ؼ�֡�е�������ͶӰ����ǰ֡�� �ı�
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;//�������ǵ�ǰ֡�е�������ͶӰ���ջ���ѡ֡�� �ı�
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];//��ǰ֡��ͼ��
        MapPoint* pMP2 = vpMatches1[i];//��ǰ֡�ͱջ���ѡ֡ƥ���ĵ�ͼ��

        const int id1 = 2*i+1;//��������Ӧ��ǰ֡��ͼ���ڵ�ǰ֡�����µ�λ��
        const int id2 = 2*(i+1);//ż�����뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ�� �ڱջ���ѡ֡�����µ�λ��

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);//�Ѿ�ƥ���ĵ�ͼ���ڱջ���ѡ֡�е�����

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();//��ǰ֡��ͼ������������ϵ�µ�λ��
                cv::Mat P3D1c = R1w*P3D1w + t1w;//��ǰ֡��ͼ���ڵ�ǰ֡����ϵ�µ�λ��
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);//��˼�����������ǹ̶�������Ҫ���Ż�
                optimizer.addVertex(vPoint1);//����ǰ֡��ͼ���ڵ�ǰ֡�����µ�λ�ü��뵽������

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();//�뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ�� ����������ϵ�µ�λ��
                cv::Mat P3D2c = R2w*P3D2w + t2w;//�뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ�� �ڱջ���ѡ֡����ϵ�µ�λ��
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);//��˼�����������ǹ̶�������Ҫ���Ż�
                optimizer.addVertex(vPoint2);//�뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ�� �ڱջ���ѡ֡�����µ�λ�ü��뵽������
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        //��=�뵱ǰ֡�������Ѿ�ƥ���ıջ���ѡ֡�ĵ�ͼ�� �ڱջ���ѡ֡����ϵ�µ�λ��+��ǰ֡�ͱջ���ѡ֡������λ��
        //����ֵ�ǵ�ǰ֡������������������
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];//��ǰ֡������
        obs1 << kpUn1.pt.x, kpUn1.pt.y;//��ǰ֡��ͼ����ͼ���е�����

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));//��Ҫ�Ż�������λ�˶���
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);//����һ����Ҫ�������õ���Ϣ������������ͼ��������������Զ�����ǽ�������ͼ����Ӧ�ıߵ����Ŷȱ��ͣ�Ҳ����Ȩ�رߴ�

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        //��=����ǰ֡��ͼ���ڵ�ǰ֡����ϵ�µ�λ��+��ǰ֡�ͱջ���ѡ֡������λ��
        //����ֵ�Ǳջ���ѡ֡�е���������
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];//�ջ���ѡ�ؼ�֡��������
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));//��Ҫ�Ż�������λ�˶���
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);//�������Ǳջ���ѡ�ؼ�֡�е�������ͶӰ����ǰ֡�� �ı�
        vpEdges21.push_back(e21);//�������ǵ�ǰ֡�е�������ͶӰ���ջ���ѡ֡�� �ı�
        vnIndexEdge.push_back(i);//�����汣�����ǵ�ͼ��������
    }

    // Optimize!
    optimizer.initializeOptimization();//���еĵ�һ���Ż�!!!!!!!!!!!!!!
    optimizer.optimize(5);

    // Check inliers
    //ͨ���Ż��ķ�ʽ�õ����Ž�Ȼ��ɸѡ��Щ�����ϴ��ıߣ�����Щ�߶�Ӧ�ĵ�ͼ��ɾ���Ӷ�����vpMatches1
    //���Ҹ���g2o�ıߣ�Ϊ�´��Ż���׼��
    int nBad=0;//��¼��Щ�������ֵ�ıߵĸ���
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;
        
        if(e12->chi2()>th2 || e21->chi2()>th2)//chi2 ���� error.dot(Omega*error), �����������ܴ���˵���˱ߵ�ֵ�������ߺܲ�����
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;//����outlier�Ƚ϶������Ż�����
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)//�ܹ��ĵ�ͼ����-outlier����<10����Ϊ�Ż�ʧ�ܣ�inliner����̫����
        return 0;

    // Optimize again only with inliers
    
    optimizer.initializeOptimization();//����ֵ��ɾ�����ٴ��Ż�!!!!!!!!!!!!!!!!!!!!!!!
    optimizer.optimize(nMoreIterations);

    //����ȥ����outlier���µıߣ�����һ�ε�ɸѡ��outlier��������һ��ֻ�Ǹ���vpMatches1�����ٸ���g2o�ı���
    int nIn = 0;//��ǰinliner����Ŀ
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)//chi2 ���� error.dot(Omega*error), �����������ܴ���˵���˱ߵ�ֵ�������ߺܲ�����
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;//��ǰinliner����Ŀ
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();//�õ����շ������Ż��Ľ���

    return nIn;
}


} //namespace ORB_SLAM
