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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

//
void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        //����mbAcceptKeyFrames=false
        //����localmapping�̲߳������µĹؼ�֡
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        //�ж�mlNewKeyFrames�Ƿ�Ϊ�գ������Ϊ������룬˵���������µĹؼ�֡
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();//��Ӧ�������е�6.A,ֻ�ܴ���һ�������Ĺؼ�֡

            // Check recent MapPoints
            MapPointCulling();//��Ӧ�����е�6.B

            // Triangulate new MapPoints
            //�����ȸ��ݽ���֡���ǻ�һЩ��ͼ�㣬��������ͼ��ԭ���ʹ�����ô����SearchInNeighbors������������֡�ĵ�ͼ���ں�
            //��������ͼ�����±��۲⵽����ͱ��½������ˣ�������SearchInNeighbors�������ںϵ�����
            CreateNewMapPoints();//��Ӧ�����е�6.C

	     //�Ҿ��������ǿ����иĽ��ռ��!!!!!!!!!!!!!!!!!!!!��
	     //�����ǽ�localmappingЧ����󻯣�һ�пվ�ִ��localmapping
            if(!CheckNewKeyFrames())//�������Ĺؼ�֡����������
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                //��鵱ǰ֡������֡�ظ���mappoint�������ظ���mappoint�ںϳ�Ϊһ����
                SearchInNeighbors();
            }

            mbAbortBA = false;//����ִ��BA�Ż�

            if(!CheckNewKeyFrames() && !stopRequested())//�������Ĺؼ�֡���������˲���û��ֹͣlocalmapping����
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)//�տ�ʼ��ͼ��ֻ�������ؼ�֡ʱ������BA�Ż�
                     //��localmapping�̴߳���������е����һ���ؼ�֡ʱ����
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);//��Ӧ�����е�6.D

                // Check redundant local Keyframes
                //���и�����:Ϊʲôֻ�������е����һ���ؼ�֡??????????????????????
                KeyFrameCulling();//����Ƕ�Ӧ�����е�6.E
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);//��ػ�����������localmapping�ؼ�֡
        }
        else if(Stop())//if(mbStopRequested && !mbNotStop)�����ֹͣlocalmapping�̵߳�����������localmapping�߳�ֹͣ����
        {
            // Safe area to stop
            //����mbStopped������mbFinishRequested
            //localmapping�̱߳�ֹͣ���Ҳ�Ҫ��ر�localmapping�̣߳������ѭ������3000΢��
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())//����mbFinishRequested�����Ҫ��ر�localmapping�߳����˳�localmapping�̵߳�whileѭ��
                break;
        }
        //���²����Ĺؼ�֡�͵�ͼ�����
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        //mbAcceptKeyFrames=true��
        //localmapping�߳̿��Խ����µĹؼ�֡
        SetAcceptKeyFrames(true);
		
		//����mbFinishRequested����ʾ����ر�localmapping����߳�
        if(CheckFinish())
            break;

        usleep(3000);//����3000΢�3����
    }//���
    //����localmapping�߳��Ѿ��رգ�����localmapping����߳��Ѿ�ֹͣ����
    SetFinish();//mbFinished = true;mbStopped = true;   
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)//���������tracking�߳��б�����
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

//��Ӧ�����е�6.A
//�����������:ʹ�õ�ǰ֡�����¹ؼ�֡��ͼ�����Ϣ�����¹ؼ�֡�ĵ�ͼ����뵽localmap�߳��е��µ�ͼ��mlpRecentAddedMapPoints
                          //�����¹ؼ�֡��Թ���ͼ����Ӱ�죬���¹���ͼ�Ĺ�ϵ��
                          //������µĹؼ�֡���뵽��ͼ��
void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();//mlNewKeyFrames�����ɵĴ��ݸ�localmapping�Ĺؼ�֡
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    //����������Ĺؼ�֡�ĵ��ʽṹ
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    //��õ�ǰ�ؼ�֡�ĵ�ͼ��(��tracking�̵߳õ�)
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    //�����������ؼ�֡�ĵ�ͼ��
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))//��������ͼ��֮ǰû�б���ǰ֡�۲⵽��
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);//����mObservations
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                    mlpRecentAddedMapPoints.push_back(pMP);//mlpRecentAddedMapPoints�������ֻ�����ﱻ����
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    //���ڴ������Ĺؼ�֡��Ҫ�������ڹ���ͼ�еĹ�ϵ
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    //���ͼ����������ؼ�֡
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

//��Ӧ�����е�6.B
//�����¹ؼ�֡�ĵ�ͼ�����ɸѡ�������������mlpRecentAddedMapPoints
void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)//���봫��������
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;
   //����mlpRecentAddedMapPoints�¼���ĵ�ͼ�㣬ɾ����Щ��Ч���¼���ĵ�ͼ�㡣
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
		//�������Ż���Чʹ�������ͼ�����ͨ֡����/ǰ��ƥ��õ�����ͨ֡����
		//��tracking�߳��е��Ż���Ϣ���ж������ͼ���ǲ���Ҫ��ɾ��
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();//ɾ�����������ͼ��Ĺؼ�֡��˫����Ϣ���ӵ�ͼ��ɾ����
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)//δ������2���ؼ�֡���������ҵ�ǰ�ؼ�֡��ID�Ϳ����õ�ĵ�һ���ؼ�֮֡���ID֮����ڵ���2��ʱ��
        {
            pMP->SetBadFlag();//ɾ�����������ͼ��Ĺؼ�֡��˫����Ϣ���ӵ�ͼ��ɾ����
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)//��ǰ�ؼ�֡��ID�Ϳ����õ�ĵ�һ���ؼ�֮֡���ID֮����ڵ���3��ʱ��
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}


//�����Ѿ�֪���˵�ǰ�ؼ�֡������ܹؼ�֡(��ǰ�ؼ�֡������Ȩ������10���ؼ�֡)��λ�ˣ�������Ҫ�����ǽ������ؼ�֡���е�ͼ��ƥ��Ȼ�����ǻ���������һ����ͼ�㣬�����������ͼ�����Ϣ��������صĽ���֡�͵�ǰ�ؼ�֡����Ϣ
//�����ؼ�֡����������ƥ�����˼·��ʹ�ü���Լ����bow���ٵķ���������ƥ�䡣
//���ǻ��Ĺ�������Ҫ�Ƚ�֡���Ӳ��֡���Ӳ�������������ǻ������ǻ������ǵõ������������������ϵ�µ����꣬Ȼ���ٷ�ͶӰ�عؼ�֡�ͽ���֡����ͶӰ���̫��ĵ�ͼ��ɾ������
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);//����ⷽ��һ����

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();//��track�߳������ǵõ��˵�ǰ�ؼ�֡��λ�ˣ���������ȷ
    cv::Mat Rwc1 = Rcw1.t();//��ǰ֡��R��ת��
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();//��ǰ֡��������������ϵ�µ�����

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;//����������߶������Լ���в��õ�=1.5*1.2

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    //�����뵱ǰ�ؼ�֡������Ȩ������10���ؼ�֡�����ǳ���10���ؼ�֡Ϊ���ܹؼ�֡
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)//˫Ŀ���
        {
            if(baseline< pKF2->mb)//���������������ؼ�֡���̫Сʱ������3D��  
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);//��ǰ�ؼ�֡�Ѿ�ƥ���ϵĵ�ͼ����뵱ǰ�ؼ�֡�����z���մ�С���򲢷����м��С�����
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)//�����ͼ����뵱ǰ֡�ľ����ձ��Զ������֡����Ͻ��򲻽�������Ĺ���
                continue;
        }

        // Compute Fundamental Matrix
        //���㵱ǰ�ؼ�֡�����֡�Ļ�������
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);//��֪�����λ�����������

        // Search matches that fullfil epipolar constraint
        //����֮֡��Ļ���������ΪԼ����ƥ����֮֡���������!!!!!!!!!!!!!!!!!
        //���ƥ�����������pkf1����pdf2���Ѿ������˶�Ӧ�ĵ�ͼ���������vMatchedPairs��!!!!!!!
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);//���ΪvMatchedIndices

        cv::Mat Rcw2 = pKF2->GetRotation();//���ܹؼ�֡��R
        cv::Mat Rwc2 = Rcw2.t();//���ܹؼ�֡��Rת��
        cv::Mat tcw2 = pKF2->GetTranslation();//���ܹؼ�֡��t
        cv::Mat Tcw2(3,4,CV_32F);//���ܹؼ�֡��[R,t]
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        //������ǰ�ؼ�֡�ͽ��ܹؼ�֡��ƥ���������
        //��ÿ��ƥ��� 2d-2d ͨ�����ǻ�����3D��,�� Triangulate�������
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;//��ǰ֡��ƥ�����������
            const int &idx2 = vMatchedIndices[ikp].second;//���ܹؼ�֡��ƥ�����������

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];//��ǰ֡����ͼ��������
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];//��ǰ֡��ͼ����������������u
            bool bStereo1 = kp1_ur>=0;//��ǰ֡�Ƿ�����ͼ��

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];//����֡������
            const float kp2_ur = pKF2->mvuRight[idx2];//����֡��ͼ����������������u
            bool bStereo2 = kp2_ur>=0;//����֡�Ƿ�����ͼ��

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);//ͨ����ǰ֡����õ��� ��һ��ƽ���ϵĵ�
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);//ͨ�����ܹؼ�֡����õ��� ��һ��ƽ���ϵĵ�
            
            cv::Mat ray1 = Rwc1*xn1;//
            cv::Mat ray2 = Rwc2*xn2;//
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));//Ӧ����֡����Ӳ��

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;//��ǰ֡���Ӳ��
            float cosParallaxStereo2 = cosParallaxStereo;//����֡֡���Ӳ��

            if(bStereo1)//��ǰ֡�����˫Ŀ�����֡���Ӳ��
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));//�����֡���Ӳ�ǣ���˫Ŀ���ɵ��Ӳ��
            else if(bStereo2))//����֡�����˫Ŀ�����֡���Ӳ��
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));
            
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);//�������ߵ�֡���Ӳ�ǵõ����յ�֡���Ӳ��

            cv::Mat x3D;//��ͼ������������ϵ�µ�����
	     //֡����Ӳ�Ǵ���֡���Ӳ�ǲ���֡���Ӳ�ǲ��Ǻ�С������£�ʹ�����Ƿ�(��������Ϊ�ƶ�����Ƚϴ�)
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                //����ʹ�õ���mvg����312ҳ�ķ�����12.2�ڣ���û��ʹ��slam14���еķ���
                //��Ҫ�õ���֡����������ϵ�µ�λ�ˣ��õ���ֱ���ǵ�����������ϵ�µ�����
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
				
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);//�Ծ���A����SVD�ֽ⡣A=U*W*V.t()

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
	      //����˫Ŀ,���֡����Ӳ�ǱȽ�С����֡���Ӳ��С��֡���Ӳ�ǣ�˭��֡���Ӳ�Ǵ���˭�ָ�����ά������
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
			
            else
                continue; //No stereo and very low parallax.û��˫Ŀ/��� �������ӽǲ�̫С  ���ǲ���Ҳ������ �ò�����ά�� No stereo and very low parallax  

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            //������ɵ�3D���Ƿ������ǰ��  
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            //����3D���ڵ�ǰ�ؼ�֡�µ���ͶӰ���  
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];//���ֲ������������������ڽ���������Ӱ��
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);//�����һ������  
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)//��Ŀ���
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else//˫Ŀ���
            {
                float u1 = fx1*x1*invz1+cx1;//������õ��ĵ�ͼ��ͶӰ����ǰ�ؼ�֡�еõ���������
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;//��ͶӰ�õ������ص�-��ʵ�����ص�
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)//ͶӰ���̫��
                    continue;
            }

            //Check reprojection error in second keyframe
            //����3D���ڽ��ܹؼ�֡�µ���ͶӰ���  
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)//��Ŀ���
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else//˫Ŀ���
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            //���߶�������  
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
			
	      //��ǰ֡�ͽ���֡���������������ڽ��������������ű���֮��
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];
            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            // �����ֵ������ͼ���µĽ������㼶��ֵӦ������
            //��Ϊ������������Զʱ��ô��Ӧͼ��ķֱ��ʱ������ܼ��������㣬���Ӧ�����ű�����Ӧ��С
            //���������ôд��ȽϺ�����:ratioDist<ratioOctave/ratioFactor||ratioDist>ratioOctave*ratioFactor
            //1.5*1.2=ratioFactor
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            //���ǻ�����3D��ɹ�������ɵ�ͼ�� MapPoint
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1); //���µ�ͼ�㣬ʹ�䱻��ǰ�ؼ�֡����
            pMP->AddObservation(pKF2,idx2);//���µ�ͼ�㣬ʹ�䱻����֡����

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);//���µ�ǰ�ؼ�֡����ǰ�ؼ�֡���������ͼ��  
            pKF2->AddMapPoint(pMP,idx2);//���½��ܹؼ�֡������ܹؼ�֡�м��������ͼ��

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);//��localmapping�ĵ�ͼ�м�������½��ĵ�ͼ��
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;//ò���������û��
        }
    }
}

//��鲢�ں� ��ǰ�ؼ�֡ �� һ����������֡ �ظ��ĵ�ͼ�� 
//���������ǽ�һ����������֡��ͼ��ͶӰ����ǰ�ؼ�֡�У����뵱ǰ֡�����������ƥ�䣬���ƥ��������������ͼ��͵�ǰ֡����Ϣ
//����ǰ֡�ĵ�ͼ��ͶӰ��һ����������֡�У�����һ����������֡�����������ƥ�䣬���ƥ��������������ͼ�������֡����Ϣ
//�����Ϊ���Ǹ����˵�ǰ֡�ĵ�ͼ����Ϣ���������Ҫ���µ�ǰ֡�ڹ���ͼ�еĹ�ϵ
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
	
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);//��õ�ǰ֡�Ľ���֡=�뵱ǰ֡������Ȩ��ǰ10�Ĺؼ�֡
    vector<KeyFrame*> vpTargetKFs;//����洢�����뵱ǰ֡��һ������֡�Ͷ�������֡
    //��������֡��Ŀ���ǽ���ǰ֡��һ������֡�Ͷ�������֡������vpTargetKFs��
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);//��ý���֡Ȩ��ǰ��Ĺ��ӹؼ�֡=2������֡
        //����2������֡
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    //����ǰ֡�ĵ�ͼ����һ������֡�Ͷ�������֡�еĵ�ͼ������ں�
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();//��ǰ֡�ĵ�ͼ��
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        //����Ĳ�����һ������֡�Ͷ�������֡����ǰ֡�ĵ�ͼ��!!!!!!!!!!!!!!!!!!!!
        //��Ӧ���ǵ�һ������
        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    //��һ������֡�Ͷ�������֡�еĵ�ͼ���뵱ǰ֡�еĵ�ͼ������ں�
    vector<MapPoint*> vpFuseCandidates;//����֡�ĵ�ͼ��
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();//һ������֡�Ͷ�������֡�еĵ�ͼ��

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }
    //��Ӧ���ǵ�һ������
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);//����Ĳ����ǵ�ǰ֡��һ����������֡�еĵ�ͼ��!!!!!!!!!!!!!!!!!!!


    // Update points
    //���µ�ǰ֡��ͼ��������ӣ���ȣ��۲������������  
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();//���µ�ͼ���������(�����й۲��ڵ���������ѡ����õ�������)  
                pMP->UpdateNormalAndDepth();//����ƽ���۲ⷽ��͹۲����  
            }
        }
    }

    // Update connections in covisibility graph
    //���µ�ǰ֡�ĵ�ͼ��� ����������֡�����ӹ�ϵ �۲⵽����ĵ�ͼ��Ĵ�������Ϣ  
       mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

//�ͷ�localmap�߳��д���Ĺؼ�֡
void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)//���localmap�߳��Ѿ�ֹͣ���򷵻�
        return;
    //
    mbStopped = false;
    mbStopRequested = false;//��ʾ����ִ�з������Ż�
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

//Ӧ��������localmapping�̲߳�Ҫֹͣ������
//����true��ʾ��Ҫֹͣ�������趨�ɹ�
//
bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)//mbStopped=true��ʾlocalmapping�߳��Ѿ���ȫֹͣ��
        return false;

    mbNotStop = flag;//����������������mbNotStop��ֵ��

    return true;
}

//����ô����������������д����������????????????????????????
void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

//���ڵ�ǰ�ؼ�֡�Ĺ���֡����ɸѡ��ɾ��������Ĺؼ�֡
void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();//�õ���ǰ�ؼ�֡֡�Ĺ��ӹؼ�֡

    //�˺�������һ�����forѭ������
    //������ǰ�ؼ�֡�Ĺ��ӹؼ�֡
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();//����֡�п����ĵ�ͼ��

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;//������ǵ�ǰ֡���ж���������ĵ�ͼ��
        int nMPs=0;
	    //�������ӹؼ�֡�еĵ�ͼ��
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)//����˫Ŀ�����ǽ�����MapPoints��������mbf * 35 / fx  
                            continue;
                    }

                    nMPs++;//����֡�е�������Ч�ĵ�ͼ��
                    if(pMP->Observations()>thObs)//��ͼ�� MapPoints ���ٱ������ؼ�֡�۲⵽  
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;//������Ǹõ�ͼ�㱻����֡�ۿ�������Ч����
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;//���������ͼ�������֡
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)//�߶�Լ������ͼ���ڸùؼ�֡�������߶ȴ��ڵ��������ؼ�֡�������߶� 
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)// �õ�ͼ�����ٱ������ؼ�֡�۲⵽  
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs) //����������֡�������ͼ��������֡��Ч��ͼ��İٷ�֮90��ɾ���������֡
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                                                                 v.at<float>(2),               0,-v.at<float>(0),
                                                                 -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();//����²����Ĺؼ�֡
        mlpRecentAddedMapPoints.clear();//�������¼ӵĵ�ͼ��
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;//��������������ﱻ��ֵ
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
