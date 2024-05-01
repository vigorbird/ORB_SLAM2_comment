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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            //��Ӧ�����е�7-A
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               //��Ӧ�����е�7-B
               if(ComputeSim3())//�Ƿ�ɹ�����õ������λ��
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}


//�����ڵ�ǰ�ؼ�֡�Ĺ���֡��ͨ������ƥ��õ��ջ���ѡ֡vpCandidateKFs
//Ȼ���ٶ�����ջ���ѡ֡��һ���Լ��飬�ų���Щ��һ���Եĺ�ѡ֡�������յĺ�ѡ�ػ�֡������mvpEnoughConsistentCandidates��
bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();//localmapping�д����Ĵ�����Ĺؼ�֡
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    //���������һ�αջ����û�г���10֡
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();//�����Ҫɾ����ǰ�ؼ�֡��ɾ��
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;//�뵱ǰ�ؼ�֡�����ؼ�֡����СBow�÷�
    //������ǰ�ؼ�֡�Ĺ���֡����õ�ǰ֡�빲��֡��С����ȵ�Bow=minScore
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];//���ӹؼ�֡
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);//��õ�ǰ֡��Bow�ʵ�͹���֡��Bow�ʵ�Ĳ���ȣ��÷�ԽС�����ԽС

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    //���õ��ջ���ѡ֡!!!!!!!!!!�ǳ���Ҫ�ĺ���
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();//���û���ҵ��ػ�������������飬�����ζ�����������б���������������ؼ�֡���ҵ��˻ػ���ѡ֡
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    //��������Ѿ�ɸѡ���Ĺؼ�֡vpCandidateKFs���ٽ��������Լ�⣬����ĳ���ȫ������һ���Լ�⡣
    //����㷨ʵ���ĵ���
     mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    //�����ػ���ѡ�ؼ�֡֡
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];//�ػ���ѡ�ؼ�֡

        //spCandidateGroup�洢���Ǻ�ѡ�ػ��ؼ�֡+�乲��֡=�Ӻ�ѡ��
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();//�õ���ѡ�ػ�֡�Ĺ���֡
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
		
		//����֮ǰ���������飬��mvConsistentGroups
		//��ʼʱ�����������ǿյ�
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;//�õ����������еĵ�һ��������Ա

            bool bConsistent = false;
	     	//�����Ӻ�ѡ��ؼ�֡������Ӻ�ѡ���еĹؼ�֡�� ֮ǰ���������� ���Ƿ����  
	     	//����Ӻ�ѡ������һ���ؼ�֡��ͬ�����ڡ� �Ӻ�ѡ�� ����֮ǰ�ġ� �������� ������ô�� �Ӻ�ѡ�� ����á� �������� ������  
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)//����Ӻ�ѡ����֮ǰ��������������
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])//��ֹ��vCurrentConsistentGroups������ͬ���Ӻ�ѡ�飬������i������iG!!!-�Ҿ��������Ӧ����iG��
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);//�Ӻ�ѡ��
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)//mnCovisibilityConsistencyTh=3
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);//�洢���������ԵĻػ�����ѡ֡
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        //����á� �Ӻ�ѡ�� �� �����йؼ�֡���������ڡ� �������� ������ô vCurrentConsistentGroups ��Ϊ�գ�  
        // ���ǾͰѡ� �Ӻ�ѡ�� ��ȫ����ӵ� vCurrentConsistentGroups�����������ڸ��� mvConsistentGroups��  
        // ��������Ϊ0�����¿�ʼ  
        //��ʼ��ʱִ��
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);//�Ӻ�ѡ��
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    //����������mvConsistentGroups������
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    // ��ӵ�ǰ�ؼ�֡ ���ؼ�֡���ݿ�      
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();//�о��������û�ð�!!!!!!!!!!!����ִ�в�����һ��
    return false;
}

//ȷ���ջ���ѡ֡�͵�ǰ֡��λ�˱仯[R,t]
//����˼·���Ƚ���ǰ֡�ͱջ���ѡ֡���е�ͼ���ƥ�䣬Ȼ��ʹ��ƥ��ĵ�ͼ���������֡�����λ�ˣ�Ȼ��ͨ���õ���λ���ٽ���һ�ε�ͼ��ƥ�䣬Ȼ��ʹ����Щƥ��ĵ�ͼ����з������Ż�
//ֻҪ��ͨ���������Ż��õ�����Ч�ߴ���20��������Ϊ�ҵ������űջ���ѡ֡��ʹ�����űջ�֡����������ϵ�µ�λ�˺����λ�������µ�ǰ֡����������ϵ�µ�λ��
//Ȼ�����űջ���ѡ֡�Ĺ���֡���������е�ͼ��ͶӰ����ǰ֡�뵱ǰ֡�����������ƥ�䣬ƥ���Ľ�������� mvpCurrentMatchedPoints
//��Ӧ����7-B
//ʧ�ܵ����������:1����Ч����С��20��2��mvpCurrentMatchedPoints��ƥ��ĵ�С��40��
bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();//�ػ�����ѡ֡
	

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    //ʹ��ֱ��ͼ���
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;//������ͬ��ѡ�ػ�֡�뵱ǰ֡��sim3
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;//�洢��ͬ�ػ���ѡ֡��Ӧ�ĵ�ͼ��
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    //�����ջ���ѡ�ؼ�֡
    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }
        //
        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);//ƥ�䵱ǰ֡�ͺ�ѡ�ջ��ؼ�֡�������㣬��Ӧ���ǵڶ���������

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            //��ƥ��ɹ��Ĺؼ�֡����sim3��⣬��Ҫ�ĵ�������.
            //�˴�ʹ�õķ�������Closed-form solution of absolute orientation using unit quaternions
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);//����ransac�㷨���еĲ���
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;//�뵱ǰ�ؼ�֡ƥ��Ƚ�ǿ�ĺ�ѡ�ؼ�֡֡-ָ�����ؼ�֡ƥ��ĵ�ͼ�����20
    }

    bool bMatch = false;//��ʾ��û���ҵ�ƥ��ĺ�ѡ�ؼ�֡

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        //�������к�ѡ�ջ�֡
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])//ֻ��ƥ��Ƚ�ǿ�ĺ�ѡ�ؼ�֡���д���
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];//��ѡ�ջ�֡

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
	     	//���ص��Ǻ�ѡ�ջ�֡����ǰ֡�����λ�˳�ֵ
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);//���ֻ����5��ransac�������,!!!!!!!!!!!!!!!!!!!!!!!!!��Ҫ����

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                //vpMapPointMatchesӦ���ǻ�ȡransac���̺�ؼ�֡1�͹ؼ�֡2�Ѿ�ƥ��ĵ�inliner��ͼ��
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];//
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
				//ͨ�����������Ѿ��õ���ǰ֡�ͺ�ѡ֡�����λ�ˣ�ͨ�����λ�˵õ������ƥ��
				//vpMapPointMatches��������Ҳ�����,!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!��Ҫ����
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);//�����λ��ת����g2o��ʽ
		 		//���ݵõ��µĺܶ��ƥ������������з������Ż�sim3λ�ˡ�!!!!!!!!!!!!!!!!!!!!!!!!!!��Ҫ����
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);//��ǰ֡����ڱջ���ѡ֡�����λ��

                // If optimization is succesful stop ransacs and continue
                //�Ҿ���������Խ����Ż�??????????????��������ֹͣ�����������ֻҪ�������Ż���inliner��������20��ֹͣ��������ѡ֡�м������λ��
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;//�ҵ������űջ���ѡ֡
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);//�ջ���ѡ֡����������ϵ�µ�λ��
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);//Ӧ���ǵ�ǰ֡����������ϵ�µ�λ��

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)//û���ҵ��ջ���ѡ֡
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    //���űջ���ѡ�ؼ�֡�Ĺ��ӹؼ�֡+���űջ���ѡ�ؼ�֡=vpLoopConnectedKFs
    //ʹ��vpLoopConnectedKFs֡�����ĵ�ͼ�����mvpLoopMapPoints
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    //�����ź�ѡ�ջ�֡�Ĺ���֡�����ĵ�ͼ���뵱ǰ֡���������ƥ�䣬��ʱ����������ƥ��㡣��һ������
    //��һ��������:��ǰ֡���ڶ��������ǵ�ǰ֡����������ϵ�µ�λ��
    //���������������űջ���ѡ֡���乲��֡�����ĵ�ͼ�㣻���ĸ����������űջ���ѡ֡�͵�ǰ֡ƥ���inliner��ͼ��
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)//����ɹ������̲߳�����ɾ�����űջ���ѡ֡������ջ���ѡ֡ȫ������ɾ��
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else//����ʧ���������߳̿���ɾ�����űջ���ѡ֡������ջ���ѡ֡
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}


//������������������Ǳ�ʾ��ǰ�ؼ�֡�ҵ���һ���ػ�֡�����Ӧ��Ҳ���Ƿ�����һ���ػ���
//�ҵ��˵�ǰ֡�����űջ���ѡ֡����Ҫ��ѡ���µ�ǰ֡������֡�Ĺ�ϵ��֮ǰͨ��Ѱ�һػ��Ѿ��ֲ��Ż��˵�ǰ֡λ�ˣ��������ǶԵ�ǰ֡����֡�͹���֡�����ĵ�ͼ����λ�˵ĵ���
//�����֮��Ĺ���֡�����������µ�λ��=�Ż���ĵ�ǰ֡��λ��*�Ż�֮ǰ��ǰ֡����ڹ���֡�����λ�ˣ�����֮��ĵ�ͼ������������ϵ�µ�λ��=ԭ����ͼ���λ��*����֡������λ��
//Ȼ��ʹ���뵱ǰ֡�������Ѿ�ƥ��� ���űջ�֡�Ĺ���֡�������ĵ�ͼ����µ�ǰ֡�ĵ�ͼ�㣬����ԭ���������ǰ֡����������бջ�ƥ��ĵ�ͼ�㣬��ʹ�ñջ�ƥ��ĵ�ͼ����浱ǰ֡�ĵ�ͼ�㣻�����ǰ֡���������û�бջ�ƥ��ĵ�ͼ�㣬����ǰ֡�м��������ͼ�㣬�����ջ�ƥ��ĵ�ͼ����뵽ǰ֡��
//ʹ�����űջ�֡�Ĺ���֡�����ĵ�ͼ����µ�ǰ֡�͹���֡�����ĵ�ͼ��
//Ȼ��õ��ɱջ��γɵ����ӹ�ϵ��ִ��essential�Ż�
//�����һ��ȫ���Ż�
void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    //��ͣlocalmapping��������ֹ�����µĹؼ�֡
    //loopclosing��localmapping ����
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    //����orb��������˵�������⵽�˻ػ�������Ҫֹͣȫ���Ż��̣߳������ڴ�����ջ��������ٿ���һ��ȫ���Ż��߳�
    if(isRunningGBA())//���ȫ���Ż�����ִ�У�����ֹͣ
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
        
            mpThreadGBA->detach();//�����̣߳����ܱ������̻߳��ջ�ɱ���ģ����Ĵ洢����Դ������ֹʱ��ϵͳ�Զ��ͷš�
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    //�Ƿ�localmapping����߳��Ƿ�ֹͣ�����ֹͣ���������г���
    //loopclosing��localmapping ����
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);//ֹͣ1����
    }

    // Ensure current keyframe is updated
    //���µ�ǰ�ؼ�֡�ڹ���ͼ�еĹ�ϵ
    mpCurrentKF->UpdateConnections();//��ǰ�ؼ�֡�ҵ��˶�Ӧ��һ���ػ�֡��������֮ǰ��������ǰ�ؼ�֡�������ĵ�ͼ�������λ�ˣ������Ҫ���¹���ͼ

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
     //֮ǰͨ��Ѱ�һػ��Ѿ��ֲ��Ż��˵�ǰ֡λ�ˣ��������ǶԵ�ǰ֡����֡��λ�˺͹���֡�����ĵ�ͼ��λ��������
    //CorrectedSim3�洢���ǵ�����֮��Ĺ���֡�͵�ǰ֡����������ϵ�µ�λ��
    //NonCorrectedSim3�ǵ���֮ǰ����֡�͵�ǰ֡����������ϵ�µ�λ��
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;//����֡�����ػ��Ż�����λ��
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();//��ǰ֡������������û�о����ػ��Ż���λ�˵������=camera->��������ϵ�ı仯


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        //������ǰ֡�Ĺ���֡�͵�ǰ֡��������Ҫ�仯��λ�ˣ�
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;//�õ� ��ǰ֡/��ǰ֡�Ĺ���֡

            cv::Mat Tiw = pKFi->GetPose();//û�лػ������� ��ǰ֡/��ǰ֡�Ĺ���֡ ����������ϵ�µ�λ��

            if(pKFi!=mpCurrentKF)//������֡�ǹ���֡ ���ǵ�ǰ֡
            {
                cv::Mat Tic = Tiw*Twc;//û�лػ������ĵ�ǰ֡�͹���֡�����λ�˱仯
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;//�õ�����֮��Ĺ���֡�����������µ�λ��=�Ż���ĵ�ǰ֡��λ��*�Ż�֮ǰ��ǰ֡����ڹ���֡�����λ��
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        //�� ��ǰ�ؼ�֡���乲��֡ �����ĵ�ͼ��λ�ú� ��ǰ֡�뵱ǰ֡�Ĺ���֡λ�˽��е����������¹���֡������֡�����ӹ�ϵ
        //������ǰ�ؼ�֡�Ĺ���֡�͵�ǰ֡
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;//��ǰ֡/����֡
            g2o::Sim3 g2oCorrectedSiw = mit->second;//��ǰ֡/����֡������ĵ�λ��
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];//δ����֮ǰ��λ��

	     //�Թ���֡�еĵ�ͼ��Ҳ������Ӧ��λ�˱仯
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();//��ǰ֡/����֡�ĵ�ͼ��
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)//��������ͼ�㱻��ǰ֡�۲⵽�ˣ����ù���֡���������ͼ��ĵ������λ����
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();//δ�����ĵ�ͼ������������ϵ�µ�λ��
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));//����֮��ĵ�ͼ������������ϵ�µ�λ��=ԭ����ͼ�����ԭ����֡���������ϵ�µ�λ��*��ǰ֡/����֡������λ��

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);//���õ�ǰ֡/����֡���������λ��
            // Make sure connections are updated
            //���¹ؼ�֡���ӹ�ϵ
            pKFi->UpdateConnections();
        }




       //����ȫ�����ǹؼ�֡�͵�ͼ��λ�˵���������
      //�����ǻػ��ںϵ�����
      


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        //��������� �뵱ǰ֡�������Ѿ�ƥ��� ���űջ�֡�Ĺ���֡�������ĵ�ͼ�� Ϊ�ջ�ƥ���ͼ��
        //ʹ�ñջ�ƥ���ͼ�� ���µ�ǰ֡�ĵ�ͼ��
        //����ԭ����;�����ǰ֡����������бջ�ƥ��ĵ�ͼ�㣬��ʹ�ñջ�ƥ��ĵ�ͼ����浱ǰ֡�ĵ�ͼ��
        //�����ǰ֡���������û�бջ�ƥ��ĵ�ͼ�㣬����ǰ֡�м��������ͼ�㣬�����ջ�ƥ��ĵ�ͼ����뵽ǰ֡��
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);//ʹ�ñջ�ƥ��ĵ�ͼ����浱ǰ֡�ĵ�ͼ��
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    //��localmappoint���µ�ǰ֡��mappoint
    //CorrectedSim3�洢���ǵ�����֮��Ĺ���֡�͵�ǰ֡����������ϵ�µ�λ��
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    //LoopConnections�洢���ǻػ�֡=(��ǰ֡����֡���ջ��ں�֮��ǰ֡��2������֡-֮ǰ2������֡-֮ǰ1������֡)+(��ǰ֡���ջ��ں�֮��1������֡-֮ǰ1������֡-֮ǰ1������֡)
    //�ɱջ��γɵ����ӹ�ϵ
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;
    //�����ػ��ں�֮ǰ��ǰ֡�Ĺ���֡!!!!!!!!!!!!!!!!!!!!!
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();//��ǰ֡�����㹲��֡/��ǰ֡��2������֡(�ػ��ں�֮ǰ��)

        // Update connections. Detect new links.
        //���� ��ǰ֡/��ǰ֡�Ĺ���֡�� ���ӹ�ϵ
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();//��ǰ֡�����㹲��֡/��ǰ֡��2������֡(�ػ��ں�֮���)
	    //LoopConnections��ɾ�� �ػ��ں�֮ǰ�� ��ǰ֡�����㹲��֡/��������֡
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
       //LoopConnections��ɾ���ػ��ں�֮ǰ�� ��ǰ֡�����㹲��֡/
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    //mpMatchedKF =�뵱ǰ�ؼ�֡��ƥ������űջ���ѡ֡
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();//��ʾ��ͼ��Ϊ�ػ�������һ�ξ޴�ı仯

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    //�½�һ���߳̽���ȫ�ֵ��Ż�!!!!!!!!!!!!!!!!,��Ҳ���ǳ���tracking localmapping �ػ����Ϳ��ӻ��̵߳ĵ�����߳�
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    //����orb������˵�ĵ���⵽һ���ջ�����Ҫ��ֹͣȫ���Ż��̣߳���ִ�бջ�������Ȼ���ٿ�����ȫ���Ż��߳�
    //ȫ���Ż��ĺ�����Ҫһ������ֵ���ǵ�ǰ�ؼ�֡�����
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    //loopclosing��localmapping ����
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

//����Ĳ����ǵ�ǰ֡���乲��֡���������������ϵ�µ�λ��
//��������������ǽ����űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�ĵ�ͼ�� ͶӰ�� ������λ�˵ĵ�ǰ֡�Ĺ���֡�в�������е�ͼ����ںϡ�
//������ ���űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�ĵ�ͼ�� ���� ������λ�˵ĵ�ǰ֡�Ĺ���֡�ĵ�ͼ��
//�Ҿ��������������Ĳ��Ǻܺ�???????????????????????????????
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    //������ǰ֡�͹���֡
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;//����֡

        g2o::Sim3 g2oScw = mit->second;//�������λ��
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));//��������洢���ǵ���֡�ĵ�ͼ��
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);//!!!!!!!!!!!!!!!!��Ҫ����

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];//����֡�ĵ�ͼ��
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);//ʹ��mvpLoopMapPoints�еĵ�ͼ��(û�о����ջ���������) ����  ����֡�ĵ�ͼ��
            }
        }
    }
}


void LoopClosing::RequestReset()
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
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

//������ǵ�ǰ֡�����
void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
			//loopclosing��localmapping ����
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
			//loopclosing��localmapping ����
            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());
            //����ȫ���Ż����¹ؼ�֡��λ��??????????????????????????????????
            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            //�����µ�ͼ���λ��
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();
            //loopclosing��localmapping ����
            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
