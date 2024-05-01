/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Ra煤l Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
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
            //对应文章中的7-A
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               //对应文章中的7-B
               if(ComputeSim3())//是否成功计算得到了相对位姿
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


//首先在当前关键帧的共视帧中通过特征匹配得到闭环候选帧vpCandidateKFs
//然后再对上面闭环候选帧做一致性检验，排除那些非一致性的候选帧并将最终的候选回环帧保存在mvpEnoughConsistentCandidates中
bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();//localmapping中传来的待处理的关键帧
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    //如果距离上一次闭环检测没有超过10帧
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();//如果需要删除当前关键帧则删除
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;//与当前关键帧相连关键帧的最小Bow得分
    //遍历当前关键帧的共视帧，获得当前帧与共视帧最小差异度的Bow=minScore
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];//共视关键帧
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);//获得当前帧的Bow词典和共视帧的Bow词典的差异度，得分越小差异度越小

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    //检测得到闭环候选帧!!!!!!!!!!非常重要的函数
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();//如果没有找到回环则清空子连续组，这就意味着子连续组中保存的是连续几个关键帧都找到了回环候选帧
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    //针对上面已经筛选过的关键帧vpCandidateKFs，再进行连续性检测，后面的程序全部都是一致性检测。
    //详见算法实现文档。
     mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    //遍历回环候选关键帧帧
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];//回环候选关键帧

        //spCandidateGroup存储的是候选回环关键帧+其共视帧=子候选组
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();//得到候选回环帧的共视帧
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
		
		//遍历之前的子连续组，即mvConsistentGroups
		//初始时刻子连续组是空的
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;//得到子连续组中的第一个连续成员

            bool bConsistent = false;
	     	//遍历子候选组关键帧，检测子候选组中的关键帧在 之前的子连续组 中是否存在  
	     	//如果子候选组中有一个关键帧共同存在于“ 子候选组 ”与之前的“ 子连续组 ”，那么“ 子候选组 ”与该“ 子连续组 ”连续  
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)//如果子候选组与之前的子连续组连续
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])//防止在vCurrentConsistentGroups插入相同的子候选组，这里是i而不是iG!!!-我觉得这里就应该是iG啊
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);//子候选组
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)//mnCovisibilityConsistencyTh=3
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);//存储满足连续性的回环检测候选帧
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        //如果该“ 子候选组 ” 的所有关键帧都不存在于“ 子连续组 ”，那么 vCurrentConsistentGroups 将为空，  
        // 于是就把“ 子候选组 ”全部添加到 vCurrentConsistentGroups，并最终用于更新 mvConsistentGroups，  
        // 计数器设为0，重新开始  
        //初始化时执行
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);//子候选组
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    //更新连续组mvConsistentGroups的内容
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    // 添加当前关键帧 到关键帧数据库      
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

    mpCurrentKF->SetErase();//感觉这个代码没用啊!!!!!!!!!!!根本执行不到这一部
    return false;
}

//确定闭环候选帧和当前帧的位姿变化[R,t]
//基本思路是先将当前帧和闭环候选帧进行地图点的匹配，然后使用匹配的地图点计算两个帧的相对位姿，然后通过得到的位姿再进行一次地图点匹配，然后使用这些匹配的地图点进行非线性优化
//只要是通过非线性优化得到的有效边大于20则我们认为找到了最优闭环候选帧，使用最优闭环帧在世界坐标系下的位姿和相对位姿来更新当前帧在世界坐标系下的位姿
//然后将最优闭环候选帧的共视帧看到的所有地图点投影到当前帧与当前帧的特征点进行匹配，匹配后的结果保存在 mvpCurrentMatchedPoints
//对应文章7-B
//失败的两种情况是:1，有效边数小与20；2，mvpCurrentMatchedPoints中匹配的点小于40个
bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();//回环检测候选帧
	

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    //使用直方图检测
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;//用于求不同候选回环帧与当前帧的sim3
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;//存储不同回环候选帧对应的地图点
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    //遍历闭环候选关键帧
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
        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);//匹配当前帧和候选闭环关键帧的特征点，对应的是第二个函数。

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            //对匹配成功的关键帧进行sim3求解，需要的点数较少.
            //此处使用的方法就是Closed-form solution of absolute orientation using unit quaternions
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);//设置ransac算法运行的参数
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;//与当前关键帧匹配比较强的候选关键帧帧-指两个关键帧匹配的地图点大于20
    }

    bool bMatch = false;//表示还没有找到匹配的候选关键帧

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        //遍历所有候选闭环帧
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])//只对匹配比较强的候选关键帧进行处理
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];//候选闭环帧

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
	     	//返回的是候选闭环帧到当前帧的相对位姿初值
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);//最多只进行5次ransac迭代求解,!!!!!!!!!!!!!!!!!!!!!!!!!重要函数

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                //vpMapPointMatches应该是获取ransac过程后关键帧1和关键帧2已经匹配的的inliner地图点
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];//
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
				//通过上面我们已经得到当前帧和候选帧的相对位姿，通过相对位姿得到更多的匹配
				//vpMapPointMatches既是输入也是输出,!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!重要函数
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);//将相对位姿转换到g2o格式
		 		//根据得到新的很多的匹配特征点则进行非线性优化sim3位姿。!!!!!!!!!!!!!!!!!!!!!!!!!!重要函数
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);//当前帧相对于闭环候选帧的相对位姿

                // If optimization is succesful stop ransacs and continue
                //我觉得这里可以进行优化??????????????作者这里停止计算的条件是只要非线性优化后inliner个数大于20就停止从其他候选帧中计算相对位姿
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;//找到了最优闭环候选帧
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);//闭环候选帧在世界坐标系下的位姿
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);//应该是当前帧在世界坐标系下的位姿

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)//没有找到闭环候选帧
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    //最优闭环候选关键帧的共视关键帧+最优闭环候选关键帧=vpLoopConnectedKFs
    //使用vpLoopConnectedKFs帧看到的地图点更新mvpLoopMapPoints
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
    //将最优候选闭环帧的共视帧看到的地图点与当前帧特征点进行匹配，此时会产生更多的匹配点。第一个函数
    //第一个参数是:当前帧；第二个参数是当前帧在世界坐标系下的位姿
    //第三个参数是最优闭环候选帧和其共视帧看到的地图点；第四个参数是最优闭环候选帧和当前帧匹配的inliner地图点
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)//计算成功其他线程不可以删除最优闭环候选帧，其余闭环候选帧全部可以删除
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else//计算失败则其他线程可以删除最优闭环候选帧和其余闭环候选帧
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}


//进入这个函数的条件是表示当前关键帧找到了一个回环帧与其对应，也就是发现了一个回环。
//找到了当前帧的最优闭环候选帧则需要首选更新当前帧与其他帧的关系。之前通过寻找回环已经局部优化了当前帧位姿，现在我们对当前帧共视帧和共视帧看到的地图点做位姿的调整
//则调整之后的共视帧在世界坐标下的位姿=优化后的当前帧的位姿*优化之前当前帧相对于共视帧的相对位姿，调整之后的地图点在世界坐标系下的位置=原来地图点的位姿*共视帧调整的位姿
//然后使用与当前帧特征点已经匹配的 最优闭环帧的共视帧所看到的地图点更新当前帧的地图点，更新原则是如果当前帧这个特征点有闭环匹配的地图点，则使用闭环匹配的地图点代替当前帧的地图点；如果当前帧这个特征点没有闭环匹配的地图点，则向当前帧中加入这个地图点，并将闭环匹配的地图点加入到前帧中
//使用最优闭环帧的共视帧看到的地图点更新当前帧和共视帧看到的地图点
//然后得到由闭环形成的连接关系，执行essential优化
//最后开启一个全局优化
void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    //暂停localmapping工作，防止插入新的关键帧
    //loopclosing和localmapping 交互
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    //正如orb文章中所说的如果检测到了回环，则需要停止全局优化线程，但是在处理完闭环工作后，再开启一个全局优化线程
    if(isRunningGBA())//如果全局优化正在执行，则将它停止
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
        
            mpThreadGBA->detach();//分离线程，不能被其他线程回收或杀死的，它的存储器资源在它终止时由系统自动释放。
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    //是否localmapping这个线程是否停止，如果停止再往下运行程序
    //loopclosing和localmapping 交互
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);//停止1毫秒
    }

    // Ensure current keyframe is updated
    //更新当前关键帧在共视图中的关系
    mpCurrentKF->UpdateConnections();//当前关键帧找到了对应的一个回环帧，而我们之前会对这个当前关键帧更新它的地图点和它的位姿，因此需要更新共视图

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
     //之前通过寻找回环已经局部优化了当前帧位姿，现在我们对当前帧共视帧的位姿和共视帧看到的地图点位姿做调整
    //CorrectedSim3存储的是调整过之后的共视帧和当前帧在世界坐标系下的位姿
    //NonCorrectedSim3是调整之前共视帧和当前帧在世界坐标系下的位姿
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;//当期帧经过回环优化过的位姿
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();//当前帧在世界坐标下没有经过回环优化的位姿的逆矩阵=camera->世界坐标系的变化


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        //遍历当前帧的共视帧和当前帧并计算需要变化的位姿，
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;//得到 当前帧/当前帧的共视帧

            cv::Mat Tiw = pKFi->GetPose();//没有回环修正的 当前帧/当前帧的共视帧 的世界坐标系下的位姿

            if(pKFi!=mpCurrentKF)//如果这个帧是共视帧 不是当前帧
            {
                cv::Mat Tic = Tiw*Twc;//没有回环修正的当前帧和共视帧的相对位姿变化
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;//得到调整之后的共视帧在世界坐标下的位姿=优化后的当前帧的位姿*优化之前当前帧相对于共视帧的相对位姿
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
        //对 当前关键帧和其共视帧 看到的地图点位置和 当前帧与当前帧的共视帧位姿进行调整，并更新共视帧与其他帧的连接关系
        //遍历当前关键帧的共视帧和当前帧
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;//当前帧/共视帧
            g2o::Sim3 g2oCorrectedSiw = mit->second;//当前帧/共视帧调整后的的位姿
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];//未调整之前的位姿

	     //对共视帧中的地图点也进行相应的位姿变化
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();//当前帧/共视帧的地图点
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)//如果这个地图点被当前帧观测到了，则不用共视帧更新这个地图点的调整后的位置了
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();//未调整的地图点在世界坐标系下的位置
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));//调整之后的地图点在世界坐标系下的位置=原来地图点的在原来此帧的相机坐标系下的位姿*当前帧/共视帧调整的位姿

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

            pKFi->SetPose(correctedTiw);//设置当前帧/共视帧调整后的新位姿
            // Make sure connections are updated
            //更新关键帧连接关系
            pKFi->UpdateConnections();
        }




       //上面全部都是关键帧和地图点位姿调整的内容
      //下面是回环融合的内容
      


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        //我们这里称 与当前帧特征点已经匹配的 最优闭环帧的共视帧所看到的地图点 为闭环匹配地图点
        //使用闭环匹配地图点 更新当前帧的地图点
        //更新原则是;如果当前帧这个特征点有闭环匹配的地图点，则使用闭环匹配的地图点代替当前帧的地图点
        //如果当前帧这个特征点没有闭环匹配的地图点，则向当前帧中加入这个地图点，并将闭环匹配的地图点加入到前帧中
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);//使用闭环匹配的地图点代替当前帧的地图点
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
    //用localmappoint更新当前帧的mappoint
    //CorrectedSim3存储的是调整过之后的共视帧和当前帧在世界坐标系下的位姿
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    //LoopConnections存储的是回环帧=(当前帧共视帧，闭环融合之后当前帧的2级共视帧-之前2级共视帧-之前1级共视帧)+(当前帧，闭环融合之后1级共视帧-之前1级共视帧-之前1级共视帧)
    //由闭环形成的连接关系
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;
    //遍历回环融合之前当前帧的共视帧!!!!!!!!!!!!!!!!!!!!!
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();//当前帧的优秀共视帧/当前帧的2级共视帧(回环融合之前的)

        // Update connections. Detect new links.
        //更新 当前帧/当前帧的共视帧的 连接关系
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();//当前帧的优秀共视帧/当前帧的2级共视帧(回环融合之后的)
	    //LoopConnections中删除 回环融合之前的 当前帧的优秀共视帧/二级共视帧
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
       //LoopConnections中删除回环融合之前的 当前帧的优秀共视帧/
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    //mpMatchedKF =与当前关键帧相匹配的最优闭环候选帧
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();//表示地图因为回环发生了一次巨大的变化

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    //新建一个线程进行全局的优化!!!!!!!!!!!!!!!!,这也算是除了tracking localmapping 回环检测和可视化线程的第五个线程
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    //正如orb文章所说的当检测到一个闭环后，需要先停止全局优化线程，先执行闭环工作，然后再开开启全局优化线程
    //全局优化的函数需要一个输入值，是当前关键帧的序号
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    //loopclosing和localmapping 交互
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

//输入的参数是当前帧和其共视帧调整后的世界坐标系下的位姿
//这个函数的作用是将最优闭环候选关键帧的共视帧+最优闭环候选关键帧的地图点 投影到 调整后位姿的当前帧的共视帧中并与其进行地图点的融合。
//就是用 最优闭环候选关键帧的共视帧+最优闭环候选关键帧的地图点 代替 调整后位姿的当前帧的共视帧的地图点
//我觉得这里作者做的不是很好???????????????????????????????
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    //遍历当前帧和共视帧
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;//共视帧

        g2o::Sim3 g2oScw = mit->second;//调整后的位姿
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));//这个参数存储的是调整帧的地图点
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);//!!!!!!!!!!!!!!!!主要函数

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];//调整帧的地图点
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);//使用mvpLoopMapPoints中的地图点(没有经过闭环修正过的) 代替  调整帧的地图点
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

//输入的是当前帧的序号
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
			//loopclosing和localmapping 交互
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
			//loopclosing和localmapping 交互
            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());
            //根据全局优化更新关键帧的位姿??????????????????????????????????
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
            //更新新地图点的位姿
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
            //loopclosing和localmapping 交互
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
