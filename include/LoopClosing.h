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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    //KeyFrameAndPose数据结构其实就是关键帧+Sim3组成的map，后面的东西表示内存对其
    //第二个元素sim3表示这个关键帧的位姿
    typedef map<  KeyFrame*,g2o::Sim3,std::less<KeyFrame*>, Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3>>  > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
	long unsigned int mLastLoopKFid;
	std::list<KeyFrame*> mlpLoopKeyFrameQueue;//这个是经过localmapping线程提炼出的关键帧
    std::mutex mMutexReset;//锁住的资源是mbResetRequested，mLastLoopKFid和mlpLoopKeyFrameQueue------------------------------------

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;//锁住的资源是mbFinishRequested和mbFinished-------------------------------------------------------

    Map* mpMap;//回环检测中的地图与tracking线程的地图和localmapping 中的地图一样，占用同样的数据空间
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;//我感觉这里存储的应该是从localmap线程中传来的关键帧
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    
    std::mutex mMutexLoopQueue;//锁住的资源是mlpLoopKeyFrameQueue

    // Loop detector parameters
    //默认值是3
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;//当前关键帧，用于匹配其他闭环候选帧
    KeyFrame* mpMatchedKF;//存储的是与当前关键帧相匹配的最优闭环候选帧
    std::vector<ConsistentGroup> mvConsistentGroups;//详见算法实现文档
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;//存储满足连续性的回环检测候选帧
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;//存储的是当前关键帧的共视帧+当前关键帧
    std::vector<MapPoint*> mvpCurrentMatchedPoints;//序号是当前关键帧特征点的序号，内容是最优闭环候选帧的共视帧所看到的地图点。
    std::vector<MapPoint*> mvpLoopMapPoints;//最优闭环候选关键帧的共视帧+最优闭环候选关键帧看到的地图点，没有经过回环融合调整过位姿
   
    cv::Mat mScw;//当前帧在世界坐标系下的位姿 opencv数据格式
    g2o::Sim3 mg2oScw;//当前帧在世界坐标系下的位姿，g2o数据格式

    

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;//全局优化是否在执行
    bool mbFinishedGBA;//表示全局优化已经结束了
    bool mbStopGBA;//是否停止全局优化
    std::mutex mMutexGBA;//锁住的资源是mbStopGBA---------------------------------------------------------------------------------
    std::thread* mpThreadGBA;//这个是loopclosing线程中用于开启全局优化的线程

    // Fix scale in the stereo/RGB-D case
    //在双目情况下是true，表示是否尺度是确定的
    bool mbFixScale;


    bool mnFullBAIdx;//表示检测到了几次回环
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
