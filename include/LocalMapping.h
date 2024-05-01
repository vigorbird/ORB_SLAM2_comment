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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();//mbStopRequested = true;mbAbortBA = true;
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();//return mbStopped;
    bool stopRequested();//return mbStopRequested;
    bool AcceptKeyFrames();//return mbAcceptKeyFrames;
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();// mbAbortBA = true;

    void RequestFinish();//mbFinishRequested = true;
    bool isFinished();// return mbFinished;

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;//双目情况下是false
    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;//锁住的资源是mbResetRequested------------------------------------------------------------

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;//表示请求关闭localmapping这个线程 只在shutdown函数中被调用
    bool mbFinished;//表示localmapping线程的状态，是关闭了还是没有被关闭
    std::mutex mMutexFinish;//锁住的资源是mbFinishRequested和mbFinished----------------------------------------------

    Map* mpMap;//localmapping中的地图其实就是system中的地图 与tracking线程中的地图一样，都占用同样的存储空间

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;//新生成的传递给localmapping的关键帧

    KeyFrame* mpCurrentKeyFrame;//localmapping线程当前处理的关键帧

    std::list<MapPoint*> mlpRecentAddedMapPoints;//localmapping新加入的地图点-
    bool mbAbortBA;//是否允许执行BA优化
    std::mutex mMutexNewKFs;//锁住的资源是mlNewKeyFrames和mbAbortBA-------------------------------------------------

  

    bool mbStopped;//用于指示localmapping这个线程是否已经停止运行了
    bool mbStopRequested;//停止localmapping线程的请求，
    bool mbNotStop;//应该是tracking线程要求localmapping线程停止的请求
    std::mutex mMutexStop;//锁住的资源是mbStopRequested，mbStopped，mbNotStop--------------------------------------

    bool mbAcceptKeyFrames;//表示localmapping线程是否能接收关键帧，即当前的localmapping线程是否空闲
    std::mutex mMutexAccept;//锁住的资源是mbAcceptKeyFrames---------------------------------------------------------
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
