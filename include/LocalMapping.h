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

    bool mbMonocular;//˫Ŀ�������false
    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;//��ס����Դ��mbResetRequested------------------------------------------------------------

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;//��ʾ����ر�localmapping����߳� ֻ��shutdown�����б�����
    bool mbFinished;//��ʾlocalmapping�̵߳�״̬���ǹر��˻���û�б��ر�
    std::mutex mMutexFinish;//��ס����Դ��mbFinishRequested��mbFinished----------------------------------------------

    Map* mpMap;//localmapping�еĵ�ͼ��ʵ����system�еĵ�ͼ ��tracking�߳��еĵ�ͼһ������ռ��ͬ���Ĵ洢�ռ�

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;//�����ɵĴ��ݸ�localmapping�Ĺؼ�֡

    KeyFrame* mpCurrentKeyFrame;//localmapping�̵߳�ǰ����Ĺؼ�֡

    std::list<MapPoint*> mlpRecentAddedMapPoints;//localmapping�¼���ĵ�ͼ��-
    bool mbAbortBA;//�Ƿ�����ִ��BA�Ż�
    std::mutex mMutexNewKFs;//��ס����Դ��mlNewKeyFrames��mbAbortBA-------------------------------------------------

  

    bool mbStopped;//����ָʾlocalmapping����߳��Ƿ��Ѿ�ֹͣ������
    bool mbStopRequested;//ֹͣlocalmapping�̵߳�����
    bool mbNotStop;//Ӧ����tracking�߳�Ҫ��localmapping�߳�ֹͣ������
    std::mutex mMutexStop;//��ס����Դ��mbStopRequested��mbStopped��mbNotStop--------------------------------------

    bool mbAcceptKeyFrames;//��ʾlocalmapping�߳��Ƿ��ܽ��չؼ�֡������ǰ��localmapping�߳��Ƿ����
    std::mutex mMutexAccept;//��ס����Դ��mbAcceptKeyFrames---------------------------------------------------------
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
