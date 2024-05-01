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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId;//用于记录地图点的序号
    static long unsigned int nNextId;//用于记录一共产生了多少个地图点
    long int mnFirstKFid;//看到该地图点的第一个关键帧的序号
    long int mnFirstFrame;
    int nObs;//这个地图点被几个关键帧观测到过

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;//这个参数应该是还是否继续追踪这个地图点
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;//这个参数应该是当前的这个地图点用于求某个帧的位姿，这个参数存储的就是某个帧的序号
    long unsigned int mnLastFrameSeen;//刚刚看到这个地图点的帧的序号

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;//这个地图点是用于局部优化，这个参数是局部优化时当前帧的序号
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    //这个参数用于闭环检测中，当我们检测到闭环候我们就需要对当前帧和其共视帧中的地图点进行调整
    //这个参数指明当前这个地图点是因为哪一个当前帧而做出的调整
    long unsigned int mnCorrectedByKF;
    //这个参数指明当前这个需要被调整的地图点是在当前帧中的哪个共视帧中被看到然后因为回环检测而进行调整的。
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;//通过全局优化得到的地图点的位置
    //我们都知道只有检测到了回环后，我们才能进行全局优化，全局优化后会得到优化后的关键帧的位姿。那么我想知道优化后的关键帧位姿是在哪一个关键帧中监测到了回环然后得到的。
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     //算法实现文档中对此变量进行了说明
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction=特征点的法向量平均值。特征点的法向量=特征点在世界坐标系下的坐标-观测到这个特征点的相机光心在世界坐标系下的坐标
     //这个变量好像从来没有被使用过
	 cv::Mat mNormalVector;

     // Best descriptor to fast matching
     //地图点的描述子，由ComputeDistinctiveDescriptors函数得到
     cv::Mat mDescriptor;

     // Reference KeyFrame=与当前帧有最多相同特征点的帧，这个参数有点不明白??????????????
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;//前期匹配得到的普通帧个数 初始值是1
     int mnFound;//非线性优化有效使用这个地图点的普通帧个数 初始值是1

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;//这个参数如果不为空，则当前的这个地图点需要被mpReplaced这个地图点替换

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
