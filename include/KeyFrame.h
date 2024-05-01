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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }
    //升序列
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;//关键帧的序号
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;//一个grid在列上占用多少个像素
    const float mfGridElementHeightInv;//一个grid在行上占用多少个像素

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;//应该是:当前关键帧被用于求那个帧的位姿，这个参数存储的就是那个帧的序号
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;//这个关键帧用于局部优化(是被优化参数)，这个参数是局部优化时当前帧的序号
    long unsigned int mnBAFixedForKF;//这个关键帧用于局部优化(是固定的参数)，这个参数是局部优化时当前帧的序号

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;//当前闭环检测帧的序号
    int mnLoopWords;//当前关键帧与闭环检测帧相同的单词数
    float mLoopScore;//当前关键帧与闭环检测帧的单词区分度
    long unsigned int mnRelocQuery;
    int mnRelocWords;//这个关键帧与当前帧的共同的单词个数
    float mRelocScore;//这个关键帧与当前帧的bow相似分数

    // Variables used by loop closing
    cv::Mat mTcwGBA;//通过全局优化得到的关键帧的位姿
    cv::Mat mTcwBefGBA;
     //我们都知道只有检测到了回环后，我们才能进行全局优化，全局优化后会得到优化后的关键帧的位姿，那么我想知道优化后的关键帧位姿是在哪一个关键帧中监测到了回环然后得到的。
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points 与左图像特征点最优匹配的右图像的特征点u轴坐标
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;//默认值是1.2
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;////每个金字塔的缩放比例,比如说图形要缩小2倍一共4层，则保存的就是[1,2,4,8]
    const std::vector<float> mvLevelSigma2;//是mvScaleFactors的平方
    const std::vector<float> mvInvLevelSigma2;//是mvLevelSigma2的倒数

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;//摄像机矩阵


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    //与当前关键帧已经匹配的地图点-我感觉应该是tracking得到的
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    //详见GetFeaturesInArea函数
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    //mvpOrderedConnectedKeyFrames+mvOrderedWeights=排序后的mConnectedKeyFrameWeights
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;//存储的是与当前关键帧相关的优秀共视帧，并记录其共视个数
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;//存储的是与当前关键帧相关的优秀共视帧(共视个数超过15个)
    std::vector<int> mvOrderedWeights;//存储的是优秀共视帧的共视个数，对应的是mvpOrderedConnectedKeyFrames中的共视个数

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;//在最小生成树中当前关键帧的父关键帧
    std::set<KeyFrame*> mspChildrens;//mspChildrens里面的关键帧有很多的共视帧，而只有当前关键帧与mspChildrens里的关键帧共视个数最多
    //如果当前关键帧是当前帧，则这个变量就是对应最优的闭环候选帧
    //如果当前关键帧是最优的闭环候选帧，则这个变量就是对应的当前帧
    std::set<KeyFrame*> mspLoopEdges;
    // Bad flags
    bool mbNotErase;//表示当前关键帧不可以被删除因为它正在被处理中
    bool mbToBeErased;//如果为true表示当前帧需要被删除了
    bool mbBad; //应该是表示当前关键帧是否有效   

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
