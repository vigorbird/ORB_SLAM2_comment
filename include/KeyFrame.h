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
    //������
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;//�ؼ�֡�����
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;//һ��grid������ռ�ö��ٸ�����
    const float mfGridElementHeightInv;//һ��grid������ռ�ö��ٸ�����

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;//Ӧ����:��ǰ�ؼ�֡���������Ǹ�֡��λ�ˣ���������洢�ľ����Ǹ�֡�����
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;//����ؼ�֡���ھֲ��Ż�(�Ǳ��Ż�����)����������Ǿֲ��Ż�ʱ��ǰ֡�����
    long unsigned int mnBAFixedForKF;//����ؼ�֡���ھֲ��Ż�(�ǹ̶��Ĳ���)����������Ǿֲ��Ż�ʱ��ǰ֡�����

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;//��ǰ�ջ����֡�����
    int mnLoopWords;//��ǰ�ؼ�֡��ջ����֡��ͬ�ĵ�����
    float mLoopScore;//��ǰ�ؼ�֡��ջ����֡�ĵ������ֶ�
    long unsigned int mnRelocQuery;
    int mnRelocWords;//����ؼ�֡�뵱ǰ֡�Ĺ�ͬ�ĵ��ʸ���
    float mRelocScore;//����ؼ�֡�뵱ǰ֡��bow���Ʒ���

    // Variables used by loop closing
    cv::Mat mTcwGBA;//ͨ��ȫ���Ż��õ��Ĺؼ�֡��λ��
    cv::Mat mTcwBefGBA;
     //���Ƕ�֪��ֻ�м�⵽�˻ػ������ǲ��ܽ���ȫ���Ż���ȫ���Ż����õ��Ż���Ĺؼ�֡��λ�ˣ���ô����֪���Ż���Ĺؼ�֡λ��������һ���ؼ�֡�м�⵽�˻ػ�Ȼ��õ��ġ�
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points ����ͼ������������ƥ�����ͼ���������u������
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;//Ĭ��ֵ��1.2
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;////ÿ�������������ű���,����˵ͼ��Ҫ��С2��һ��4�㣬�򱣴�ľ���[1,2,4,8]
    const std::vector<float> mvLevelSigma2;//��mvScaleFactors��ƽ��
    const std::vector<float> mvInvLevelSigma2;//��mvLevelSigma2�ĵ���

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;//���������


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    //�뵱ǰ�ؼ�֡�Ѿ�ƥ��ĵ�ͼ��-�Ҹо�Ӧ����tracking�õ���
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    //���GetFeaturesInArea����
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    //mvpOrderedConnectedKeyFrames+mvOrderedWeights=������mConnectedKeyFrameWeights
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;//�洢�����뵱ǰ�ؼ�֡��ص����㹲��֡������¼�乲�Ӹ���
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;//�洢�����뵱ǰ�ؼ�֡��ص����㹲��֡(���Ӹ�������15��)
    std::vector<int> mvOrderedWeights;//�洢�������㹲��֡�Ĺ��Ӹ�������Ӧ����mvpOrderedConnectedKeyFrames�еĹ��Ӹ���

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;//����С�������е�ǰ�ؼ�֡�ĸ��ؼ�֡
    std::set<KeyFrame*> mspChildrens;//mspChildrens����Ĺؼ�֡�кܶ�Ĺ���֡����ֻ�е�ǰ�ؼ�֡��mspChildrens��Ĺؼ�֡���Ӹ������
    //�����ǰ�ؼ�֡�ǵ�ǰ֡��������������Ƕ�Ӧ���ŵıջ���ѡ֡
    //�����ǰ�ؼ�֡�����ŵıջ���ѡ֡��������������Ƕ�Ӧ�ĵ�ǰ֡
    std::set<KeyFrame*> mspLoopEdges;
    // Bad flags
    bool mbNotErase;//��ʾ��ǰ�ؼ�֡�����Ա�ɾ����Ϊ�����ڱ�������
    bool mbToBeErased;//���Ϊtrue��ʾ��ǰ֡��Ҫ��ɾ����
    bool mbBad; //Ӧ���Ǳ�ʾ��ǰ�ؼ�֡�Ƿ���Ч   

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
