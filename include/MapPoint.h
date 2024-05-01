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
    long unsigned int mnId;//���ڼ�¼��ͼ������
    static long unsigned int nNextId;//���ڼ�¼һ�������˶��ٸ���ͼ��
    long int mnFirstKFid;//�����õ�ͼ��ĵ�һ���ؼ�֡�����
    long int mnFirstFrame;
    int nObs;//�����ͼ�㱻�����ؼ�֡�۲⵽��

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;//�������Ӧ���ǻ��Ƿ����׷�������ͼ��
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;//�������Ӧ���ǵ�ǰ�������ͼ��������ĳ��֡��λ�ˣ���������洢�ľ���ĳ��֡�����
    long unsigned int mnLastFrameSeen;//�ոտ��������ͼ���֡�����

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;//�����ͼ�������ھֲ��Ż�����������Ǿֲ��Ż�ʱ��ǰ֡�����
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    //����������ڱջ�����У������Ǽ�⵽�ջ������Ǿ���Ҫ�Ե�ǰ֡���乲��֡�еĵ�ͼ����е���
    //�������ָ����ǰ�����ͼ������Ϊ��һ����ǰ֡�������ĵ���
    long unsigned int mnCorrectedByKF;
    //�������ָ����ǰ�����Ҫ�������ĵ�ͼ�����ڵ�ǰ֡�е��ĸ�����֡�б�����Ȼ����Ϊ�ػ��������е����ġ�
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;//ͨ��ȫ���Ż��õ��ĵ�ͼ���λ��
    //���Ƕ�֪��ֻ�м�⵽�˻ػ������ǲ��ܽ���ȫ���Ż���ȫ���Ż����õ��Ż���Ĺؼ�֡��λ�ˡ���ô����֪���Ż���Ĺؼ�֡λ��������һ���ؼ�֡�м�⵽�˻ػ�Ȼ��õ��ġ�
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     //�㷨ʵ���ĵ��жԴ˱���������˵��
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction=������ķ�����ƽ��ֵ��������ķ�����=����������������ϵ�µ�����-�۲⵽���������������������������ϵ�µ�����
     //��������������û�б�ʹ�ù�
	 cv::Mat mNormalVector;

     // Best descriptor to fast matching
     //��ͼ��������ӣ���ComputeDistinctiveDescriptors�����õ�
     cv::Mat mDescriptor;

     // Reference KeyFrame=�뵱ǰ֡�������ͬ�������֡����������е㲻����??????????????
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;//ǰ��ƥ��õ�����ͨ֡���� ��ʼֵ��1
     int mnFound;//�������Ż���Чʹ�������ͼ�����ͨ֡���� ��ʼֵ��1

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;//������������Ϊ�գ���ǰ�������ͼ����Ҫ��mpReplaced�����ͼ���滻

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
