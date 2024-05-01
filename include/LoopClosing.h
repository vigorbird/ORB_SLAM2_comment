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
    //KeyFrameAndPose���ݽṹ��ʵ���ǹؼ�֡+Sim3��ɵ�map������Ķ�����ʾ�ڴ����
    //�ڶ���Ԫ��sim3��ʾ����ؼ�֡��λ��
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
	std::list<KeyFrame*> mlpLoopKeyFrameQueue;//����Ǿ���localmapping�߳��������Ĺؼ�֡
    std::mutex mMutexReset;//��ס����Դ��mbResetRequested��mLastLoopKFid��mlpLoopKeyFrameQueue------------------------------------

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;//��ס����Դ��mbFinishRequested��mbFinished-------------------------------------------------------

    Map* mpMap;//�ػ�����еĵ�ͼ��tracking�̵߳ĵ�ͼ��localmapping �еĵ�ͼһ����ռ��ͬ�������ݿռ�
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;//�Ҹо�����洢��Ӧ���Ǵ�localmap�߳��д����Ĺؼ�֡
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    
    std::mutex mMutexLoopQueue;//��ס����Դ��mlpLoopKeyFrameQueue

    // Loop detector parameters
    //Ĭ��ֵ��3
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;//��ǰ�ؼ�֡������ƥ�������ջ���ѡ֡
    KeyFrame* mpMatchedKF;//�洢�����뵱ǰ�ؼ�֡��ƥ������űջ���ѡ֡
    std::vector<ConsistentGroup> mvConsistentGroups;//����㷨ʵ���ĵ�
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;//�洢���������ԵĻػ�����ѡ֡
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;//�洢���ǵ�ǰ�ؼ�֡�Ĺ���֡+��ǰ�ؼ�֡
    std::vector<MapPoint*> mvpCurrentMatchedPoints;//����ǵ�ǰ�ؼ�֡���������ţ����������űջ���ѡ֡�Ĺ���֡�������ĵ�ͼ�㡣
    std::vector<MapPoint*> mvpLoopMapPoints;//���űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�����ĵ�ͼ�㣬û�о����ػ��ںϵ�����λ��
   
    cv::Mat mScw;//��ǰ֡����������ϵ�µ�λ�� opencv���ݸ�ʽ
    g2o::Sim3 mg2oScw;//��ǰ֡����������ϵ�µ�λ�ˣ�g2o���ݸ�ʽ

    

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;//ȫ���Ż��Ƿ���ִ��
    bool mbFinishedGBA;//��ʾȫ���Ż��Ѿ�������
    bool mbStopGBA;//�Ƿ�ֹͣȫ���Ż�
    std::mutex mMutexGBA;//��ס����Դ��mbStopGBA---------------------------------------------------------------------------------
    std::thread* mpThreadGBA;//�����loopclosing�߳������ڿ���ȫ���Ż����߳�

    // Fix scale in the stereo/RGB-D case
    //��˫Ŀ�������true����ʾ�Ƿ�߶���ȷ����
    bool mbFixScale;


    bool mnFullBAIdx;//��ʾ��⵽�˼��λػ�
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
