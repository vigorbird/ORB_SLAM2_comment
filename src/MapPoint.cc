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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

//�½�һ����ͼ����Ҫ���µ���Ϣ:
//�����ͼ���Ӧ�������ӡ���ͼ�����š�mfMinDistance��mfMinDistance
MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* p Frame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))//��������ͼ���Ѿ��ڵ�ǰ֡���򷵻�
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)//����Ҳ�����ͷҲ�۲⵽�������ͼ��
        nObs+=2;
    else
        nObs++;
}

//���������ͼ�㲻��pKF����ؼ�֡����
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);
	     //��Ҫɾ���ؼ�֡�����������ͼ�㣬
	    //�����������ؼ�֡�������ͼ��Ĳο��ؼ�֡�������ͼ��Ĳο��ؼ�֡��mObservations��һ���ؼ�֡����ֵ
            if(mpRefKF==pKF)
                 mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

//���������ͼ��ı���mObservations
//ԭ���ܹ����������ͼ��Ĺؼ�֡���������ǹ۲ⲻ�������ͼ��
//��������ͼ������ڵ�ͼ��ɾ��
void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

//ʹ��pMP��ͼ����������ͼ��
//ͬʱpMP�����ͼ��̳��������ͼ��ı��۲����Ϣ��ע��û�м̳���λ����Ϣ
void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP; 
    }

    //�������������ͼ������йؼ�֡
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))//����ĵ�ͼ�㲻�������ͼ��Ĺؼ�֡��
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);//���������ͼ��Ĺؼ�֡��Ϊ����pMP�����ͼ�㡣
            pMP->AddObservation(pKF,mit->second);//����ĵ�ͼ�㱻�����ͼ��Ĺؼ�֡�۲⵽
        }
        else//����ĵ�ͼ���������ͼ��Ĺؼ�֡��
        {
            pKF->EraseMapPointMatch(mit->second);//�������ͼ��������ͼ��Ĺؼ�֡��ɾ��
        }
    }
    pMP->IncreaseFound(nfound);//��Ϊ��������ͼ���ں�Ϊ1�ˣ����������tracking�б���ͨ֡�۲�Ĵ�����Ҫ���
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);//�ӵ�ͼ��ɾ�������ͼ��
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)//����Ĭ������ֵ��1
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

//
void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;//�������Ż���Чʹ�������ͼ��Ĺؼ�֡����/ǰ��ƥ��õ��Ĺؼ�֡����
}

//�۲⵽�õ�ͼ��Ķ���������У���Ӧ����ؼ�֡������ѡ����ƽ���������ӣ���Ϊ�����ͼ���������
//����㷨ʵ���ĵ�-�о�������Խ��иĽ�!!!!!!!!!!!!!!!!!!!!��
//��������û���������ӵ�ƽ��ֵ�ǲ��ǵ��ļ�����̫��?����������ķ����ܹ����������ӵ�ֱ�Ӽ���
void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)//��ʱ����֪���������������
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
	     //row������ʾ�������ĵڼ��У������ӵ�������ʾ������ĸ�����������ʾ�����ӵ�ά�ȡ�
	     //������仰����˼���ǽ��ؼ�֡�е�������ѹ�뵽vDescriptors��
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    //��������������֮��Ĳ�ֵ���浽Distances��
    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    //��Distances��ÿһ�е����ݽ�������ѡ���м�ֵ������ÿһ�е��м�ֵ���ҵ���С���Ǹ��м�ֵ������¼�к�ΪBestIdx
    int BestMedian = INT_MAX;//int�ܹ���ʾ��������ֵ
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)]; 

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));//���ص��ǵ�ͼ���Ƿ�������ؼ�֡�У�ֻ�ܷ���0����1
}

//���µ�ͼ��ķ��ߡ���ע:һ����ͼ���ͬʱ������ؼ�֡�������������������ڸ���֡�е�ƽ�����������������ڲο�֡�е�
//ͬʱ���������ͼ��ĳ߶Ȳ������:mfMaxDistance��mfMinDistance
void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();//����������������ϵ�µ�λ��
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    //����ؼ�֡��������ķ�����=����������������ϵ�µ�λ��-�۲⵽��������ؼ�֡�������λ�ã��������ۼ�
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);//����������������ο�֡���ĵľ���
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;//���ص����������ڲο�֡�н���������һ�㱻��������
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];//���ݽ������Ĳ����õ����ŵı���
    const int nLevels = pRefKF->mnScaleLevels;//�õ��ο�֡�ܵĽ������Ĳ���

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;//mfMaxDistanceֻ��������PredictScale������
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
		
        mNormalVector = normal/n;//���ۼӵõ���������ƽ���õ���ͼ���ƽ��������
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

//����ĵ�һ�������ǵ�ͼ�㵽���ĵľ���
int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

//����ʹ�õ�ԭ����:mfMaxDistance=dist(��ǰ��ͼ�㵽����ľ���)*levelScaleFactor(��ͼ���Ӧ�������������ڵĽ�����������Ӧ��ͼ�����ű���)
//mfLogScaleFactor��ÿ��ͼ�����ű�����log
//Ŀ�ľ�������֪���������£����¹���������������ͼ�����һ�㱻����
int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;//mfMaxDistance=dist*levelScaleFactor
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} //namespace ORB_SLAM
