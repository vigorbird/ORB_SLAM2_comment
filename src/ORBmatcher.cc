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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

//��֪�ֲ���ͼ���λ�ú͵�ǰ֡��λ�ˣ����ֲ���ͼ��ͶӰ����ǰ֡�в��뵱ǰ֡�����������ƥ��,���ƥ��ɹ��������ͼ����ӵ���ǰ֡��
//����ֵ�ǵ�ͼ���뵱ǰ֡������ƥ��ĸ���
//�����������SearchLocalPoints������
//����㷨ʵ���ĵ�
int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;
    //������ͼ��
    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)//Ӧ�����ж������ͼ���Ƿ���Ҫ�����٣�������������һ֡ƥ���򲻽���������Χ
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;//��ͼ���ܹ�����һ��Ľ������б�����Ϊ������

        // The size of the window will depend on the viewing direction
        //��ͼ���ƽ�������� �� ��ͼ��͵�ǰ֡���ɵķ�����֮��ļнǾ����������ķ�Χ���н�С��������ΧС��
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;
        ///����ͼ��ӳ�䵽��ǰ֡��ͼ���ϣ����ڴ˵����Χ������������Ϊ��ѡƥ���
        const vector<size_t> vIndices = F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();//��õ�ͼ���������

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        //������ǰ֡�еĺ�ѡƥ��㣬�ҵ����ͼ����ƥ��ĺ�ѡ��
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)//�жϵ�ǰ֡����ͼ����������ͼ��ӳ�䵽��ͼ�����������̫��
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)//Ĭ��ֵ��100
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;//����ƥ��ú󣬽���ͼ�㸳ֵ����ǰ֡������Ϊ�˵�ͼ���뵱ǰ֡����ƥ�䡣
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}

//kp1�ǵ�ǰ֡�����㣬kp2�ǹ���֡�����㣬F12������֮֡��Ļ�������,pKF2�ǹ���֡
bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}


// Search matches between MapPoints in a KeyFrame and ORB in a Frame.
 // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
 // Used in Relocalisation and Loop Detection
 //ע������������SearchByBoW���� �ڶ�������һ���ǹؼ�֡ һ������ͨ֡.ǰ�������������룬���һ�������������ƥ��ؼ�֡�еĵ�ͼ��
//�����������Ҫ�����ǹؼ�֡�еĵ�ͼ����֪����Ҫ����ͨ֡�е����������ƥ�䣬����Ľ����ƥ���ϵĹؼ�֡�еĵ�ͼ�㡣
//ע�����������ȫʹ�õľ���bow+�����ӽ���ƥ�䣬û��ǣ��������ͼ��ͶӰ����ǰ֡�������
//vpMapPointMatches���������ͨ֡�е���������ţ������ǹؼ�֡��ƥ��ĵ�ͼ��
//�˺�������TrackReferenceKeyFrame������ʹ����תһ���Լ��
//����㷨ʵ���ĵ�
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    //�ؼ�֡��bow����localmapping�߳��е�processnewkeyframe��������õ���
    //��ǰ֡��bow����trackframe�����м���õ���
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();//�ؼ�֡��ͼ��

    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;//Vector of nodes with indexes of local features ����㷨ʵ���ĵ������˵��

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];//����ǽǶȣ������ǵ�ǰ֡����������
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();//�ؼ�֡��
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();//��ǰ֡��
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    //�Թؼ�֡��ͬ�ĵ��ʽ��б���
    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;
             //����������ͬ���ʵĹؼ�֡�е���������б���
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);//�ؼ�֡��ͼ���������

                int bestDist1=256;
                int bestIdxF =-1 ;//��ǰ֡�����������
                int bestDist2=256;
		 		//����������ͬ���ʵĵ�ǰ֡�е���������б���
                //���㵱ǰ֡��ؼ�֡������������������ӡ���������bestDist2��bestDist1(��С�ľ���)
                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<=TH_LOW)//Ĭ��ֵ��50
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))//mfNNratioĬ��ֵ��0.6����һ��С�ľ���ȵڶ�������㹻С
                    {
                        vpMapPointMatches[bestIdxF]=pMP;

                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];

                        if(mbCheckOrientation)
                        {
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);//���λ�ñ����һ����С��value ��ֵ���ú���ΪC++ STL�ڵĺ�����Ҳ��˵������һ������
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }//whileѭ������

    //�����תһ���Լ�⣬�޳����õ�������
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);//HISTO_LENGTH=30����ʾ��ֱ��ͼ����30�ȷ�

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

//��һ��������:��ǰ�ؼ�֡�����룻�ڶ��������ǵ�ǰ�ؼ�֡����������ϵ�µ�λ�ˣ����롣
//���������������űջ���ѡ֡���乲��֡�����ĵ�ͼ�㣬���룻���ĸ����������űջ���ѡ֡�͵�ǰ֡ƥ���inliner��ͼ��,�����
//���������������:��ǰ֡�Ѿ������űջ���ѡ֡�Ѿ�ƥ����һ�������ĵ�ͼ�㣬����������õ������ƥ���ͼ�㡣
//���ǽ����űջ���ѡ֡�Ĺ���֡�������ĵ�ͼ��ͶӰ����ǰ֡�У��뵱ǰ֡�����������ƥ�䣬���ƥ�����������vpMatched��vpMatched����ǵ�ǰ֡���������ţ����������űջ���ѡ֡�Ĺ���֡�������ĵ�ͼ��
//�˺�������ComputeSim3������
int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    //�������űջ���ѡ֡���乲��֡�����ĵ�ͼ��
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        //��ͼ���ڵ�ǰ֡����ϵ�µ�λ��
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//�õ���Щ��ͼ���ڵ�ǰ֡�µ�ͶӰ��������
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist,pKF);//���ݵ�ͼ����뵱ǰ֡�ľ��룬�õ������ͼ����ڵ�ǰ֡�н�������һ�㱻������

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];//���ݽ������Ĳ����ĵ������뾶

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);//�õ�ͶӰ�����������Χ�ĺ�ѡ������

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //����ͼ����ͼ���еĺ�ѡ���������ƥ�䣬�ҵ�������֮�������С���Ǹ���ѡ������
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int &kpLevel= pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

//ע�����������֡���ǹؼ�֡
//�����ؼ�֡���ж�Ӧ�ĵ�ͼ�㣬Ŀ����ƥ�������ؼ�֡�еĵ�ͼ�㲢�����vpMatches12��������ƥ��ĵ�ͼ��������
//vpMatches12��һ��vector������ǹؼ�֡1����������ţ�������ƥ��Ĺؼ�֡2�еĵ�ͼ��
//�˺�������ComputeSim3����
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    //��ȡ�������ؼ�֡�е������㣬������������ͼ�㣬������
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;

    //
    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;//Ĭ��ֵ��30

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

   //���������ؼ�֡��������������
    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)//��������ؼ�֡�еĵ�����ͬ
        {   
            //�����ؼ�֡1�е���X��Ӧ��������
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;//�����С��������֮��
                int bestIdx2 =-1 ;
                int bestDist2=256;
               //�����ؼ�֡2����ͬ����X��Ӧ��������
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);//����ؼ�֡1�͹ؼ�֡2������ͬ���������������������֮��

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))//�����СֵԶԶС�ڵڶ���Сֵ
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];//
                        vbMatched2[bestIdx2]=true;

                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
	 //�������ؼ�֡�еĵ��ʲ���ͬ��
	 //����ؼ�֡1��������ؼ�֡2��ʹ�ؼ�֡1����ؼ�֡2
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);//�ҵ���һ�����ڻ����f2it->first
        }
        else //����ؼ�֡2��������ؼ�֡1��ʹ�ؼ�֡2����ؼ�֡1
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

//ͨ��֡��ʵ���������ƥ�䣬��ʹ�ü���Լ������ƥ���������  
//F12Ϊ��ǰ֡�͹���֡�Ļ����������ΪvMatchedPairs�����������ƥ������������֡ͼ���е���š���һ�������ǵ�ǰ֡�е���������ţ��ڶ��������ǹ���֡�����������
//���ص�int���͵ı�����ƥ���������
//pKF1Ϊ����ĵ�ǰ�ؼ�֡��pKF2Ϊ��ǰ֡�Ĺ��ӹؼ�֡��bOnlyStereo����Ϊfalse
//vMatchedPairs������������ǹ���֡�е���������ţ�����ǵ�ǰ֡������������
//����һ��Ҫע��!!!!!!!!!!���ƥ�����������pkf1����pdf2���Ѿ������˶�Ӧ�ĵ�ͼ���������vMatchedPairs��!!!!!!!
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat C2 = R2w*Cw+t2w;//��õ�ǰ֡�������ڹ���֡����ϵ�µ�����
    const float invz = 1.0f/C2.at<float>(2);
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;//��ǰ֡�������ڹ���֡ͼ���е���������
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);//�����ǹ���֡�е���������ţ�����ǵ�ǰ֡������������

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)//������ͬ�ĵ���
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)//�������ڵ�ǰ�ؼ�֡֡��������е�����������
            {
                const size_t idx1 = f1it->second[i1];//����������ʵ��ڵ�ǰ֡ͼ���е����
                
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                
                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);//��ǰ�ؼ�֡֡����������ʵ�������������
                
                int bestDist = TH_LOW;//������ͬ�����뵱ǰǰ֡ƥ����õ�������������֮��
                int bestIdx2 = -1;//������ͬ�����뵱ǰǰ֡ƥ����õ��������ڹ���֡�е����
                //��������������ʵ��ڹ��ӹؼ�֡�е�������
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];
                    
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
                    
                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    const bool bStereo2 = pKF2->mvuRight[idx2]>=0;

                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;
                    
                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);//����֡����������ʵ�������������
                    
                    const int dist = DescriptorDistance(d1,d2);//���㵱ǰ֡�͹���֡������ͬ����һ�����ʵ������������ӵĲ�
                    
                    if(dist>TH_LOW || dist>bestDist)//TH_LOW=50
                        continue;

                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                    if(!bStereo1 && !bStereo2)//�����ǰ֡�͹���֡����ͼ��û�����������
                    {
                        const float distex = ex-kp2.pt.x;
                        const float distey = ey-kp2.pt.y;
                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }
		     		//�жϵ�ǰ֡��������͹���֡�е��������Ƿ������������涨�ļ��޶���!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                
                if(bestIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;

                    if(mbCheckOrientation)//�������߲�ʹ����תһ���Լ��
                    {
                        float rot = kp1.angle-kp2.angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }//���forѭ������

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);//�ҵ���һ�����ڻ����f2it->first
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }//whileѭ������

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}


// ͶӰ��ͼ�㵽��ǰ�ؼ�֡�ϣ��ڸ�����������ƥ��ؼ��㣬���ƥ�����˵�ǰ֡�������㣬���жϵ�ǰ֡������������Ƿ��ж�Ӧ�ĵ�ͼ��  
// ����õ��ж�Ӧ��MapPoint����ô������MapPoint�ϲ����ù۲�����Ĵ���۲����ٵĵ�ͼ�㣩  
// ����õ�û�ж�Ӧ��MapPoint����ô��ǰ֡��������ͼ��
//thĬ��ֵ��3,���ھ�����ͼ���������İ뾶
//���ص�ͼ��͵�ǰ֡�������ںϵĸ���
//��������localmapping���ںϵ�ͼ���fuse����
int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
{
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    cv::Mat Ow = pKF->GetCameraCenter();

    int nFused=0;

    const int nMPs = vpMapPoints.size();
    //Fuse����������һ�����forѭ������
    //������������е�ͼ��
    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw*p3Dw + tcw;//�����������������ϵ�µ�λ�˺͵�ͼ������������ϵ�µ�λ�� �õ���ͼ���ڵ�ǰ֡����ϵ�µ�λ������

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//�õ����������ͶӰ����ǰ֡��ͼ���е���������
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float ur = u-bf*invz;//�õ����������ͶӰ����ǰ֡��ͼ���е���������

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

	//�õ�ͼ���еĺ�ѡ������
	//���ص����ڰ뾶��Χ�������������ͼ���е����������
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();//��ͼ���������

        int bestDist = 256;
        int bestIdx = -1;
	//��������õ��ĺ�ѡ������
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];//��ͼ���еĺ�ѡ������

            const int &kpLevel= kp.octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            if(pKF->mvuRight[idx]>=0)//����ͼ���е�����������ͼ����Ҳ������
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                    continue;
            }
            else//����ͼ���е�����������ͼ����û�г���
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);//��ѡ�������������

            const int dist = DescriptorDistance(dMP,dKF);//�����ͼ�������Ӻͺ�ѡ�����������ӵĲ�

            if(dist<bestDist)//�ҳ�������ĺ�ѡ������
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
         if(bestDist<=TH_LOW)//Ĭ��ֵ��50
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);//�õ���ѡ�������Ӧ�ĵ�ͼ��
            if(pMPinKF)//�����ѡ�������ж�Ӧ�ĵ�ͼ��
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())//�����ѡ�������Ӧ�ĵ�ͼ�㱻����֡�����Ĵ������������ͼ�㱻�����Ĵ���
                        pMP->Replace(pMPinKF);//�ú�ѡ������ĵ�ͼ���������ĵ�ͼ��
                    else
                        pMPinKF->Replace(pMP);//������ĵ�ͼ������ѡ������ĵ�ͼ��
                }
            }
            else//�����ѡ������û�ж�Ӧ�ĵ�ͼ��
            {
                pMP->AddObservation(pKF,bestIdx);//���µ�ǰ��ͼ�����Ϣ����ǰ��ͼ�㱻��ǰ֡�������뵱ǰ֡��������bestIdx��Ӧ
                pKF->AddMapPoint(pMP,bestIdx);//���µ�ǰ֡�ĵ���Ϣ�������������ͼ��
            }
            nFused++;//���ص�ͼ��͵�ǰ֡�������ںϵĸ���
        }
    }

    return nFused;
}

//pKF�ǵ���֡��Scw���Ѿ���������λ��
//vpPoints�����űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�����ĵ�ͼ�㣬û�о����ػ�������
//vpReplacePointΪ��������������vpPointsʹ�õ���ţ������ǵ���֡�ĵ�ͼ��
//��������������ǽ����űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�����ĵ�ͼ�� ͶӰ�� ������λ�˵ĵ�ǰ֡�Ĺ���֡�в�����������ƥ��
//���ƥ��ɹ�������������������֡���ж�Ӧ�ĵ�ͼ�㣬�򽫵���֡�ĵ�ͼ�㸳ֵ��vpReplacePoint
//���ƥ��ɹ������������������֡��û�ж�Ӧ�ĵ�ͼ�㣬�򽫹���֡��ͼ����뵽����֡�У����������ͼ�㱻����֡����
int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();//����֡�ĵ�ͼ��

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    //�������űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�����ĵ�ͼ�㣬�������������ôһ�����forѭ��
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];//���űջ���ѡ�ؼ�֡�Ĺ���֡+���űջ���ѡ�ؼ�֡�����ĵ�ͼ��

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        //���ջ���ѡ�ؼ�֡���乲��֡�����ĵ�ͼ��ͶӰ����ǰ֡�Ĺ���֡��
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//��ǰ֡����֡����������
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;//�ջ���ѡ�ؼ�֡���乲��֡�����ĵ�ͼ�� ���뵱ǰ֡����֡���ĵľ���
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();//�ջ���ѡ�ؼ�֡���乲��֡�����ĵ�ͼ�� ƽ��������

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);//�ڵ���֡ͼ���һ���뾶��Ѱ�Һ�ѡ������

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //�ں�ѡ������ķ�Χ���ҵ���ջ���ѡ֡�����ĵ�ͼ����Ϊƥ���������
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int &kpLevel = pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;//����֡ͼ���е����������
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        //�Ѿ��ڵ���֡ͼ�����ҵ�����Ϊƥ���������
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);//���ݵ���֡ͼ���е�����ƥ���������ҵ�����֡�ж�Ӧ�ĵ�ͼ��
            if(pMPinKF)//��������űջ���ѡ֡�еĵ�ͼ��ƥ��ĵ���֡�е��������е�ͼ��
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;//ʹ�õ���֡�еĵ�ͼ����¼���Ҫ����ı���vpReplacePoint
            }
            else//��������űջ���ѡ֡�еĵ�ͼ��ƥ��ĵ���֡�е�����û���е�ͼ��
            {
                pMP->AddObservation(pKF,bestIdx);//���¹���֡��ͼ�㱻����֡����
                pKF->AddMapPoint(pMP,bestIdx);//���µ���֡�������������֡�ĵ�ͼ��
            }
            nFused++;
        }
    }

    return nFused;
}


//��һ�������ǵ�ǰ�ؼ�֡���ڶ��������Ǻ�ѡ�ջ��ؼ�֡��������������������Ҳ��������ǵ�ǰ֡�ͺ�ѡ�ջ�֡ƥ��ĵ�ͼ��
//R12��t12�ǵ�ǰ֡�ͺ�ѡ�ջ��ؼ�֡�����λ�ˣ����һ������th�����������İ뾶
//vpMatches12�е�����ǵ�ǰ֡ͼ�������������ţ������Ǻ�ѡ�ջ��ؼ�֡����Ӧƥ��ĵ�ͼ��
//��������������ǽ���ǰ֡�ĵ�ͼ��ͶӰ����ѡ�ջ��ؼ�֡�н���������ƥ��
//����ѡ�ջ��ؼ�֡�ĵĵ�ͼ��ͶӰ����ǰ�ؼ�֡�н���������ƥ�䣬
//�������ƥ��������ȫһ�������vpMatches12
int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                                                             const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();//��ǰ֡�ĵ�ͼ��
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();//�ջ���ѡ�ؼ�֡�ĵ�ͼ��
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

   //���Ѿ�ƥ��ĵ�ͼ���ڵ�ǰ֡�ͱջ���ѡ�ؼ�֡�ĵ�ͼ���б�ʶ����
    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];//��ǰ֡�ͱջ���ѡ�ؼ�֡�Ѿ�ƥ��ĵ�ͼ��
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);//��ű�ʾ��ǰ֡��ͼ�����ţ����ݱ�ʾ�ջ���ѡ�ؼ�֡�е�ƥ�����������
    vector<int> vnMatch2(N2,-1);//��ű�ʾ�ջ���ѡ�ؼ�֡��ͼ�����ţ����ݱ�ʾ��ǰ֡�е�ƥ�����������

    // Transform from KF1 to KF2 and search
    //������ǰ֡�ĵ�ͼ��
    //����ǰ֡��ͼ��ͶӰ���ջ���ѡ�ؼ�֡��ͼ���н�������ƥ��
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;//��ǰ֡��ͼ���ڵ�ǰ֡����ϵ��λ��
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;//��ǰ֡��ͼ���ڱջ���ѡ�ؼ�֡����ϵ��λ��

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;//��ǰ֡��ͼ���ڱջ���ѡ�ؼ�֡ͼ���е���������
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);//�õ��ջ���ѡ�ؼ�֡ͼ���к�ѡ��ƥ��������

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //��ǰ֡��ͼ���������
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
	//����ǰ֡��ͼ�����������ջ���ѡ�ؼ�֡�еĺ�ѡ�������������Ƚ�
	//�����������
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    //�����ջ���ѡ�ؼ�֡�ĵ�ͼ��
    //���ջ���ѡ�ؼ�֡��ͼ��ͶӰ����ǰ֡��ͼ���н�������ƥ��
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;//
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;
    //
    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];//�õ���ǰ֡��ͼ��ͶӰ���ջ���ѡ�ؼ�֡ͼ���е����������

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];//�õ��ջ���ѡ�ؼ�֡��ͼ��ͶӰ����ǰ֡ͼ���е����������
            if(idx1==i1)//��������໥ͶӰƥ������໥һ�£������vpMatches12
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

// Search matches between Frame keypoints and projected MapPoints. Returns number of matches
 // Used to track the local map (Tracking)
 //����������ص��ǵ�ǰ֡�е����������һ֡��ͼ��ƥ��ĸ���
 //˫Ŀ�����bMono=false
 //��������Ļ���˼·:�����Ѿ�֪������һ֡��λ�˺͵�ǰ֡��һ��Ԥ��λ�ˣ�����֪����һ֡�е�ͼ�㣬������������þ��ǽ���һ֡�ĵ�ͼ����ݵ�ǰ֡��Ԥ��λ��ͶӰ����ǰ֡��ͼ����
 //Ȼ���ڵ�ǰ֡ͶӰ�õ������ص���Χ����һ֡���е�ͼ���ƥ�䣬ƥ��ɹ���ʹ�ò�������תһ���Լ��ɸѡ����һ֡���õĵ�ͼ�㣬Ȼ��ɸѡ��ĵ�ͼ����µ�ǰ֡�ĵ�ͼ��
 //��ԭ��������TrackWithMotionModel������
 //����㷨ʵ���ĵ�
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];//HISTO_LENGTH=30
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);//��ǰ֡����̬(��������˶�ģ������ǰ֡����̬�Ǹ����ٶȺ���һ֡��λ�˵õ��Ĺ���ֵ)��Ϊʲô��һ��4*4�ľ���???
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);//��ǰ֡��λ��

    const cv::Mat twc = -Rcw.t()*tcw;//����㷨ʵ���ĵ����õ��������ϵ�µĵ㵽��������ϵ���λ�ñ任

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);//��һʱ��֡����̬��
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);//��һʱ��֡��λ��

    const cv::Mat tlc = Rlw*twc+tlw;//��ǰ֡����һʱ��֡��λ��

    //mb��˫Ŀ���߳��ȣ���λ����
    //ֻ����ǰ��������ƶ��ľ��볬��˫Ŀ�Ļ��߲����ƶ��ˡ�
    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    //��ѯ��һ֡�е����е�ͼ��
    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // Project
                cv::Mat x3Dw = pMP->GetWorldPos();//�õ���һ֡��ͼ�������λ��
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//�õ��������ڵ�ǰ֡����ϵ�µ�λ��

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

		  //�õ��ĵ�һ�����������ϵ�µ���ȱ���������
                if(invzc<0)
                    continue;
	         //�õ���һ֡��ͼ���ڵ�ǰ֡��ͼ���е���������
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

	         //���Ƶõ�����������Ҫ�ڷ�Χ֮��
                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                 
                int nLastOctave = LastFrame.mvKeys[i].octave;

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);//������ǰ֡���ص�һ���뾶��Χ�ڵ�����������
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

	         //������һ֡��ͼ�������������
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;
                //��ѯ��ǰ֡ͶӰ�õ�����������Χ������������
                //�õ�һ������һ֡��ͼ��������������㣬����¼����ͼ���е����������кż�ΪbestIdx2
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;//�����Ӳʽ�����ͼ��������������õ��Ҳ�ͼ���ϵ���������
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);//�����Ӳʽ�õ������������ʵ��ͨ������ͼ��ƥ��ĵ����������겻�����̫��
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

	         //�����ǰ֡���������ܹ��Ϻõ�ƥ����һ֡��ͼ�㣬��ʹ����һ֡�ĵ�ͼ����µ�ǰ֡�ĵ�ͼ��
                if(bestDist<=TH_HIGH)//TH_HIGH=100
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;
                          
                    if(mbCheckOrientation)//Ĭ�ϵ��Ǽ���������ķ����
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;//�������360�жϣ������㷽��ĵ�λӦ���Ƕ�
                        int bin = round(rot*factor);//factor=1/30
                        if(bin==HISTO_LENGTH)//HISTO_LENGTH=30
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
				
            }
			
        }
    }

    //Apply rotation consistency
    //Ĭ�ϵ��Ǽ���������ķ����
    //���������Ŀ���Ǽ���Ѿ�ƥ��ĵ�ǰ֡����һ֡���������Ƿ���һ�£�����һ��ֱ��ͼ��ֻ�����������ֱ��ͼ�е���һ֡��ͼ�������µ�ǰ֡�ĵ�ͼ��
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);//����ֱ��ͼrotHist�е��������������������������������������

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

// Project MapPoints seen in KeyFrame into the Frame and search matches.
 // Used in relocalisation (Tracking)
 //��ǰ֡��������ؼ�֡��ͼ�����ƥ�䣬����ֵ��ƥ���������ĸ���
 //���������Ѿ�֪���˵�ǰ֡��λ�˺͹ؼ�֡�е�ͼ������������ϵ�µ�λ�ã�˼·�ǽ��ؼ�֡�еĵ�ͼ��ͶӰ����ǰ֡��Ȼ�������������Χ�����������ҵ�ƥ��ĵ�ͼ��
//����ҵ���ƥ�䣬��ʹ������֡�ĵ�ͼ����µ�ǰ֡�ĵ�ͼ�㡣
 //����Ĳ���th��ʾ��ͼ���������ķ�Χ��ORBdist��ʾ�뵱ǰ���������Ƶ�һ����ֵ���������������������С�������������Ϊƥ��ɹ�
//�������������Relocalization������
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);//��ǰ֡����������ϵ�µ�λ��
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);//��ǰ֡����������ϵ�µ�λ��
    const cv::Mat Ow = -Rcw.t()*tcw;//Ӧ���ǵ�ǰ֡��������������ϵ�µ�����

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();//��ȡ�ؼ�֡�е�ͼ��

    //��ѯ�ؼ�֡�еĵ�ͼ��
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();//��ͼ������������ϵ�µ�λ��
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//��ͼ�����������ϵ�µ�λ��

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;//�õ���ͼ���ڵ�ǰ֡ͼ���е���������
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Compute predicted scale level
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);//

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);//�ڶ�������

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();//�õ��ؼ�֡�е�ͼ���������

                int bestDist = 256;
                int bestIdx2 = -1;

		 //��ѯ��ͼ���ڵ�ǰ֡��������Χ������������
                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)//��תһ���Լ�⣬Ϊ��״ͼ����������
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)//��תһ���Լ�⣬�ҵ����������״ͼ��������
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

//�ҵ�ֱ��ͼhisto�е������������ݲ������Ƕ�Ӧ����ű������������������С�
void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
//Ӧ�þ��Ǽ�����������ľ��룬������������֮������ƶ�
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
