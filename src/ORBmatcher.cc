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

//已知局部地图点的位置和当前帧的位姿，将局部地图点投影到当前帧中并与当前帧的特征点进行匹配,如果匹配成功则将这个地图点添加到当前帧中
//返回值是地图点与当前帧特征点匹配的个数
//这个函数用在SearchLocalPoints函数中
//详见算法实现文档
int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;
    //遍历地图点
    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)//应该是判断这个地图点是否还需要被跟踪，如果这个点与上一帧匹配则不进入搜索范围
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;//地图点能够在哪一层的金字塔中被检测出为特征点

        // The size of the window will depend on the viewing direction
        //地图点的平均法向量 与 地图点和当前帧构成的法向量之间的夹角决定了搜索的范围，夹角小则搜索范围小。
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;
        ///将地图点映射到当前帧的图像上，并在此点的周围搜索特征点作为候选匹配点
        const vector<size_t> vIndices = F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();//获得地图点的描述子

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        //遍历当前帧中的候选匹配点，找到与地图点最匹配的候选点
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)//判断当前帧的右图像的坐标与地图点映射到右图像的坐标相差不能太多
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
        if(bestDist<=TH_HIGH)//默认值是100
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;//最终匹配好后，将地图点赋值给当前帧。则认为此地图点与当前帧进行匹配。
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

//kp1是当前帧特征点，kp2是共视帧特征点，F12是两个帧之间的基础矩阵,pKF2是共视帧
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
 //注意这里有两个SearchByBoW函数 第二个参数一个是关键帧 一个是普通帧.前两个参数是输入，最后一个参数是输出的匹配关键帧中的地图点
//这个函数的主要作用是关键帧中的地图点已知，想要和普通帧中的特征点进行匹配，输出的结果是匹配上的关键帧中的地图点。
//注意这个函数完全使用的就是bow+描述子进行匹配，没有牵扯到将地图点投影到当前帧的情况。
//vpMapPointMatches序号是在普通帧中的特征点序号，内容是关键帧中匹配的地图点
//此函数用于TrackReferenceKeyFrame函数，使用旋转一致性检测
//详见算法实现文档
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    //关键帧的bow是在localmapping线程中的processnewkeyframe函数计算得到的
    //当前帧的bow是在trackframe函数中计算得到的
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();//关键帧地图点

    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;//Vector of nodes with indexes of local features 详见算法实现文档对其的说明

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];//序号是角度，内容是当前帧特征点的序号
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();//关键帧的
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();//当前帧的
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    //对关键帧不同的单词进行遍历
    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;
             //对隶属于相同单词的关键帧中的特征点进行遍历
            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);//关键帧地图点的描述子

                int bestDist1=256;
                int bestIdxF =-1 ;//当前帧的特征点序号
                int bestDist2=256;
		 		//对隶属于相同单词的当前帧中的特征点进行遍历
                //计算当前帧与关键帧两个个最相近的描述子。并保存在bestDist2和bestDist1(最小的距离)
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

                if(bestDist1<=TH_LOW)//默认值是50
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))//mfNNratio默认值是0.6，第一最小的距离比第二个相比足够小
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
            KFit = vFeatVecKF.lower_bound(Fit->first);//这个位置标记了一个不小于value 的值。该函数为C++ STL内的函数。也就说遍历下一个单词
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }//while循环结束

    //检查旋转一致性检测，剔除不好的特征点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);//HISTO_LENGTH=30，表示将直方图划分30等分

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

//第一个参数是:当前关键帧，输入；第二个参数是当前关键帧在世界坐标系下的位姿，输入。
//第三个参数是最优闭环候选帧和其共视帧看到的地图点，输入；第四个参数是最优闭环候选帧和当前帧匹配的inliner地图点,是输出
//这个函数的作用是:当前帧已经和最优闭环候选帧已经匹配了一定数量的地图点，但是我们想得到更多的匹配地图点。
//我们将最优闭环候选帧的共视帧所看到的地图点投影到当前帧中，与当前帧的特征点进行匹配，如果匹配上了则更新vpMatched，vpMatched序号是当前帧特征点的序号，内容是最优闭环候选帧的共视帧所看到的地图点
//此函数用于ComputeSim3函数中
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
    //遍历最优闭环候选帧和其共视帧看到的地图点
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        //地图点在当前帧坐标系下的位置
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//得到这些地图点在当前帧下的投影像素坐标
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

        int nPredictedLevel = pMP->PredictScale(dist,pKF);//根据地图点距离当前帧的距离，得到这个地图点会在当前帧中金字塔哪一层被检测出来

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];//根据金字塔的层数的到搜索半径

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);//得到投影像素坐标点周围的候选特征点

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //将地图点与图像中的候选特征点进行匹配，找到描述子之间距离最小的那个候选特征点
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

//注意输入的两个帧都是关键帧
//两个关键帧都有对应的地图点，目的是匹配两个关键帧中的地图点并输出给vpMatches12，并返回匹配的地图点数量。
//vpMatches12是一个vector。序号是关键帧1的特征点序号，内容是匹配的关键帧2中的地图点
//此函数用于ComputeSim3函数
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    //提取出两个关键帧中的特征点，单词向量，地图点，描述子
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

    const float factor = 1.0f/HISTO_LENGTH;//默认值是30

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

   //遍历两个关键帧的两个单词向量
    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)//如果两个关键帧中的单词相同
        {   
            //遍历关键帧1中单词X对应的特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;//保存更小的描述子之差
                int bestIdx2 =-1 ;
                int bestDist2=256;
               //遍历关键帧2中相同单词X对应的特征点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);//计算关键帧1和关键帧2属于相同单词两个特征点的描述子之差

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
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))//如果最小值远远小于第二最小值
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
	 //在两个关键帧中的单词不相同则
	 //如果关键帧1的序号落后关键帧2则使关键帧1对齐关键帧2
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);//找到第一个大于或等于f2it->first
        }
        else //如果关键帧2的序号落后关键帧1则使关键帧2对齐关键帧1
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

//通过帧间词典向量加速匹配，再使用极线约束限制匹配的特征点  
//F12为当前帧和共视帧的基础矩阵，输出为vMatchedPairs，其输出的是匹配特征点在两帧图像中的序号。第一个整数是当前帧中的特征点序号，第二个整数是共视帧的特征点序号
//返回的int类型的变量是匹配的特征点
//pKF1为输入的当前关键帧，pKF2为当前帧的共视关键帧，bOnlyStereo输入为false
//vMatchedPairs输出的是内容是共视帧中的特征点序号，序号是当前帧中特征点的序号
//这里一定要注意!!!!!!!!!!如果匹配的特征点在pkf1或者pdf2中已经存在了对应的地图点则不输出到vMatchedPairs中!!!!!!!
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat C2 = R2w*Cw+t2w;//获得当前帧的中心在共视帧坐标系下的坐标
    const float invz = 1.0f/C2.at<float>(2);
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;//当前帧的中心在共视帧图像中的像素坐标
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);//内容是共视帧中的特征点序号，序号是当前帧中特征点的序号

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
        if(f1it->first == f2it->first)//属于相同的单词
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)//遍历属于当前关键帧帧这个单词中的所有特征点
            {
                const size_t idx1 = f1it->second[i1];//属于这个单词的在当前帧图像中的序号
                
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                
                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);//当前关键帧帧属于这个单词的特征点描述子
                
                int bestDist = TH_LOW;//属于相同单词与当前前帧匹配最好的特征点描述子之差
                int bestIdx2 = -1;//属于相同单词与当前前帧匹配最好的特征点在共视帧中的序号
                //遍历属于这个单词的在共视关键帧中的特征点
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
                    
                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);//共视帧属于这个单词的特征点描述子
                    
                    const int dist = DescriptorDistance(d1,d2);//计算当前帧和共视帧这两个同属于一个单词的特征点描述子的差
                    
                    if(dist>TH_LOW || dist>bestDist)//TH_LOW=50
                        continue;

                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                    if(!bStereo1 && !bStereo2)//如果当前帧和共视帧中右图像都没有这个特征点
                    {
                        const float distex = ex-kp2.pt.x;
                        const float distey = ey-kp2.pt.y;
                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }
		     		//判断当前帧的特征点和共视帧中的特征点是否满足基础矩阵规定的极限定理!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

                    if(mbCheckOrientation)//这里作者不使用旋转一致性检测
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
            }//外层for循环结束

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);//找到第一个大于或等于f2it->first
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }//while循环结束

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


// 投影地图点到当前关键帧上，在附加区域搜索匹配关键点，如果匹配上了当前帧的特征点，并判断当前帧的这个特征点是否有对应的地图点  
// 如果该点有对应的MapPoint，那么将两个MapPoint合并（用观测数多的代替观测数少的地图点）  
// 如果该点没有对应的MapPoint，那么向当前帧添加这个地图点
//th默认值是3,用于决定在图像中搜索的半径
//返回地图点和当前帧特征点融合的个数
//这是用于localmapping中融合地图点的fuse函数
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
    //Fuse函数就是这一个大的for循环函数
    //遍历输入的所有地图点
    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw*p3Dw + tcw;//根据相机在世界坐标系下的位姿和地图点在世界坐标系下的位置 得到地图点在当前帧坐标系下的位置坐标

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//得到世界坐标点投影到当前帧左图像中的像素坐标
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float ur = u-bf*invz;//得到世界坐标点投影到当前帧右图像中的像素坐标

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

	//得到图像中的候选特征点
	//返回的是在半径范围的特征点在这个图像中的特征点序号
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();//地图点的描述子

        int bestDist = 256;
        int bestIdx = -1;
	//遍历上面得到的候选特征点
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];//左图像中的候选特征点

            const int &kpLevel= kp.octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            if(pKF->mvuRight[idx]>=0)//在左图像中的特征点在右图像中也出现了
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
            else//在左图像中的特征点在右图像中没有出现
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);//候选特征点的描述子

            const int dist = DescriptorDistance(dMP,dKF);//计算地图点描述子和候选特征点描述子的差

            if(dist<bestDist)//找出最相近的候选特征点
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
         if(bestDist<=TH_LOW)//默认值是50
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);//得到候选特征点对应的地图点
            if(pMPinKF)//如果候选特征点有对应的地图点
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())//如果候选特征点对应的地图点被其他帧看到的次数大于输入地图点被看到的次数
                        pMP->Replace(pMPinKF);//用候选特征点的地图点代替输入的地图点
                    else
                        pMPinKF->Replace(pMP);//用输入的地图点代替候选特征点的地图点
                }
            }
            else//如果候选特征点没有对应的地图点
            {
                pMP->AddObservation(pKF,bestIdx);//更新当前地图点的信息，当前地图点被当前帧看到并与当前帧的特征点bestIdx对应
                pKF->AddMapPoint(pMP,bestIdx);//更新当前帧的的信息，看到了这个地图点
            }
            nFused++;//返回地图点和当前帧特征点融合的个数
        }
    }

    return nFused;
}

//pKF是调整帧，Scw是已经调整过的位姿
//vpPoints是最优闭环候选关键帧的共视帧+最优闭环候选关键帧看到的地图点，没有经过回环修正的
//vpReplacePoint为输出参数，序号是vpPoints使用的序号，内容是调整帧的地图点
//这个函数的作用是将最优闭环候选关键帧的共视帧+最优闭环候选关键帧看到的地图点 投影到 调整后位姿的当前帧的共视帧中并与其特征点匹配
//如果匹配成功并且这个特征点在这个帧中有对应的地图点，则将调整帧的地图点赋值给vpReplacePoint
//如果匹配成功但是这个特征点在这帧中没有对应的地图点，则将共视帧地图点加入到调整帧中，更新这个地图点被调整帧看到
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
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();//调整帧的地图点

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    //遍历最优闭环候选关键帧的共视帧+最优闭环候选关键帧看到的地图点，这个函数就是这么一个大的for循环
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];//最优闭环候选关键帧的共视帧+最优闭环候选关键帧看到的地图点

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        //将闭环候选关键帧和其共视帧看到的地图点投影到当前帧的共视帧上
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;//当前帧共视帧的像素坐标
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;//闭环候选关键帧和其共视帧看到的地图点 距离当前帧共视帧光心的距离
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();//闭环候选关键帧和其共视帧看到的地图点 平均法向量

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);//在调整帧图像的一定半径内寻找候选特征点

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //在候选特征点的范围内找到与闭环候选帧看到的地图点最为匹配的特征点
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
                bestIdx = idx;//调整帧图像中的特征点序号
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        //已经在调整帧图像中找到了最为匹配的特征点
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);//根据调整帧图像中的最优匹配特征点找到调整帧中对应的地图点
            if(pMPinKF)//如果和最优闭环候选帧中的地图点匹配的调整帧中的特征点有地图点
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;//使用调整帧中的地图点更新即将要输出的变量vpReplacePoint
            }
            else//如果和最优闭环候选帧中的地图点匹配的调整帧中的特征没点有地图点
            {
                pMP->AddObservation(pKF,bestIdx);//更新共视帧地图点被调整帧看到
                pKF->AddMapPoint(pMP,bestIdx);//更新调整帧看到了这个共视帧的地图点
            }
            nFused++;
        }
    }

    return nFused;
}


//第一个参数是当前关键帧；第二个参数是候选闭环关键帧；第三个参数既是输入也是输出，是当前帧和候选闭环帧匹配的地图点
//R12和t12是当前帧和候选闭环关键帧的相对位姿，最后一个参数th决定了搜索的半径
//vpMatches12中的序号是当前帧图像中特征点的序号，内容是候选闭环关键帧所对应匹配的地图点
//这个函数的作用是将当前帧的地图点投影到候选闭环关键帧中进行特征点匹配
//将候选闭环关键帧的的地图点投影到当前关键帧中进行特征点匹配，
//如果两者匹配的序号完全一致则更新vpMatches12
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

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();//当前帧的地图点
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();//闭环候选关键帧的地图点
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

   //将已经匹配的地图点在当前帧和闭环候选关键帧的地图点中标识出来
    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];//当前帧和闭环候选关键帧已经匹配的地图点
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);//序号表示当前帧地图点的序号，内容表示闭环候选关键帧中的匹配特征点序号
    vector<int> vnMatch2(N2,-1);//序号表示闭环候选关键帧地图点的序号，内容表示当前帧中的匹配特征点序号

    // Transform from KF1 to KF2 and search
    //遍历当前帧的地图点
    //将当前帧地图点投影到闭环候选关键帧的图像中进行特征匹配
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;//当前帧地图点在当前帧坐标系的位置
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;//当前帧地图点在闭环候选关键帧坐标系的位置

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;//当前帧地图点在闭环候选关键帧图像中的像素坐标
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

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);//得到闭环候选关键帧图像中候选的匹配特征点

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        //当前帧地图点的描述子
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
	//将当前帧地图点的描述子与闭环候选关键帧中的候选特征点进行逐个比较
	//将距离最近的
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
    //遍历闭环候选关键帧的地图点
    //将闭环候选关键帧地图点投影到当前帧的图像中进行特征匹配
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
        int idx2 = vnMatch1[i1];//得到当前帧地图点投影到闭环候选关键帧图像中的特征点序号

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];//得到闭环候选关键帧地图点投影到当前帧图像中的特征点序号
            if(idx1==i1)//如果两者相互投影匹配序号相互一致，则更新vpMatches12
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
 //这个函数返回的是当前帧中的特征点和上一帧地图点匹配的个数
 //双目情况下bMono=false
 //这个函数的基本思路:我们已经知道了上一帧的位姿和当前帧的一个预估位姿，并且知道上一帧中地图点，这个函数的作用就是将上一帧的地图点根据当前帧的预估位姿投影到当前帧的图像中
 //然后在当前帧投影得到的像素点周围与上一帧进行地图点的匹配，匹配成功后使用并经过旋转一致性检测筛选出上一帧不好的地图点，然后将筛选后的地图点更新当前帧的地图点
 //此原函数用于TrackWithMotionModel函数中
 //详见算法实现文档
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];//HISTO_LENGTH=30
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);//当前帧的姿态(如果用在运动模型中则当前帧的姿态是根据速度和上一帧的位姿得到的估计值)，为什么是一个4*4的矩阵???
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);//当前帧的位置

    const cv::Mat twc = -Rcw.t()*tcw;//详见算法实现文档，得到相机坐标系下的点到世界坐标系点的位置变换

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);//上一时刻帧的姿态，
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);//上一时刻帧的位置

    const cv::Mat tlc = Rlw*twc+tlw;//当前帧在上一时刻帧的位置

    //mb是双目基线长度，单位是米
    //只有向前或者向后移动的距离超过双目的基线才算移动了。
    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    //轮询上一帧中的所有地图点
    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // Project
                cv::Mat x3Dw = pMP->GetWorldPos();//得到上一帧地图点的世界位置
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//得到特征点在当前帧坐标系下的位置

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

		  //得到的点一定在相机坐标系下的深度必须是正的
                if(invzc<0)
                    continue;
	         //得到上一帧地图点在当前帧的图像中的像素坐标
                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

	         //反推得到的像素坐标要在范围之内
                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;
                 
                int nLastOctave = LastFrame.mvKeys[i].octave;

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);//搜索当前帧像素点一定半径范围内的其他特征点
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

	         //返回上一帧地图特征点的描述子
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;
                //轮询当前帧投影得到的特征点周围的所有特征点
                //得到一个和上一帧地图点最相近的特征点，并记录其在图像中的特征点序列号记为bestIdx2
                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;//根据视差公式和左侧图像的像素坐标计算得到右侧图像上的像素坐标
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);//根据视差公式得到的像素坐标和实际通过左右图像匹配的到的像素坐标不能相差太多
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

	         //如果当前帧的特征点能够较好的匹配上一帧地图点，则使用上一帧的地图点更新当前帧的地图点
                if(bestDist<=TH_HIGH)//TH_HIGH=100
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;
                          
                    if(mbCheckOrientation)//默认的是计算特征点的方向的
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;//根据这个360判断，特征点方向的单位应该是度
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
    //默认的是计算特征点的方向的
    //这个函数的目的是检查已经匹配的当前帧和上一帧的特征点是否方向一致，构造一个直方图，只保留三个最高直方图中的上一帧地图点来更新当前帧的地图点
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);//计算直方图rotHist中的最大的三个条形柱，并保存在最后三个参数中

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
 //当前帧特征点与关键帧地图点进行匹配，返回值是匹配的特征点的个数
 //这里我们已经知道了当前帧的位姿和关键帧中地图点在世界坐标系下的位置，思路是将关键帧中的地图点投影到当前帧中然后在这个像素周围根据描述子找到匹配的地图点
//如果找到了匹配，则使用特征帧的地图点更新当前帧的地图点。
 //输入的参数th表示在图像中搜索的范围，ORBdist表示与当前特征点相似的一个阈值，如果两个特征点描述子小于这个数，则认为匹配成功
//这个函数用在了Relocalization函数中
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);//当前帧在世界坐标系下的位姿
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);//当前帧在世界坐标系下的位置
    const cv::Mat Ow = -Rcw.t()*tcw;//应该是当前帧光心在世界坐标系下的坐标

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();//获取关键帧中地图点

    //轮询关键帧中的地图点
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();//地图点在世界坐标系下的位置
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//地图点在相机坐标系下的位置

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;//得到地图点在当前帧图像中的像素坐标
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

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);//第二个函数

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();//得到关键帧中地图点的描述子

                int bestDist = 256;
                int bestIdx2 = -1;

		 //轮询地图点在当前帧中坐标周围的所有特征点
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

                    if(mbCheckOrientation)//旋转一致性检测，为柱状图分配特征点
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

    if(mbCheckOrientation)//旋转一致性检测，找到三个最高柱状图的特征点
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

//找到直方图histo中的三个最大的数据并将他们对应的序号保存在最后的三个参数中。
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
//应该就是计算两个矩阵的距离，及两个描述子之间的相似度
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
