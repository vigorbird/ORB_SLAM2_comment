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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{


//pKF1表示当前帧；pKF2表示闭环候选帧；vpMatched12表示当前帧和闭环候选帧所匹配的地图点；
//此函数的主要作用是更新Sim3Solver中的参数
Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();//当前帧的地图点

    mN1 = vpMatched12.size();//当前帧和闭环候选关键帧匹配的地图点数

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    cv::Mat Rcw1 = pKF1->GetRotation();//世界坐标系下当前帧的旋转矩阵
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();//世界坐标系下的闭环候选关键帧的旋转矩阵
    cv::Mat tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    //遍历已经匹配的地图点数
    size_t idx=0;
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            MapPoint* pMP1 = vpKeyFrameMP1[i1];//当前帧的地图点
            MapPoint* pMP2 = vpMatched12[i1];//当前帧和闭环候选关键帧已经匹配的地图点

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);//当前帧地图点在当前帧中的序号
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);//当前帧和闭环候选关键帧已经匹配的地图点在闭环候选关键帧中的序号

            if(indexKF1<0 || indexKF2<0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            mvnMaxError1.push_back(9.210*sigmaSquare1);//这个参数用于判定样本点是否为inliner。
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            cv::Mat X3D1w = pMP1->GetWorldPos();//当前帧地图点的世界坐标
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);//使用当前帧地图点在当前帧坐标系下的坐标，更新mvX3Dc1

            cv::Mat X3D2w = pMP2->GetWorldPos();//	当前帧和闭环候选关键帧已经匹配的地图点在闭环候选关键帧坐标系下的坐标
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);//使用已经匹配地图点在闭环候选关键帧坐标系下的坐标更新mvX3Dc2

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mK1 = pKF1->mK;//当前帧的摄像机矩阵K
    mK2 = pKF2->mK;//闭环候选关键帧的摄像机矩阵K

    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);//输出是mvP1im1，当前帧地图点在当前帧图像中的像素坐标
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);//输出是mvP2im2，已经匹配的地图点在闭环候选关键帧图像中的像素坐标

    SetRansacParameters();
}

//输入的参数probability表示ransac运行k次后成功的概率=0.99
//maxIterations表示ransac算法允许迭代运行的最高次数=300
//minInliers表示所有样本中inliner可能的数量=20
void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;//对应的是ransac中的参数p，算法运行k次后成功的概率
    mRansacMinInliers = minInliers;//对应的是ransac中所有样本中是inliner的数量
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences，对应ransac参数中所有样本的数量

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;//对应的参数是ransac中的参数w

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));//对应的是ransac中的参数k，算法需要迭代多少次才能找到最优解

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));//通过公式计算得到的迭代次数nIterations如果大于我们认为规定的次数maxIterations，则我们使用人为规定的迭代次数

    mnIterations = 0;
}

//就第一个参数是输入，其他的三个参数都是输出
//vbInliers返回的是最优模型对应的inliner，序号表示对应地图点的序号，内容表示是否为inliner
//nInliers返回的是最大inliner的个数
//函数返回的值是候选闭环帧到当前帧的相对位姿变换
cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F); 
	

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;//当前帧和闭环候选帧已经匹配地图点的序号

        // Get min set of points
        //最少需要三组匹配点才能计算得到相对位姿变换
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));//当前帧的地图点在当前帧坐标系下的坐标
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));//当前帧和闭环候选关键帧已经匹配地图点在闭环候选关键帧坐标系下的坐标

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);//根据三对地图点在两个坐标下的坐标，计算相对位姿(ICP)!!!!!!!!!!!!!!!!!!!!!!!!!重要函数

        CheckInliers();//详见算法实现文档

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;//这个是计算成功后的返回值
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}

//根据三对图点在两个坐标系下的坐标更新了Sim3Solver类中的如下四个变量 
//cv::Mat mR12i; 
//cv::Mat mt12i;
//float ms12i;
//cv::Mat mT12i;
//cv::Mat mT21i;
//输入参数是:P1=当前帧的地图点在当前帧坐标系下的坐标,按照列存储坐标(3*3矩阵)，相当于论文中的右坐标系
//P2=当前帧和闭环候选关键帧已经匹配地图点在闭环候选关键帧坐标系下的坐标(3*3矩阵)，相当于论文中的左坐标系
//注意本作者并没有使用论文中三个点的特殊情况(对应文章的第五章节)来求解，还是使用的普世的方法来解的。
  void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);//计算P1的几何中心O1,Pr1是P1的去位移坐标
    ComputeCentroid(P2,Pr2,O2);//计算P2的几何中心O1,Pr2是P2的去位移坐标

    // Step 2: Compute M matrix

    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;//矩阵N的特征值和特征向量

    //计算矩阵N的特征值和特征向量，特征值保存在eval，特征向量保存在evec
    //其中特征值按照从大到小的顺序排列
    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    cv::Mat vec(1,3,evec.type());//这个是轴角表示向量
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

    mR12i.create(3,3,P1.type());//mR12i是求得的R

    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2

    cv::Mat P3 = mR12i*Pr2;//这个参数是用在求尺度时

    // Step 6: Scale
   
    if(!mbFixScale)
    {
        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation

    mt12i.create(1,3,P1.type());
    mt12i = O1 - ms12i*mR12i*O2;//求得的t

    // Step 8: Transformation

    // Step 8.1 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    // Step 8.2 T21

    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}


//详见算法实现文档
void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,mK1);//将候选闭环关键帧坐标系下的地图点根据相对位姿变换，投影到当前帧的图像中得到像素坐标vP2im1
    Project(mvX3Dc1,vP1im2,mT21i,mK2);//将当前帧坐标系下的地图点根据相对位姿变换，投影到候选闭环关键帧的图像中得到像素坐标vP2im2

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];//当前帧地图点在当前帧图像中的坐标-通过相对变换得到的像素坐标
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}


cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

//已经知道两个帧之间的相对位姿变换Tcw,并且知道地图点在某个帧坐标系下的坐标vP3Dw，求得地图点在另一帧图像中的图像坐标vP2D
void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

//将摄像机坐标系下的坐标点vP3Dc通过摄像机矩阵K得到在图像中的像素坐标vP2D
void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
