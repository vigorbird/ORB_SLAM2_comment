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


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

class Sim3Solver
{
public:

    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    cv::Mat GetEstimatedRotation();
    cv::Mat GetEstimatedTranslation();
    float GetEstimatedScale();


protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    void CheckInliers();

    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;

    std::vector<cv::Mat> mvX3Dc1;//当前帧地图点在当前帧坐标系下的坐标
    std::vector<cv::Mat> mvX3Dc2;//当前帧和闭环候选关键帧已经匹配地图点在闭环候选关键帧坐标系下的坐标
    std::vector<MapPoint*> mvpMapPoints1;//当前帧的地图点
    std::vector<MapPoint*> mvpMapPoints2;//当前帧和闭环候选关键帧已经匹配的地图点
    std::vector<MapPoint*> mvpMatches12;//当前帧和闭环候选关键帧已经匹配的地图点
    std::vector<size_t> mvnIndices1;//里面存储的是已经匹配地图点的序号
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    std::vector<size_t> mvnMaxError1;//用于判定样本是否为inliner，对应ransac参数中的t，但是这个参数根据特征点所在金字塔层数不同，其阈值也不同
    std::vector<size_t> mvnMaxError2;

    int N;
    int mN1;

    // Current Estimation
    cv::Mat mR12i;//闭环候选帧相对当前帧的旋转矩阵
    cv::Mat mt12i;//闭环候选帧相对当前帧的位移矩阵
    float ms12i;//尺度
    cv::Mat mT12i;//闭环候选帧相对当前帧的位姿变换矩阵
    cv::Mat mT21i;//当前帧相对于闭环候选帧的位姿变换矩阵
    std::vector<bool> mvbInliersi;//标识哪些地图点是inliner，序号对应地图点的序号，值对应是否为inliner
    int mnInliersi;//对应ransac中的参数d

    // Current Ransac State
    int mnIterations;//当前ransac算法迭代的总次数
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;//当前使得总样本为最多inliner的个数
    cv::Mat mBestT12;
    cv::Mat mBestRotation;//经过ransac过程后两帧相对最优旋转矩阵
    cv::Mat mBestTranslation;//经过ransac过程后两帧相对最优位移矩阵
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    std::vector<size_t> mvAllIndices;//里面存储的是已经匹配地图点的序号

    // Projections
    std::vector<cv::Mat> mvP1im1;//当前帧地图点在当前帧图像中的像素坐标
    std::vector<cv::Mat> mvP2im2;//已经匹配的地图点在闭环候选关键帧图像中的像素坐标

    // RANSAC probability
    //对应ransac参数p=0.99
    double mRansacProb;

    // RANSAC min inliers
    //对应ransac所有样本中inliner可能的数量=20
    int mRansacMinInliers;

    // RANSAC max iterations
    //对应ransac参数k=300
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    cv::Mat mK1;
    cv::Mat mK2;

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
