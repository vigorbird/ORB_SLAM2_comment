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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;//����������ڵ��������
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;//��ʾ����ڵ��Ƿ���Ҫ�����ٴλ���
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;//�洢���ǲ�ͬ��������е�ͼ�������ͼ�񲻰����߽�

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;//�洢����ʵ����bit_pattern_31_�����еĵ�ԣ�����brief�����ӡ�bit_pattern_31_��256����ԣ���pattern�о���512���㡣
    int nfeatures;//�������õ���1200
    double scaleFactor;
    int nlevels;//�������Ĳ���
    int iniThFAST;//�������õ���20
    int minThFAST;//�������õ���7

    std::vector<int> mnFeaturesPerLevel;//�洢����ÿ���������Ҫ��������������ĸ���,�������ķ�����opencv��ͬ

    std::vector<int> umax;//ÿ��������Բ�����λ����Ϣ  

    std::vector<float> mvScaleFactor;//ÿ�������������ű���,����˵ͼ��Ҫ��С2��һ��4�㣬�򱣴�ľ���[1,2,4,8]
    std::vector<float> mvInvScaleFactor;  //mvScaleFactor�ĵ���,�������[1,1/2,1/4,1/8]
    std::vector<float> mvLevelSigma2;//��mvScaleFactor��ƽ��,�������[1,2*2,4*4,8*8]
    std::vector<float> mvInvLevelSigma2;//��mvLevelSigma2�ĵ������������[1,1/(2*2),1/(4*4),1/(8*8)]
};

} //namespace ORB_SLAM

#endif

