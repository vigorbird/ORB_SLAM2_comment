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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor)
    :mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file!!!!!!!!!!!!!!!!!!!!!!!!!!!

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
    //mbf=基线*fx
    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    int nFeatures = fSettings["ORBextractor.nFeatures"];//每个图像中提取orb特征的个数，作者设置的是1200
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];//尺度自金字塔的缩放参数，作者设置的是1.2
    int nLevels = fSettings["ORBextractor.nLevels"];//金字塔的层数，作者设置的是8
    //详见算法实现文档关于特征点检测部分说明
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

   //注意!!!这里作者没有使用opencv提供的orb特征提取，而是自己新建的一个类
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    //这个大的if语句 将输入的左右图像变成灰色图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

   //输入参数共10个:左右灰度图像，时间戳，左右图像特征提取，词典，内参=mk，畸变矩阵=mDistCoef，基线*FX=mbf,远近特征点阈值=mThDepth。
   //搜索"双目frame构造函数"
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();
    
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;
    //将灰色图像变为
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
    
    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

//Main tracking function. It is independent of the input sensor.
//Track函数
void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)//如果没有初始化则初始化
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();//双目初始化
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);//更新要显示的内容

        if(mState!=OK)
            return;
    }
    else//已经完成初始化
    {
        // System is initialized. Track Frame.
        bool bOK;//表示追踪是否成功

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)//tracking + local mapping + 回环检测同时工作=默认模式
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)//跟踪成功
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
		//如果刚进行完重定位或者是还没有计算出速度
		//因为这两种情况之前都没有速度可以参考，所以无法使用匀速模型
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();//使用匀速模型求解当前帧的初始位姿
                    if(!bOK)//如果匀速模型模型失败则使用关键帧模型
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else//跟踪失败，进行重定位!!!!!!!!!!!!!
            {
                bOK = Relocalization();
            }
        }
        else//只是tracking线程工作，local mapping和 回环检测都不工作
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;//更新当前帧的参考帧，等式左边的mpReferenceKF是属于帧结构的，右边的mpReferenceKF属于track结构的

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)//如果local mapping被使能 
        {
            if(bOK)
                bOK = TrackLocalMap();//这个是为了更新本地地图和关键帧的信息，并对epnp得到的初始位姿进行非线性优化
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        
        mpFrameDrawer->Update(this)//更新要显示的内容;

        // If tracking were good, check if we insert a keyframe
        //如果跟踪成功
          if(bOK)
        {
	            // Update motion model
	            if(!mLastFrame.mTcw.empty())
	            {
	                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
	                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
	                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
	                mVelocity = mCurrentFrame.mTcw*LastTwc;//计算当前帧和上一帧之间的相对位姿变换
	            }
	            else
	                mVelocity = cv::Mat();

	            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);//用于在可视化中显示相机的位姿

	            // Clean VO matches
	            for(int i=0; i<mCurrentFrame.N; i++)
	            {
	                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
	                if(pMP)
	                    if(pMP->Observations()<1)
	                    {
	                        mCurrentFrame.mvbOutlier[i] = false;
	                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
	                    }
            }

            // Delete temporal MapPoints
            //遍历上一帧中距离相机位姿最近的前100个地图点，只有这里使用到了mlpTemporalPoints这个变量
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            //比较重要的函数，判断是否需要插入关键帧!!!!!!!!!!!!!!
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            //通过tracklocalmap非线性优化可以得到一些地图点是异值点，将这些异值点去除掉
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        //如果跟踪失败
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)//跟踪失败时地图中的关键帧不超过5个
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;//等式左边的mpReferenceKF是属于帧结构的，右边的mpReferenceKF属于track结构的

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    //
    if(!mCurrentFrame.mTcw.empty())//如果当前帧的位姿已经知道，即跟踪成功
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();//当前帧与其参考帧之间的相对位姿变化
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

/*   设置第一帧为关键帧，
	第一帧的位姿为单位矩阵。	
	将第一帧加入到地图中。
	根据左右图像匹配得到的特征点生成地图点，将这个关键帧加入到这些地图点中，将这些地图点加入到这个关键帧中，更新当前帧中的地图点，并将这些地图点添加到地图中，将第一帧关键帧加入到地图中，将第一帧关键帧传给localmap线程。
	设置置track线程中参考关键帧为当前关键帧，
	当前帧的参考帧为当前关键帧。
	使用第一帧生成的地图点更新局部地图点(=track线程中局部关键帧看到的地图点)，
	更新可视化的地图点mvpReferenceMapPoints
	设置整个算法的起始帧mvpKeyFrameOrigins，在最优的全局优化中会用到。*/
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)//如果当前帧的关键点数量大于500个
    {
        // Set Frame pose to the origin
        //生成一个4*4的单位矩阵
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        //第一个帧是关键帧
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        //将关键帧加入到tracking线程的地图中
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);//得到特征点在世界坐标系下的坐标
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);//生成一个地图点
                pNewMP->AddObservation(pKFini,i);//更新这个地图点的信息，被pKFini关键帧看到，主要更新地图点中的mObservations和nobs变量
                pKFini->AddMapPoint(pNewMP,i);//更新关键帧中的地图点，即更新这个关键帧中的mvpMapPoints变量
                //观测到该地图点的多个特征点中（对应多个关键帧），挑选出最平均的的描述子，作为这个地图点的描述子
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();//更新地图点的平均法线和深度信息
                mpMap->AddMapPoint(pNewMP);//向地图中添加tracking地图点
                mCurrentFrame.mvpMapPoints[i]=pNewMP;//使用地图点更新当前帧中的地图点
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);//向local mapping中插入当前帧作为关键帧

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();//更新局部地图点
        mpReferenceKF = pKFini;//tracking线程中的参考关键帧赋值为第一个关键帧
        mCurrentFrame.mpReferenceKF = pKFini;//更新当前帧的参考关键帧是自己

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);//更新地图中可视化的点，

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);//mvpKeyFrameOrigins这个参数在双目的情况下只有一个变量，那就是第一个帧。在全局优化中会被用到

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);//更新绘图中的相机的位姿

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

//对于上一帧图像的地图点进行遍历
//如果上一帧特征点对应的地图点需要被替换则对其进行替换
//而指明哪些帧的地图点需要被替换的程序集中在SearchAndFuse函数中
void Tracking::CheckReplacedInLastFrame()
{
    
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();//返回mpReplaced就是这个地图点的替换点，这个变量只在MapPoint::Replace()函数中被更新
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

//基本思路是先将上一帧的位姿设置为当前帧的位姿的初值，然后将当前帧特征点和track线程中的参考关键帧地图点进行匹配，并使用非线性优化得到当前帧的位姿
//通过非线性优化会得到已经匹配的地图点有些是异值点则将这个地图点赋值为空，然后使用没有异值的地图点更新当前帧的地图点。
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    //根据当前帧的描述子得到当前帧的词汇结构
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    //第二个参数是普通帧
    //与距离当前时间最近的关键帧与当前帧进行词典的比较，第三个参数输出是的当前帧与关键帧中匹配的地图点
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;//用当前帧和track线程中的参考关键帧匹配的地图点来更新当前帧的地图点
    mCurrentFrame.SetPose(mLastFrame.mTcw);//设置当前帧的位姿是上一帧的位姿


    Optimizer::PoseOptimization(&mCurrentFrame);//!!!!!!!!!!!!!关键的函数，会得到哪些地图点是异值点

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])//这个地图点通过优化发现是异值点
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);//将当前帧的这个地图点赋值为空
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)//如果这个地图点也被其他关键帧观测到了
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

//根据参考帧的位姿，得到上一帧的绝对位姿，使用距离上一帧前100的特征点生成地图点并更新上一帧的地图点和mlpTemporalPoints变量
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());//根据参考帧的位姿，得到上一个帧的绝对位姿

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)//如果上一帧是关键帧则直接返回
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    //我们对上一帧中所有特征点按照其深度进行排序
    vector<pair<float,int> > vDepthIdx;//存储的是特征点的深度和在上一帧中特征点的序号
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    //使用前100个距离相机位姿近的地图点更新mLastFrame.mvpMapPoints的地图点和mlpTemporalPoints
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);//特征点在世界坐标系下的位置
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);//这里生成的地图点是tracking线程中的

            mLastFrame.mvpMapPoints[i]=pNewMP;//更新上一帧的地图点

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

//返回的bool值为是否当前帧的特征点与10个地图点进行了匹配并且还是有效的非线性优化边
//基本思路是首先根据上一帧的特征点更新上一帧的地图点和上一帧的位姿，并使用上一帧的位姿和速度预估得到当前帧的位姿。然后将上一帧的地图点与当前帧特征点进行匹配，如果匹配上则使用上一帧的地图点更新当前帧的地图点
//然后再使用非线性优化得到精确的当前帧的位姿。非线性优化之后会得到某些地图点是异值点，需要把当前帧的异值地图点设置为空。
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

     //根据上个时刻的位置和速度(这里的速度其实就是上一次帧的位移)估计得到当前帧的位置
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    //std::fill函数,第一个参数 为容器的首迭代器，第二个参数为容器的末迭代器，最后一个参数为将要替换的值。
    //就是将当前帧的mvpMapPoints清零
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    //返回的是当前帧匹配前一帧的特征点个数
    //
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);//System::MONOCULAR=0

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    //将当前帧地图点投影到相机坐标系下的相机平面。优化变量只有pose，地图点位置固定，是一边元。
    //并在优化的过程中进行了四次优化，使用了一定的小技巧判断哪些边是有效的哪些是无效的。
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    //
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)//只运行tracking模式么?
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

//使用本地地图的信息得到当前帧更加精确的位姿，然后更新当前帧的地图点
bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();//!!!!!!!!!!!!!!!!跟新局部地图点和局部关键帧

    SearchLocalPoints();//!!!!!!!!!!!!!!!!!!将局部地图点与当前帧的特征点进行匹配，如果匹配上则将这个地图点加入到当前帧中
    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);//最小化投影误差优化位姿
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    //遍历当前帧中的特征点，剔除mCurrentFrame.mvpMapPoints中的outlier
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])//mvbOutlier是通过上面的非线性优化设置的，表示当前帧的这个地图点不是异值点
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();//mnFound+=n;
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //如果当前帧距离上一次的重定位太近 并且 inliner<50 则认为跟踪失败
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)//mMaxFrames=fps
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

//
bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    //tracking线程和localmapping线程交互-读localmapping线程的状态
    //如果localmapping线程已经停止了或者
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();//地图中关键帧的个数

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    //从上次重定位还没有过去许多帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);//返回参考关键帧中高质量地图点的数量，高质量地图点=被至少minObs个关键帧观测到
    // Local Mapping accept keyframes?
    //tracking线程和localmapping线程交互-读localmapping线程的状态
    //向local mapping线程询问是否可以接收关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;//表示当前帧一共有多少个近特征点
                else
                    nNonTrackedClose++;
            }
        }
    }
	
    //参看orb2文章3-E章节。如果当前帧中没有足够的近特征点则需要创建新的近特征点
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //与上一次的关键帧要超过fps
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //如果localmap线程空闲
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    //是双目并且(需要插入近特征点或者匹配的地图点小于当前帧的参考关键帧的高质量的地图的四分之一)
    const bool c1c =  (mSensor!=System::MONOCULAR) && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    //这是一个必须要满足的条件:匹配的地图点大于15 并且 匹配的地图点小于参考关键帧中高质量地图点的四分之三或者没有足够的近特征点  
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else//如果localmapping不空闲=无法接受关键帧
        {
            //tracking线程和localmapping线程交互-设置localmapping线程的状态
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)//双目情况下进入这个条件
            {
            	//tracking线程和localmapping线程交互-读localmapping线程的状态
                if(mpLocalMapper->KeyframesInQueue()<3)//在localmapping队列中的关键帧小于3
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

//主要是向localmap线程加入当前帧为关键帧。
//遍历当前帧的双目匹配成功的特征点，
/*如果还没有对应的地图点则对其初始化为tracking的地点，然后更新这个地图点被当前关键帧观测到，将这些地图点加入到当前帧中。
  如果有对应的地图点则什么都不做
向localmap线程中加入当前帧为关键帧。对于关键帧中的地图点只保留距离前100个近特征点(<mThDepth)，其余的删除。*/
void Tracking::CreateNewKeyFrame()
{
	//tracking线程和localmapping线程交互-设置localmapping线程状态
	//这个应该是让localmapping线程不要停止运行，如果设置失败表示localmapping已经停止运行了
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);//新建的关键帧

    mpReferenceKF = pKF;//设置track线程的参考关键帧是当前的关键帧
    mCurrentFrame.mpReferenceKF = pKF;//设置当前帧的参考帧就是它自己(也就是说所有关键帧的参考关键帧都是自己)

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();//更新当前帧的位姿

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        //下面这个程序好像之前出现过。也是对特征点按照距离相机的大小进行排序然后选前100个，然后更新当前帧中的mvpMapPoints
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)//判断当前帧这个特征点是否已经是地图点，如果没有对应的地图点就新建地图点
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);//新建地图点，这里生成的新地图点是tracking中的
                    pNewMP->AddObservation(pKF,i);//更新这个地图点被当前关键帧观测到了
                    pKF->AddMapPoint(pNewMP,i);//当前关键帧观测到了这个地图点
                    pNewMP->ComputeDistinctiveDescriptors();//综合地图的信息 更新这个地图点的描述子
                    pNewMP->UpdateNormalAndDepth();//综合地图的信息，更新这个地图点的法向
                    mpMap->AddMapPoint(pNewMP);//向地图添加这个地图点

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;//将地图点加入到当前帧中
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }
    //tracking线程和localmapping线程交互-设置localmapping线程状态
    mpLocalMapper->InsertKeyFrame(pKF);//关键的一句话，向localmapping中插入新建的关键帧

    mpLocalMapper->SetNotStop(false);//允许让localmapping线程停止运行了

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

//这个函数有三个个作用第一个是更新通过参考帧模型或者速度模型得到的当前帧的地图点被观察到的次数。
//第二个是判断(局部地图点-上一帧匹配的地图点)是否在当前帧的视野内，如果在当前帧的视野内则更新这个局部地图点被真实观察到的次数
//第三个个作用是SearchByProjection，将(局部地图点-上一帧匹配的地图点)投影到当前帧上，并与当前帧的特征点进行匹配如果成功则将这个地图点加入到当前帧中
//这个函数作用其实就是检验局部地图点中是否还有漏网之鱼与当前帧匹配
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //遍历当前帧中已经与地图点匹配的特征点
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;//这里表示与上一帧地图点匹配得到的当前帧的地图点跟踪
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    //遍历局部地图中的地图点。判断是否在当前帧的视野范围内，如果在视野范围内则增加该地图点被观察到的次数
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        //判断地图点是否在当前帧的视野中，一定要注意这里的地图点一定不是上面已经匹配的地图点!!!!!!!!!!!!!!!
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
	//将局部地图点投影到当前帧中并与当前帧的特征点进行匹配!!!!!!!!!!!!!!!!!
	//参数th是在特征点进行搜索时的半径，如果th=1则法向量决定搜索半径，如果th!=1，则th的值就是搜索半径
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();//更新局部关键帧
    UpdateLocalPoints();//更新局部地图点
}

//更新局部地图点mvpLocalMapPoints=局部关键帧的地图点
void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();
   //轮询局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();//返回的是mvpMapPoints
        //轮询本地关键帧中的地图点
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)//防止添加重复
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);//将局部关键帧的地图点插入到mvpLocalMapPoints中
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

//更新局部关键帧mvpLocakKeyFrames=当前帧的所有地图点的共视关键帧+共视关键帧的父子帧+共视帧的前10名的共视帧，
//并设置与当前帧共视最多的关键帧设置为当前帧的参考帧和track线程的参考帧
void Tracking::UpdateLocalKeyFrames()
{ 
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;//存储其他关键帧和当前帧的共视次数
    //遍历当前帧的所有地图点找到共视这些地图点的其他关键帧
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])//此处的当前帧的地图点，是通过与track线程参考关键帧匹配得到的
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
		  //遍历其他关键帧中与当前帧特征点共视的所有特征点
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    //找到与当前帧共视最多的那个关键帧保存在pKFmax中。并将所有与当前帧共视的关键帧保存在mvpLocalKeyFrames中。
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    //遍历与当前帧共视的所有关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);//得到与共视关键帧共视程度最高的前十个关键帧
        //将共视关键帧相连程度最高的前10个关键帧也保存在mvpLocalKeyFrames中。
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        //将共视关键帧的子关键帧和父关键帧保存在mvpLocalKeyFrames中
        const set<KeyFrame*> spChilds = pKF->GetChilds();//目前还不清楚这个父节点和子节点是什么
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

   //与当前帧共视最多的关键帧设置为当前帧的参考关键帧和track线程的参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

//首先计算当前帧的bow，然后使用bow加速在所有的关键帧中寻找重定位候选帧，然后将当前帧特征点和所有的重定位候选帧地图点进行bow匹配，下面就是pnp问题了
//然后使用ransac+epnp求解相机的初始位姿，并剔除掉异值的地图点
//再使用剔除掉异值点进行非线性优化得到最终的位姿,通过非线性优化会对地图点再进行一次筛选排除异值地图点，然后使用再次排除过异值的地图点加入到当前帧中。
//如果非线性优化的有效边较少则放宽匹配条件，再进行一次匹配。而此时进行匹配时不只是使用bow匹配而是使用当前帧的位姿，将关键帧的地图点投影到当前帧中进行匹配。并再进行一次非线性优化。
//非线性优化会对地图点再进行一次筛选排除异值地图点，然后使用再次排除过异值的地图点加入到当前帧中。
//epnp部分使用的是opencv中的代码
bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();
	

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    //轮询候选关键帧
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);//将候选关键帧的地图点和当前帧进行匹配
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                //已知世界坐标系下空间点位置和图像对应像素坐标求世界坐标系下的相机位姿 
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    //只要匹配到就立刻退出
    
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)//轮询候选关键帧
        {
            if(vbDiscarded[i])//如果匹配的特征点太少则不计算
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);//!!!!!!!!!!!!!关键函数求pnp

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();
                
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);//!!!!!!!!!!!!使用非线性优化求当前帧的位姿

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                //如果非线性优化得到的有效边比较少则再进行一次当前帧的特征点和关键帧的地图点匹配，然后再使用一次非线性优化
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);//第二个函数

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)//如果再进行一次非线性优化后发现还是没有足够的有效边则将匹配条件放宽
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);//匹配条件放宽，第二个函数

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;//记录下重定位的关键帧id，也就是当前帧
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
	//tracking线程和localmapping线程交互-设置localmapping状态
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
