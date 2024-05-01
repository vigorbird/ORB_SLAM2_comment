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
    //mbf=����*fx
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

    int nFeatures = fSettings["ORBextractor.nFeatures"];//ÿ��ͼ������ȡorb�����ĸ������������õ���1200
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];//�߶��Խ����������Ų������������õ���1.2
    int nLevels = fSettings["ORBextractor.nLevels"];//�������Ĳ������������õ���8
    //����㷨ʵ���ĵ������������ⲿ��˵��
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

   //ע��!!!��������û��ʹ��opencv�ṩ��orb������ȡ�������Լ��½���һ����
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
    //������if��� �����������ͼ���ɻ�ɫͼ��
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

   //���������10��:���һҶ�ͼ��ʱ���������ͼ��������ȡ���ʵ䣬�ڲ�=mk���������=mDistCoef������*FX=mbf,Զ����������ֵ=mThDepth��
   //����"˫Ŀframe���캯��"
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();
    
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;
    //����ɫͼ���Ϊ
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
//Track����
void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)//���û�г�ʼ�����ʼ��
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();//˫Ŀ��ʼ��
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);//����Ҫ��ʾ������

        if(mState!=OK)
            return;
    }
    else//�Ѿ���ɳ�ʼ��
    {
        // System is initialized. Track Frame.
        bool bOK;//��ʾ׷���Ƿ�ɹ�

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)//tracking + local mapping + �ػ����ͬʱ����=Ĭ��ģʽ
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)//���ٳɹ�
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
		//����ս������ض�λ�����ǻ�û�м�����ٶ�
		//��Ϊ���������֮ǰ��û���ٶȿ��Բο��������޷�ʹ������ģ��
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();//ʹ������ģ����⵱ǰ֡�ĳ�ʼλ��
                    if(!bOK)//�������ģ��ģ��ʧ����ʹ�ùؼ�֡ģ��
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else//����ʧ�ܣ������ض�λ!!!!!!!!!!!!!
            {
                bOK = Relocalization();
            }
        }
        else//ֻ��tracking�̹߳�����local mapping�� �ػ���ⶼ������
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

        mCurrentFrame.mpReferenceKF = mpReferenceKF;//���µ�ǰ֡�Ĳο�֡����ʽ��ߵ�mpReferenceKF������֡�ṹ�ģ��ұߵ�mpReferenceKF����track�ṹ��

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)//���local mapping��ʹ�� 
        {
            if(bOK)
                bOK = TrackLocalMap();//�����Ϊ�˸��±��ص�ͼ�͹ؼ�֡����Ϣ������epnp�õ��ĳ�ʼλ�˽��з������Ż�
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
        
        mpFrameDrawer->Update(this)//����Ҫ��ʾ������;

        // If tracking were good, check if we insert a keyframe
        //������ٳɹ�
          if(bOK)
        {
	            // Update motion model
	            if(!mLastFrame.mTcw.empty())
	            {
	                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
	                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
	                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
	                mVelocity = mCurrentFrame.mTcw*LastTwc;//���㵱ǰ֡����һ֮֡������λ�˱任
	            }
	            else
	                mVelocity = cv::Mat();

	            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);//�����ڿ��ӻ�����ʾ�����λ��

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
            //������һ֡�о������λ�������ǰ100����ͼ�㣬ֻ������ʹ�õ���mlpTemporalPoints�������
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            //�Ƚ���Ҫ�ĺ������ж��Ƿ���Ҫ����ؼ�֡!!!!!!!!!!!!!!
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            //ͨ��tracklocalmap�������Ż����Եõ�һЩ��ͼ������ֵ�㣬����Щ��ֵ��ȥ����
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        //�������ʧ��
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)//����ʧ��ʱ��ͼ�еĹؼ�֡������5��
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;//��ʽ��ߵ�mpReferenceKF������֡�ṹ�ģ��ұߵ�mpReferenceKF����track�ṹ��

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    //
    if(!mCurrentFrame.mTcw.empty())//�����ǰ֡��λ���Ѿ�֪���������ٳɹ�
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();//��ǰ֡����ο�֮֡������λ�˱仯
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

/*   ���õ�һ֡Ϊ�ؼ�֡��
	��һ֡��λ��Ϊ��λ����	
	����һ֡���뵽��ͼ�С�
	��������ͼ��ƥ��õ������������ɵ�ͼ�㣬������ؼ�֡���뵽��Щ��ͼ���У�����Щ��ͼ����뵽����ؼ�֡�У����µ�ǰ֡�еĵ�ͼ�㣬������Щ��ͼ����ӵ���ͼ�У�����һ֡�ؼ�֡���뵽��ͼ�У�����һ֡�ؼ�֡����localmap�̡߳�
	������track�߳��вο��ؼ�֡Ϊ��ǰ�ؼ�֡��
	��ǰ֡�Ĳο�֡Ϊ��ǰ�ؼ�֡��
	ʹ�õ�һ֡���ɵĵ�ͼ����¾ֲ���ͼ��(=track�߳��оֲ��ؼ�֡�����ĵ�ͼ��)��
	���¿��ӻ��ĵ�ͼ��mvpReferenceMapPoints
	���������㷨����ʼ֡mvpKeyFrameOrigins�������ŵ�ȫ���Ż��л��õ���*/
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)//�����ǰ֡�Ĺؼ�����������500��
    {
        // Set Frame pose to the origin
        //����һ��4*4�ĵ�λ����
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        //��һ��֡�ǹؼ�֡
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        //���ؼ�֡���뵽tracking�̵߳ĵ�ͼ��
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);//�õ�����������������ϵ�µ�����
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);//����һ����ͼ��
                pNewMP->AddObservation(pKFini,i);//���������ͼ�����Ϣ����pKFini�ؼ�֡��������Ҫ���µ�ͼ���е�mObservations��nobs����
                pKFini->AddMapPoint(pNewMP,i);//���¹ؼ�֡�еĵ�ͼ�㣬����������ؼ�֡�е�mvpMapPoints����
                //�۲⵽�õ�ͼ��Ķ���������У���Ӧ����ؼ�֡������ѡ����ƽ���ĵ������ӣ���Ϊ�����ͼ���������
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();//���µ�ͼ���ƽ�����ߺ������Ϣ
                mpMap->AddMapPoint(pNewMP);//���ͼ�����tracking��ͼ��
                mCurrentFrame.mvpMapPoints[i]=pNewMP;//ʹ�õ�ͼ����µ�ǰ֡�еĵ�ͼ��
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);//��local mapping�в��뵱ǰ֡��Ϊ�ؼ�֡

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();//���¾ֲ���ͼ��
        mpReferenceKF = pKFini;//tracking�߳��еĲο��ؼ�֡��ֵΪ��һ���ؼ�֡
        mCurrentFrame.mpReferenceKF = pKFini;//���µ�ǰ֡�Ĳο��ؼ�֡���Լ�

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);//���µ�ͼ�п��ӻ��ĵ㣬

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);//mvpKeyFrameOrigins���������˫Ŀ�������ֻ��һ���������Ǿ��ǵ�һ��֡����ȫ���Ż��лᱻ�õ�

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);//���»�ͼ�е������λ��

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

//������һ֡ͼ��ĵ�ͼ����б���
//�����һ֡�������Ӧ�ĵ�ͼ����Ҫ���滻���������滻
//��ָ����Щ֡�ĵ�ͼ����Ҫ���滻�ĳ�������SearchAndFuse������
void Tracking::CheckReplacedInLastFrame()
{
    
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();//����mpReplaced���������ͼ����滻�㣬�������ֻ��MapPoint::Replace()�����б�����
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

//����˼·���Ƚ���һ֡��λ������Ϊ��ǰ֡��λ�˵ĳ�ֵ��Ȼ�󽫵�ǰ֡�������track�߳��еĲο��ؼ�֡��ͼ�����ƥ�䣬��ʹ�÷������Ż��õ���ǰ֡��λ��
//ͨ���������Ż���õ��Ѿ�ƥ��ĵ�ͼ����Щ����ֵ���������ͼ�㸳ֵΪ�գ�Ȼ��ʹ��û����ֵ�ĵ�ͼ����µ�ǰ֡�ĵ�ͼ�㡣
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    //���ݵ�ǰ֡�������ӵõ���ǰ֡�Ĵʻ�ṹ
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    //�ڶ�����������ͨ֡
    //����뵱ǰʱ������Ĺؼ�֡�뵱ǰ֡���дʵ�ıȽϣ���������������ǵĵ�ǰ֡��ؼ�֡��ƥ��ĵ�ͼ��
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;//�õ�ǰ֡��track�߳��еĲο��ؼ�֡ƥ��ĵ�ͼ�������µ�ǰ֡�ĵ�ͼ��
    mCurrentFrame.SetPose(mLastFrame.mTcw);//���õ�ǰ֡��λ������һ֡��λ��


    Optimizer::PoseOptimization(&mCurrentFrame);//!!!!!!!!!!!!!�ؼ��ĺ�������õ���Щ��ͼ������ֵ��

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])//�����ͼ��ͨ���Ż���������ֵ��
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);//����ǰ֡�������ͼ�㸳ֵΪ��
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)//��������ͼ��Ҳ�������ؼ�֡�۲⵽��
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

//���ݲο�֡��λ�ˣ��õ���һ֡�ľ���λ�ˣ�ʹ�þ�����һ֡ǰ100�����������ɵ�ͼ�㲢������һ֡�ĵ�ͼ���mlpTemporalPoints����
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());//���ݲο�֡��λ�ˣ��õ���һ��֡�ľ���λ��

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)//�����һ֡�ǹؼ�֡��ֱ�ӷ���
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    //���Ƕ���һ֡�����������㰴������Ƚ�������
    vector<pair<float,int> > vDepthIdx;//�洢�������������Ⱥ�����һ֡������������
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
    //ʹ��ǰ100���������λ�˽��ĵ�ͼ�����mLastFrame.mvpMapPoints�ĵ�ͼ���mlpTemporalPoints
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
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);//����������������ϵ�µ�λ��
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);//�������ɵĵ�ͼ����tracking�߳��е�

            mLastFrame.mvpMapPoints[i]=pNewMP;//������һ֡�ĵ�ͼ��

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

//���ص�boolֵΪ�Ƿ�ǰ֡����������10����ͼ�������ƥ�䲢�һ�����Ч�ķ������Ż���
//����˼·�����ȸ�����һ֡�������������һ֡�ĵ�ͼ�����һ֡��λ�ˣ���ʹ����һ֡��λ�˺��ٶ�Ԥ���õ���ǰ֡��λ�ˡ�Ȼ����һ֡�ĵ�ͼ���뵱ǰ֡���������ƥ�䣬���ƥ������ʹ����һ֡�ĵ�ͼ����µ�ǰ֡�ĵ�ͼ��
//Ȼ����ʹ�÷������Ż��õ���ȷ�ĵ�ǰ֡��λ�ˡ��������Ż�֮���õ�ĳЩ��ͼ������ֵ�㣬��Ҫ�ѵ�ǰ֡����ֵ��ͼ������Ϊ�ա�
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

     //�����ϸ�ʱ�̵�λ�ú��ٶ�(������ٶ���ʵ������һ��֡��λ��)���Ƶõ���ǰ֡��λ��
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    //std::fill����,��һ������ Ϊ�������׵��������ڶ�������Ϊ������ĩ�����������һ������Ϊ��Ҫ�滻��ֵ��
    //���ǽ���ǰ֡��mvpMapPoints����
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    //���ص��ǵ�ǰ֡ƥ��ǰһ֡�����������
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
    //����ǰ֡��ͼ��ͶӰ���������ϵ�µ����ƽ�档�Ż�����ֻ��pose����ͼ��λ�ù̶�����һ��Ԫ��
    //�����Ż��Ĺ����н������Ĵ��Ż���ʹ����һ����С�����ж���Щ������Ч����Щ����Ч�ġ�
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

    if(mbOnlyTracking)//ֻ����trackingģʽô?
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

//ʹ�ñ��ص�ͼ����Ϣ�õ���ǰ֡���Ӿ�ȷ��λ�ˣ�Ȼ����µ�ǰ֡�ĵ�ͼ��
bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();//!!!!!!!!!!!!!!!!���¾ֲ���ͼ��;ֲ��ؼ�֡

    SearchLocalPoints();//!!!!!!!!!!!!!!!!!!���ֲ���ͼ���뵱ǰ֡�����������ƥ�䣬���ƥ�����������ͼ����뵽��ǰ֡��
    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);//��С��ͶӰ����Ż�λ��
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    //������ǰ֡�е������㣬�޳�mCurrentFrame.mvpMapPoints�е�outlier
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])//mvbOutlier��ͨ������ķ������Ż����õģ���ʾ��ǰ֡�������ͼ�㲻����ֵ��
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
    //�����ǰ֡������һ�ε��ض�λ̫�� ���� inliner<50 ����Ϊ����ʧ��
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
    //tracking�̺߳�localmapping�߳̽���-��localmapping�̵߳�״̬
    //���localmapping�߳��Ѿ�ֹͣ�˻���
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();//��ͼ�йؼ�֡�ĸ���

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    //���ϴ��ض�λ��û�й�ȥ���֡
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);//���زο��ؼ�֡�и�������ͼ�����������������ͼ��=������minObs���ؼ�֡�۲⵽
    // Local Mapping accept keyframes?
    //tracking�̺߳�localmapping�߳̽���-��localmapping�̵߳�״̬
    //��local mapping�߳�ѯ���Ƿ���Խ��չؼ�֡
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
                    nTrackedClose++;//��ʾ��ǰ֡һ���ж��ٸ���������
                else
                    nNonTrackedClose++;
            }
        }
    }
	
    //�ο�orb2����3-E�½ڡ������ǰ֡��û���㹻�Ľ�����������Ҫ�����µĽ�������
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);
    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //����һ�εĹؼ�֡Ҫ����fps
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //���localmap�߳̿���
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    //��˫Ŀ����(��Ҫ��������������ƥ��ĵ�ͼ��С�ڵ�ǰ֡�Ĳο��ؼ�֡�ĸ������ĵ�ͼ���ķ�֮һ)
    const bool c1c =  (mSensor!=System::MONOCULAR) && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    //����һ������Ҫ���������:ƥ��ĵ�ͼ�����15 ���� ƥ��ĵ�ͼ��С�ڲο��ؼ�֡�и�������ͼ����ķ�֮������û���㹻�Ľ�������  
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else//���localmapping������=�޷����ܹؼ�֡
        {
            //tracking�̺߳�localmapping�߳̽���-����localmapping�̵߳�״̬
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)//˫Ŀ����½����������
            {
            	//tracking�̺߳�localmapping�߳̽���-��localmapping�̵߳�״̬
                if(mpLocalMapper->KeyframesInQueue()<3)//��localmapping�����еĹؼ�֡С��3
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

//��Ҫ����localmap�̼߳��뵱ǰ֡Ϊ�ؼ�֡��
//������ǰ֡��˫Ŀƥ��ɹ��������㣬
/*�����û�ж�Ӧ�ĵ�ͼ��������ʼ��Ϊtracking�ĵص㣬Ȼ����������ͼ�㱻��ǰ�ؼ�֡�۲⵽������Щ��ͼ����뵽��ǰ֡�С�
  ����ж�Ӧ�ĵ�ͼ����ʲô������
��localmap�߳��м��뵱ǰ֡Ϊ�ؼ�֡�����ڹؼ�֡�еĵ�ͼ��ֻ��������ǰ100����������(<mThDepth)�������ɾ����*/
void Tracking::CreateNewKeyFrame()
{
	//tracking�̺߳�localmapping�߳̽���-����localmapping�߳�״̬
	//���Ӧ������localmapping�̲߳�Ҫֹͣ���У��������ʧ�ܱ�ʾlocalmapping�Ѿ�ֹͣ������
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);//�½��Ĺؼ�֡

    mpReferenceKF = pKF;//����track�̵߳Ĳο��ؼ�֡�ǵ�ǰ�Ĺؼ�֡
    mCurrentFrame.mpReferenceKF = pKF;//���õ�ǰ֡�Ĳο�֡�������Լ�(Ҳ����˵���йؼ�֡�Ĳο��ؼ�֡�����Լ�)

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();//���µ�ǰ֡��λ��

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        //��������������֮ǰ���ֹ���Ҳ�Ƕ������㰴�վ�������Ĵ�С��������Ȼ��ѡǰ100����Ȼ����µ�ǰ֡�е�mvpMapPoints
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
                if(!pMP)//�жϵ�ǰ֡����������Ƿ��Ѿ��ǵ�ͼ�㣬���û�ж�Ӧ�ĵ�ͼ����½���ͼ��
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);//�½���ͼ�㣬�������ɵ��µ�ͼ����tracking�е�
                    pNewMP->AddObservation(pKF,i);//���������ͼ�㱻��ǰ�ؼ�֡�۲⵽��
                    pKF->AddMapPoint(pNewMP,i);//��ǰ�ؼ�֡�۲⵽�������ͼ��
                    pNewMP->ComputeDistinctiveDescriptors();//�ۺϵ�ͼ����Ϣ ���������ͼ���������
                    pNewMP->UpdateNormalAndDepth();//�ۺϵ�ͼ����Ϣ�����������ͼ��ķ���
                    mpMap->AddMapPoint(pNewMP);//���ͼ��������ͼ��

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;//����ͼ����뵽��ǰ֡��
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
    //tracking�̺߳�localmapping�߳̽���-����localmapping�߳�״̬
    mpLocalMapper->InsertKeyFrame(pKF);//�ؼ���һ�仰����localmapping�в����½��Ĺؼ�֡

    mpLocalMapper->SetNotStop(false);//������localmapping�߳�ֹͣ������

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

//������������������õ�һ���Ǹ���ͨ���ο�֡ģ�ͻ����ٶ�ģ�͵õ��ĵ�ǰ֡�ĵ�ͼ�㱻�۲쵽�Ĵ�����
//�ڶ������ж�(�ֲ���ͼ��-��һ֡ƥ��ĵ�ͼ��)�Ƿ��ڵ�ǰ֡����Ұ�ڣ�����ڵ�ǰ֡����Ұ�����������ֲ���ͼ�㱻��ʵ�۲쵽�Ĵ���
//��������������SearchByProjection����(�ֲ���ͼ��-��һ֡ƥ��ĵ�ͼ��)ͶӰ����ǰ֡�ϣ����뵱ǰ֡�����������ƥ������ɹ��������ͼ����뵽��ǰ֡��
//�������������ʵ���Ǽ���ֲ���ͼ�����Ƿ���©��֮���뵱ǰ֡ƥ��
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //������ǰ֡���Ѿ����ͼ��ƥ���������
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
                pMP->mbTrackInView = false;//�����ʾ����һ֡��ͼ��ƥ��õ��ĵ�ǰ֡�ĵ�ͼ�����
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    //�����ֲ���ͼ�еĵ�ͼ�㡣�ж��Ƿ��ڵ�ǰ֡����Ұ��Χ�ڣ��������Ұ��Χ�������Ӹõ�ͼ�㱻�۲쵽�Ĵ���
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        //�жϵ�ͼ���Ƿ��ڵ�ǰ֡����Ұ�У�һ��Ҫע������ĵ�ͼ��һ�����������Ѿ�ƥ��ĵ�ͼ��!!!!!!!!!!!!!!!
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
	//���ֲ���ͼ��ͶӰ����ǰ֡�в��뵱ǰ֡�����������ƥ��!!!!!!!!!!!!!!!!!
	//����th�����������������ʱ�İ뾶�����th=1���������������뾶�����th!=1����th��ֵ���������뾶
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();//���¾ֲ��ؼ�֡
    UpdateLocalPoints();//���¾ֲ���ͼ��
}

//���¾ֲ���ͼ��mvpLocalMapPoints=�ֲ��ؼ�֡�ĵ�ͼ��
void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();
   //��ѯ�ֲ��ؼ�֡
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();//���ص���mvpMapPoints
        //��ѯ���عؼ�֡�еĵ�ͼ��
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)//��ֹ����ظ�
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);//���ֲ��ؼ�֡�ĵ�ͼ����뵽mvpLocalMapPoints��
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

//���¾ֲ��ؼ�֡mvpLocakKeyFrames=��ǰ֡�����е�ͼ��Ĺ��ӹؼ�֡+���ӹؼ�֡�ĸ���֡+����֡��ǰ10���Ĺ���֡��
//�������뵱ǰ֡�������Ĺؼ�֡����Ϊ��ǰ֡�Ĳο�֡��track�̵߳Ĳο�֡
void Tracking::UpdateLocalKeyFrames()
{ 
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;//�洢�����ؼ�֡�͵�ǰ֡�Ĺ��Ӵ���
    //������ǰ֡�����е�ͼ���ҵ�������Щ��ͼ��������ؼ�֡
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])//�˴��ĵ�ǰ֡�ĵ�ͼ�㣬��ͨ����track�̲߳ο��ؼ�֡ƥ��õ���
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
		  //���������ؼ�֡���뵱ǰ֡�����㹲�ӵ�����������
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
    //�ҵ��뵱ǰ֡���������Ǹ��ؼ�֡������pKFmax�С����������뵱ǰ֡���ӵĹؼ�֡������mvpLocalKeyFrames�С�
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
    //�����뵱ǰ֡���ӵ����йؼ�֡
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);//�õ��빲�ӹؼ�֡���ӳ̶���ߵ�ǰʮ���ؼ�֡
        //�����ӹؼ�֡�����̶���ߵ�ǰ10���ؼ�֡Ҳ������mvpLocalKeyFrames�С�
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

        //�����ӹؼ�֡���ӹؼ�֡�͸��ؼ�֡������mvpLocalKeyFrames��
        const set<KeyFrame*> spChilds = pKF->GetChilds();//Ŀǰ�������������ڵ���ӽڵ���ʲô
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

   //�뵱ǰ֡�������Ĺؼ�֡����Ϊ��ǰ֡�Ĳο��ؼ�֡��track�̵߳Ĳο��ؼ�֡
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

//���ȼ��㵱ǰ֡��bow��Ȼ��ʹ��bow���������еĹؼ�֡��Ѱ���ض�λ��ѡ֡��Ȼ�󽫵�ǰ֡����������е��ض�λ��ѡ֡��ͼ�����bowƥ�䣬�������pnp������
//Ȼ��ʹ��ransac+epnp�������ĳ�ʼλ�ˣ����޳�����ֵ�ĵ�ͼ��
//��ʹ���޳�����ֵ����з������Ż��õ����յ�λ��,ͨ���������Ż���Ե�ͼ���ٽ���һ��ɸѡ�ų���ֵ��ͼ�㣬Ȼ��ʹ���ٴ��ų�����ֵ�ĵ�ͼ����뵽��ǰ֡�С�
//����������Ż�����Ч�߽�����ſ�ƥ���������ٽ���һ��ƥ�䡣����ʱ����ƥ��ʱ��ֻ��ʹ��bowƥ�����ʹ�õ�ǰ֡��λ�ˣ����ؼ�֡�ĵ�ͼ��ͶӰ����ǰ֡�н���ƥ�䡣���ٽ���һ�η������Ż���
//�������Ż���Ե�ͼ���ٽ���һ��ɸѡ�ų���ֵ��ͼ�㣬Ȼ��ʹ���ٴ��ų�����ֵ�ĵ�ͼ����뵽��ǰ֡�С�
//epnp����ʹ�õ���opencv�еĴ���
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

    //��ѯ��ѡ�ؼ�֡
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);//����ѡ�ؼ�֡�ĵ�ͼ��͵�ǰ֡����ƥ��
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                //��֪��������ϵ�¿ռ��λ�ú�ͼ���Ӧ������������������ϵ�µ����λ�� 
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

    //ֻҪƥ�䵽�������˳�
    
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)//��ѯ��ѡ�ؼ�֡
        {
            if(vbDiscarded[i])//���ƥ���������̫���򲻼���
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);//!!!!!!!!!!!!!�ؼ�������pnp

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

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);//!!!!!!!!!!!!ʹ�÷������Ż���ǰ֡��λ��

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                //����������Ż��õ�����Ч�߱Ƚ������ٽ���һ�ε�ǰ֡��������͹ؼ�֡�ĵ�ͼ��ƥ�䣬Ȼ����ʹ��һ�η������Ż�
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);//�ڶ�������

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)//����ٽ���һ�η������Ż����ֻ���û���㹻����Ч����ƥ�������ſ�
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);//ƥ�������ſ��ڶ�������

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
        mnLastRelocFrameId = mCurrentFrame.mnId;//��¼���ض�λ�Ĺؼ�֡id��Ҳ���ǵ�ǰ֡
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
	//tracking�̺߳�localmapping�߳̽���-����localmapping״̬
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
