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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;//1֡ͼ����ٺ���

    mImageWidth = fSettings["Camera.width"];//����ͼ��Ŀ�
    mImageHeight = fSettings["Camera.height"];//����ͼ��ĸ�
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];//Ĭ�ϲ�����0
    mViewpointY = fSettings["Viewer.ViewpointY"];//Ĭ�ϲ�����-0.7
    mViewpointZ = fSettings["Viewer.ViewpointZ"];//Ĭ�ϲ�����-1.8
    mViewpointF = fSettings["Viewer.ViewpointF"];//Ĭ�ϲ�����500
}

//���ӻ�����Ҫ����
//ע��!pangolin���д洢���ȵ�
void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);//����һ����ʾ���ڴ�СΪ1024*768

    // 3D Mouse handler requires depth testing to be enabled
    //������Ȳ��ԣ���ǰ����ǰ���Ƿ��б�����أ����������ص��������������Ͳ�����ƣ�Ҳ����˵��OpenGL��ֻ������ǰ���һ�㡣
     glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    //������ڻ����ʾ
    glEnable (GL_BLEND);//���û��
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//����һ����Ϊ���õĻ����ʾģʽ

    //�����˵�����ť
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);//��һ����Ĭ��״̬���ڶ����ǵ�ǰ��״̬
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    //�ڷ�һ��������۲�ռ�
    //ProjectionMatrix��������������ڲ�:1024-ͼ��Ŀ�768-ͼ��ĸߣ�ViewpointF-����ڲ�fx,512-u0,389-v0,0.1-����ܹ������ľ��룬1000-��Զ�ܿ����ľ���
    //ModelViewLookAt������:mViewpointX,mViewpointY,mViewpointZ�������������������ϵ�µ�λ�ã�0,0,0�����Ҫ�����ĵ��λ�ã�0��-1,0�Ƕ����ĸ�����ͼ����
     pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                                                          pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                                                             );

    // Add named OpenGL viewport to window and provide 3D Handler
    //����һ������-����۲⵽��ͼ��ƽ��
    //0,1��ʾ�ܹ����������Ĵ��ڣ�-1024.0f/768.0f��ʾ��ͼ�ı���
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f).SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();//���Ƚ������λ�˾�������Ϊ��λ����

    cv::namedWindow("ORB-SLAM2: Current Frame");//Ϊ��ʹ��opencv��ʾͼ�񣬱�����ʹ��namedWindow����һ������

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// �����ɫ�����Լ���Ȼ��壬��ֹ�����ص�

	    //���������λ��
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

	    //�����趨��ģʽ�����в������趨
        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

	    //����Ϳ�ʼ�����pangolin��ͼ����
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);//���ñ�����ɫ�������ɫ����
        mpMapDrawer->DrawCurrentCamera(Twc);//���Ƶ�ǰ֡��λ��
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);//�������йؼ�֡����Щ�ؼ�֡��ͼ����
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();//���Ƶ�ͼ�е����е�ͼ��
        pangolin::FinishFrame();//�ȴ��¼��Ĵ���Ӧ����pangolin�����������ȴ�������ɡ�

	    //����ע������ȫ������ʹ��pangolin���Ƶ�ͼ��
	    //���涼�Ƕ�opencv����ʾ��ͼ����л���
        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);//��λ�Ǻ���=1/fps��һ��Ҫ������� ����ͼ���޷���ʾ

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }   

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
