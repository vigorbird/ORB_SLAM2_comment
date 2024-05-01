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
    mT = 1e3/fps;//1帧图像多少毫秒

    mImageWidth = fSettings["Camera.width"];//设置图像的宽
    mImageHeight = fSettings["Camera.height"];//设置图像的高
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];//默认参数是0
    mViewpointY = fSettings["Viewer.ViewpointY"];//默认参数是-0.7
    mViewpointZ = fSettings["Viewer.ViewpointZ"];//默认参数是-1.8
    mViewpointF = fSettings["Viewer.ViewpointF"];//默认参数是500
}

//可视化的主要函数
//注意!pangolin是列存储优先的
void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);//生成一个显示窗口大小为1024*768

    // 3D Mouse handler requires depth testing to be enabled
    //启用深度测试，当前像素前面是否有别的像素，如果别的像素挡道了它，那它就不会绘制，也就是说，OpenGL就只绘制最前面的一层。
     glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    //这个用于混合显示
    glEnable (GL_BLEND);//启用混合
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//这是一种最为常用的混合显示模式

    //创建菜单栏按钮
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);//第一个是默认状态，第二个是当前的状态
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    //摆放一个相机即观察空间
    //ProjectionMatrix参数相是相机的内参:1024-图像的宽，768-图像的高，ViewpointF-相机内参fx,512-u0,389-v0,0.1-最近能够看到的距离，1000-最远能看到的距离
    //ModelViewLookAt参数是:mViewpointX,mViewpointY,mViewpointZ是相机光心在世界坐标系下的位置；0,0,0是相机要看到的点的位置；0，-1,0是定义哪个轴在图像上
     pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                                                          pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                                                             );

    // Add named OpenGL viewport to window and provide 3D Handler
    //创建一个窗口-相机观测到的图像平面
    //0,1表示能够看到整个的窗口，-1024.0f/768.0f表示绘图的比例
    pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f).SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();//首先将相机的位姿矩阵设置为单位矩阵

    cv::namedWindow("ORB-SLAM2: Current Frame");//为了使用opencv显示图像，必须先使用namedWindow创建一个窗体

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// 清除颜色缓冲以及深度缓冲，防止绘制重叠

	    //跟随相机的位姿
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

	    //根据设定的模式，进行参数的设定
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

	    //下面就开始具体的pangolin画图工作
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);//设置背景白色并清除颜色缓存
        mpMapDrawer->DrawCurrentCamera(Twc);//绘制当前帧的位置
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);//绘制所有关键帧和这些关键帧的图关心
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();//绘制地图中的所有地图点
        pangolin::FinishFrame();//等待事件的处理，应该是pangolin阻塞函数，等待绘制完成。

	    //这里注意上面全部都是使用pangolin绘制的图像，
	    //下面都是对opencv中显示的图像进行绘制
        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);//单位是毫秒=1/fps，一定要加上这句 否则图像无法显示

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
