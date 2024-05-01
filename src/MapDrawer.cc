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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];//默认设置是0.05
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];//默认设置是1
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];//默认设置是0.9
    mPointSize = fSettings["Viewer.PointSize"];//默认参数是2，栅格化点的直径
    mCameraSize = fSettings["Viewer.CameraSize"];//默认参数是0.08
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];//默认值是3

}
//详见pangolin文档
void MapDrawer::DrawMapPoints()
{
    //此处应该是优化过的地图的点
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //mvpLocalMapPoints;局部地图点=局部关键帧看到的地图点
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);//默认是2
    glBegin(GL_POINTS);//开始绘制点
	    glColor3f(0.0,0.0,0.0);//黑色的点

	    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	    {
	        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
	            continue;
	        cv::Mat pos = vpMPs[i]->GetWorldPos();
	        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	    }
    glEnd();//结束绘制

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
	    glColor3f(1.0,0.0,0.0);//红色的点
	    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
	    {
	        if((*sit)->isBad())
	            continue;
	        cv::Mat pos = (*sit)->GetWorldPos();
	        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

	    }
    glEnd();
}

//详见pangolin文档
//bDrawKF 表示是否绘制关键帧
//bDrawGraph表示是否绘制所有关键帧的前100个共视帧、闭环候选帧和父亲关键帧
void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    //这个是绘制关键帧
    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();//GetPoseInverse = Twc因为pangolin是列存储优先，因此需要转置

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));//ptr函数访问任意一行像素的首地址

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);//蓝色
            glBegin(GL_LINES);
	            glVertex3f(0,0,0);
	            glVertex3f(w,h,z);
	            glVertex3f(0,0,0);
	            glVertex3f(w,-h,z);
	            glVertex3f(0,0,0);
	            glVertex3f(-w,-h,z);
	            glVertex3f(0,0,0);
	            glVertex3f(-w,h,z);

	            glVertex3f(w,h,z);
	            glVertex3f(w,-h,z);

	            glVertex3f(-w,h,z);
	            glVertex3f(-w,-h,z);


	            glVertex3f(-w,h,z);
	            glVertex3f(w,h,z);

	            glVertex3f(-w,-h,z);
	            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }
   //下面是绘制covisible图，就是各个关键帧中的看到相同地图点的关系，如果看到的点相同的比较多就是用线将这两个帧相连
    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);//绿色和蓝色的融合
        glBegin(GL_LINES);

	        for(size_t i=0; i<vpKFs.size(); i++)
	        {
	            // Covisibility Graph
	            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);//获得这个关键帧的前100的共视帧
	            cv::Mat Ow = vpKFs[i]->GetCameraCenter();//这个关键帧的光心位置
	            if(!vCovKFs.empty())
	            {
	                //遍历这个关键帧前100的共视帧
	                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
	                {
	                    if((*vit)->mnId<vpKFs[i]->mnId)//这个共视帧的关键帧序号小于当前关键帧的序号，应该是只绘制在这个关键帧之后出现的共视帧
	                        continue;
	                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
				//在共视帧和关键帧之间画一条直线
	                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
	                }
	            }

	            // Spanning tree
	            KeyFrame* pParent = vpKFs[i]->GetParent();
	            if(pParent)
	            {
	                cv::Mat Owp = pParent->GetCameraCenter();
			   //在关键帧和这个关键帧的父亲帧之间画一条直线
	                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
	            }

	            // Loops
	            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();//闭环候选帧
	            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
	            {
	                if((*sit)->mnId<vpKFs[i]->mnId)//只绘制在当前关键帧之后的闭环候选帧
	                    continue;
	                cv::Mat Owl = (*sit)->GetCameraCenter();‘
			   //在关键帧和这个关键帧的闭环候选帧之间画一条直线
	                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
	            }
	        }

      glEnd();
    }
}

//详见pangloin文档
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;//默认参数是0.08
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);//转入到当前的相机坐标系下绘制
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);//用绿色渲染下面的绘图
    glBegin(GL_LINES);//glBegin()是和glEnd()结合起来使用。表示画线段
	    glVertex3f(0,0,0);
	    glVertex3f(w,h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(w,-h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(-w,-h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(-w,h,z);

	    glVertex3f(w,h,z);
	    glVertex3f(w,-h,z);

	    glVertex3f(-w,h,z);
	    glVertex3f(-w,-h,z);

	    glVertex3f(-w,h,z);
	    glVertex3f(w,h,z);

	    glVertex3f(-w,-h,z);
	    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

//在tracking线程中被调用
void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
