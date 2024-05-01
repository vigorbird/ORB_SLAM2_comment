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

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];//Ĭ��������0.05
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];//Ĭ��������1
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];//Ĭ��������0.9
    mPointSize = fSettings["Viewer.PointSize"];//Ĭ�ϲ�����2��դ�񻯵��ֱ��
    mCameraSize = fSettings["Viewer.CameraSize"];//Ĭ�ϲ�����0.08
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];//Ĭ��ֵ��3

}
//���pangolin�ĵ�
void MapDrawer::DrawMapPoints()
{
    //�˴�Ӧ�����Ż����ĵ�ͼ�ĵ�
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //mvpLocalMapPoints;�ֲ���ͼ��=�ֲ��ؼ�֡�����ĵ�ͼ��
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);//Ĭ����2
    glBegin(GL_POINTS);//��ʼ���Ƶ�
	    glColor3f(0.0,0.0,0.0);//��ɫ�ĵ�

	    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
	    {
	        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
	            continue;
	        cv::Mat pos = vpMPs[i]->GetWorldPos();
	        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	    }
    glEnd();//��������

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
	    glColor3f(1.0,0.0,0.0);//��ɫ�ĵ�
	    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
	    {
	        if((*sit)->isBad())
	            continue;
	        cv::Mat pos = (*sit)->GetWorldPos();
	        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

	    }
    glEnd();
}

//���pangolin�ĵ�
//bDrawKF ��ʾ�Ƿ���ƹؼ�֡
//bDrawGraph��ʾ�Ƿ�������йؼ�֡��ǰ100������֡���ջ���ѡ֡�͸��׹ؼ�֡
void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    //����ǻ��ƹؼ�֡
    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();//GetPoseInverse = Twc��Ϊpangolin���д洢���ȣ������Ҫת��

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));//ptr������������һ�����ص��׵�ַ

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);//��ɫ
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
   //�����ǻ���covisibleͼ�����Ǹ����ؼ�֡�еĿ�����ͬ��ͼ��Ĺ�ϵ����������ĵ���ͬ�ıȽ϶�������߽�������֡����
    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);//��ɫ����ɫ���ں�
        glBegin(GL_LINES);

	        for(size_t i=0; i<vpKFs.size(); i++)
	        {
	            // Covisibility Graph
	            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);//�������ؼ�֡��ǰ100�Ĺ���֡
	            cv::Mat Ow = vpKFs[i]->GetCameraCenter();//����ؼ�֡�Ĺ���λ��
	            if(!vCovKFs.empty())
	            {
	                //��������ؼ�֡ǰ100�Ĺ���֡
	                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
	                {
	                    if((*vit)->mnId<vpKFs[i]->mnId)//�������֡�Ĺؼ�֡���С�ڵ�ǰ�ؼ�֡����ţ�Ӧ����ֻ����������ؼ�֮֡����ֵĹ���֡
	                        continue;
	                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
				//�ڹ���֡�͹ؼ�֮֡�仭һ��ֱ��
	                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
	                }
	            }

	            // Spanning tree
	            KeyFrame* pParent = vpKFs[i]->GetParent();
	            if(pParent)
	            {
	                cv::Mat Owp = pParent->GetCameraCenter();
			   //�ڹؼ�֡������ؼ�֡�ĸ���֮֡�仭һ��ֱ��
	                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
	            }

	            // Loops
	            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();//�ջ���ѡ֡
	            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
	            {
	                if((*sit)->mnId<vpKFs[i]->mnId)//ֻ�����ڵ�ǰ�ؼ�֮֡��ıջ���ѡ֡
	                    continue;
	                cv::Mat Owl = (*sit)->GetCameraCenter();��
			   //�ڹؼ�֡������ؼ�֡�ıջ���ѡ֮֡�仭һ��ֱ��
	                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
	                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
	            }
	        }

      glEnd();
    }
}

//���pangloin�ĵ�
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;//Ĭ�ϲ�����0.08
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);//ת�뵽��ǰ���������ϵ�»���
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);//����ɫ��Ⱦ����Ļ�ͼ
    glBegin(GL_LINES);//glBegin()�Ǻ�glEnd()�������ʹ�á���ʾ���߶�
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

//��tracking�߳��б�����
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
