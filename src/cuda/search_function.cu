#include "search_function.h"
#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Initializer.h"
#include"G2oTypes.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include<chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>

//A global function for searching.. just taken from 
GPUSearchLocalPoints()
{
	std::chrono::steady_clock::time_point begin_mutex,end_mutex;
	double total_mutex_time = 0.0;
	std::chrono::steady_clock::time_point begin_local = std::chrono::steady_clock::now();

    // Do not search map points already matched
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
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }


    int nToMatch=0;

    std::chrono::steady_clock::time_point begin_proj = std::chrono::steady_clock::now();
    double agg = 0.0;

    //Aditya: Check how many map-points are there.
    //std::cout<<"The total number of local map points are: "<<mvpLocalMapPoints.size()<<std::endl;
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        begin_mutex = std::chrono::steady_clock::now();
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        end_mutex = std::chrono::steady_clock::now();
        total_mutex_time += std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(end_mutex - begin_mutex).count();

        if(pMP->mbTrackInView)
        {
        	std::chrono::steady_clock::time_point begin_p2f = std::chrono::steady_clock::now();
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
            std::chrono::steady_clock::time_point end_p2f = std::chrono::steady_clock::now();
            double t_p2f = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(end_p2f - begin_p2f).count();
            agg += t_p2f;

        }
    }
    std::chrono::steady_clock::time_point end_proj = std::chrono::steady_clock::now();

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        std::chrono::steady_clock::time_point begin_projection = std::chrono::steady_clock::now();
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
        std::chrono::steady_clock::time_point end_projection = std::chrono::steady_clock::now();
        double t_p_project = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(end_proj- begin_proj).count();
        std::cout<<"Local Tracking (point projection) time (ms): "<<t_p_project<<std::endl;
        double t_project = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(end_projection - begin_projection).count();
        std::cout<<"Local Tracking (search by projection) time (ms): "<<t_project<<std::endl;
    }
    std::chrono::steady_clock::time_point end_local = std::chrono::steady_clock::now();
    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(end_local - begin_local).count();
    std::cout<<"Local Tracking time (ms): "<<t_track<<std::endl;
    std::cout<<"Aggregate P2f time (ms): "<<agg<<std::endl;
    std::cout<<"Aggregate total MUTEX time (ms): "<<total_mutex_time<<std::endl;
}