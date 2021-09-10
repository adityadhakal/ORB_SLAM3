/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/set.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

struct classcomp {
  bool operator() (boost::interprocess::offset_ptr<KeyFrame> lhs, boost::interprocess::offset_ptr<KeyFrame> rhs) const
  {return lhs.get_offset()<rhs.get_offset();}
};


class KeyFrame
{

public:
    KeyFrame();
    KeyFrame(Frame &F, boost::interprocess::offset_ptr<Map> pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    void SetVelocity(const cv::Mat &Vw_);

    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetImuPosition();
    cv::Mat GetImuRotation();
    cv::Mat GetImuPose();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    cv::Mat GetVelocity();

    cv::Matx33f GetRotation_();
    cv::Matx31f GetTranslation_();
    cv::Matx31f GetCameraCenter_();
    cv::Matx33f GetRightRotation_();
    cv::Matx31f GetRightTranslation_();
    cv::Matx44f GetRightPose_();
    cv::Matx31f GetRightCameraCenter_();
    cv::Matx44f GetPose_();


    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(boost::interprocess::offset_ptr<KeyFrame> pKF, const int &weight);
    void EraseConnection(boost::interprocess::offset_ptr<KeyFrame> pKF);

    void UpdateConnections(bool upParent=true);
    void UpdateBestCovisibles();
    std::set<boost::interprocess::offset_ptr<KeyFrame> > GetConnectedKeyFrames();
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > GetVectorCovisibleKeyFrames();
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > GetBestCovisibilityKeyFrames(const int &N);
    std::vector<boost::interprocess::offset_ptr<KeyFrame> > GetCovisiblesByWeight(const int &w);
    int GetWeight(boost::interprocess::offset_ptr<KeyFrame> pKF);

    // Spanning tree functions
    void AddChild(boost::interprocess::offset_ptr<KeyFrame> pKF);
    void EraseChild(boost::interprocess::offset_ptr<KeyFrame> pKF);
    void ChangeParent(boost::interprocess::offset_ptr<KeyFrame> pKF);
    std::set<boost::interprocess::offset_ptr<KeyFrame> > GetChilds();
    boost::interprocess::offset_ptr<KeyFrame> GetParent();
    bool hasChild(boost::interprocess::offset_ptr<KeyFrame> pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(boost::interprocess::offset_ptr<KeyFrame> pKF);
    std::set<boost::interprocess::offset_ptr<KeyFrame> > GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(boost::interprocess::offset_ptr<KeyFrame> pKF);
    set<boost::interprocess::offset_ptr<KeyFrame> > GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(boost::interprocess::offset_ptr<MapPoint>  pMP, const size_t &idx);
    void EraseMapPointMatch(const int &idx);
    void EraseMapPointMatch(boost::interprocess::offset_ptr<MapPoint>  pMP);
    void ReplaceMapPointMatch(const int &idx, boost::interprocess::offset_ptr<MapPoint>  pMP);
    std::set<boost::interprocess::offset_ptr<MapPoint> > GetMapPoints();
    std::vector<boost::interprocess::offset_ptr<MapPoint> > GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    boost::interprocess::offset_ptr<MapPoint>  GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;
    cv::Mat UnprojectStereo(int i);
    cv::Matx31f UnprojectStereo_(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(boost::interprocess::offset_ptr<KeyFrame> pKF1, boost::interprocess::offset_ptr<KeyFrame> pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    boost::interprocess::offset_ptr<Map> GetMap();
    void UpdateMap(boost::interprocess::offset_ptr<Map> pMap);

    void SetNewBias(const IMU::Bias &b);
    cv::Mat GetGyroBias();
    cv::Mat GetAccBias();
    IMU::Bias GetImuBias();

    bool ProjectPointDistort(boost::interprocess::offset_ptr<MapPoint>  pMP, cv::Point2f &kp, float &u, float &v);
    bool ProjectPointUnDistort(boost::interprocess::offset_ptr<MapPoint>  pMP, cv::Point2f &kp, float &u, float &v);

    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    bool bImu;


    //new code
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<MapPoint>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_mappoint;
    typedef boost::interprocess::vector<boost::interprocess::offset_ptr<MapPoint>, ShmemAllocator_mappoint> MyVector_mappoint;

    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<KeyFrame>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_keyframe;
    typedef boost::interprocess::vector<boost::interprocess::offset_ptr<KeyFrame>, ShmemAllocator_keyframe> MyVector_keyframe;



    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;


    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    cv::Mat mVwbGBA;
    cv::Mat mVwbBefGBA;

    // Their component void * functions
    boost::interprocess::offset_ptr<char> mTcwGBA_ptr;
    boost::interprocess::offset_ptr<char> mTcwBefGBA_ptr;
    boost::interprocess::offset_ptr<char> mVwbGBA_ptr;
    boost::interprocess::offset_ptr<char> mVwbBefGBA_ptr;

    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    cv::Mat mTcwMerge;
    cv::Mat mTcwBefMerge;
    cv::Mat mTwcBefMerge;
    cv::Mat mVwbMerge;
    cv::Mat mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // Preintegrated IMU measurements from previous keyframe
    boost::interprocess::offset_ptr<KeyFrame> mPrevKF;
    boost::interprocess::offset_ptr<KeyFrame> mNextKF;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;


    unsigned int mnOriginMapId;

    string mNameFile;

    int mnDataset;

    //Old CODE:
    //---------//
    //   std::vector <boost::interprocess::offset_ptr<KeyFrame> > mvpLoopCandKFs;
    //   std::vector <boost::interprocess::offset_ptr<KeyFrame> > mvpMergeCandKFs;
    //---------//
    //New Code:
    boost::interprocess::offset_ptr<MyVector_keyframe> mvpLoopCandKFs;
    boost::interprocess::offset_ptr<MyVector_keyframe> mvpMergeCandKFs;



    bool mbHasHessian;
    cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    cv::Mat Cw; // Stereo middel point. Only for visualization

    // matrices data pointers
    boost::interprocess::offset_ptr<char> Tcw_ptr;
    boost::interprocess::offset_ptr<char> Twc_ptr;
    boost::interprocess::offset_ptr<char> Ow_ptr;
    boost::interprocess::offset_ptr<char> Cw_ptr;
    boost::interprocess::offset_ptr<char> Owb_ptr;
    boost::interprocess::offset_ptr<char> Vw_ptr;



    cv::Matx44f Tcw_, Twc_, Tlr_;
    cv::Matx31f Ow_;

    // IMU position
    cv::Mat Owb;

    // Velocity (Only used for inertial SLAM)
    cv::Mat Vw;

    // Imu bias
    IMU::Bias mImuBias;

    //example code
    //typedef boost::interprocess::allocator<unsigned long int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_longint; 
    //typedef vector<unsigned long int, ShmemAllocator_longint> MyVector_longint;
    //MyVector_longint *mvBackupKeyFrameOriginsId;

    // MapPoints associated to keypoints:: OLD CODE
    //std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpMapPoints;

    //new code
    //boost::interprocess::offset_ptr<MyVector_mappoint> mvpMapPoints_original;
    boost::interprocess::offset_ptr<MyVector_mappoint> mvpMapPoints;
    std::vector<boost::interprocess::offset_ptr<MapPoint> > mvpMapPoints_vector;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;



    //we need different datastructure for maps
    typedef boost::interprocess::offset_ptr<KeyFrame> keyType;
    typedef int MappedType;
    typedef std::pair<const boost::interprocess::offset_ptr<KeyFrame>, int> ValueType;
    typedef boost::interprocess::allocator<ValueType,boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_map_keyframe;

    //Maps and Vector.
    typedef boost::interprocess::map<keyType,MappedType,std::less<keyType>,ShmemAllocator_map_keyframe> MyMap;

    //old-code : Map
    //std::map<boost::interprocess::offset_ptr<KeyFrame>,int> mConnectedKeyFrameWeights;
    //new-code : Map
    boost::interprocess::offset_ptr<MyMap> mConnectedKeyFrameWeights;


    //old-code
    //std::vector<boost::interprocess::offset_ptr<KeyFrame> > mvpOrderedConnectedKeyFrames;
    //new-code
    boost::interprocess::offset_ptr<MyVector_keyframe> mvpOrderedConnectedKeyFrames;

    std::vector<int> mvOrderedWeights;


    //new code for Set.
    typedef boost::interprocess::allocator<boost::interprocess::offset_ptr<KeyFrame>, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_keyframe_set;
    typedef boost::interprocess::set<boost::interprocess::offset_ptr<KeyFrame>, std::less<boost::interprocess::offset_ptr<KeyFrame> >,ShmemAllocator_keyframe_set> Myset_keyframe;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    boost::interprocess::offset_ptr<KeyFrame> mpParent;

    //replace the STL set with regular set
    std::set<boost::interprocess::offset_ptr<KeyFrame> > mspChildrens;
    std::set<boost::interprocess::offset_ptr<KeyFrame> > mspLoopEdges;
    //OLD CODE
    //std::set<boost::interprocess::offset_ptr<KeyFrame> > mspMergeEdges;
    //New CODE
    boost::interprocess::offset_ptr<Myset_keyframe> mspMergeEdges;
    std::set<boost::interprocess::offset_ptr<KeyFrame> > mspMergeEdges_support;
    

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    boost::interprocess::offset_ptr<Map> mpMap;

    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //Transformation matrix between cameras in stereo fisheye
    cv::Mat mTlr;
    cv::Mat mTrl;

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> mvKeysRight;

    const int NLeft, NRight;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    cv::Mat GetRightPose();
    cv::Mat GetRightPoseInverse();
    cv::Mat GetRightPoseInverseH();
    cv::Mat GetRightCameraCenter();
    cv::Mat GetRightRotation();
    cv::Mat GetRightTranslation();

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            //if(mvpMapPoints[i]){
            if((*mvpMapPoints)[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
    }


};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
