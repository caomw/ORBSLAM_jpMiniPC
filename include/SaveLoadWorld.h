/*
* Added by wangjing
*/

#ifndef SAVELOADWORLD_H
#define SAVELOADWORLD_H

#include <Frame.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <Map.h>
#include <KeyFrameDatabase.h>

#include<ros/ros.h>
#include<ros/package.h>

#include <opencv2/core/core.hpp>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;
using namespace ORB_SLAM;

typedef map<long unsigned int, MapPoint*> MapMPIndexPointer;
typedef map<long unsigned int, KeyFrame*> MapKFIndexPointer;


// load MapPoint data
bool loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap);

// load KeyFrame data
bool loadKFVariables(KeyFrameDatabase *db, Map *wd, MapKFIndexPointer *kfIdxPtMap);

// load voc-keyframe invert index, after keyframe data is loaded
bool loadKFDatabase(KeyFrameDatabase *db);

// --------------------------------------------------
// --------------------------------------------------
// --------------------------------------------------


static bool myOpenFile(ifstream &ifs, string strFile)
{
	ifs.open(strFile.c_str());
	if(!ifs.is_open() || ifs.eof())
	{
		cout<<strFile<<" open failed."<<endl;
		return false;
	}
	return true;
}


// load plain variables of mappoints from file
bool loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap)
{
	ifstream ifs;
	if(!myOpenFile(ifs, string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.txt")))
		return false;

	unsigned long int nNextId;

	Frame tmpFrame;
    KeyFrame * tmpKF = new KeyFrame(tmpFrame, wd, db);

	unsigned long int linecnt=0;
	while(!ifs.eof())
	{
		// new MapPoint memory space and pointer
        MapPoint* tmpMP = new MapPoint(mWorldPos, tmpKF, wd);
		
		string sl;
		stringstream ss;
		
		getline(ifs,sl);
		ss << sl;

		// plain variables in MapPoint
		long unsigned int mnId, mnTrackReferenceForFrame, mnLastFrameSeen, mnBALocalForKF, mnFuseCandidateForKF;
		long unsigned int mnLoopPointForKF, mnCorrectedByKF, mnCorrectedReference, mpRefKFId;
		long int mnFirstKFid;
		float mTrackProjX, mTrackProjY, mTrackViewCos, mfMinDistance, mfMaxDistance;
		bool mbTrackInView;
		int mnTrackScaleLevel, mnVisible, mnFound;
		Mat mWorldPos = Mat::zeros(3, 1, CV_32F);
		Mat mNormalVector = Mat::zeros(3, 1, CV_32F);
		Mat mDescriptor = Mat::zeros(1, 32, CV_8UC1);
		
		ss >> nNextId >> mnId >> mnFirstKFid >> mTrackProjX >> mTrackProjY >> mbTrackInView >> mnTrackScaleLevel >> mTrackViewCos >> mnTrackReferenceForFrame;
		ss >> mnLastFrameSeen >> mnBALocalForKF >> mnFuseCandidateForKF >> mnLoopPointForKF >> mnCorrectedByKF >> mnCorrectedReference;
		for (int i = 0; i < 3; i++)
		{
			float tmpf;
			ss >> tmpf;
			mWorldPos.at<float>(i) = tmpf;
		}
		for (int i = 0; i < 3; i++)
		{
			float tmpf;
			ss >> tmpf;
			mNormalVector.at<float>(i) = tmpf;
		}
		uint32_t *tpdes = mDescriptor.ptr<uint32_t>();
		for (int i = 0; i < 8; i++)
		{
			uint32_t tmpi;
			ss >> tmpi;
			tpdes[i] = tmpi;
		}
		ss >> mnVisible >> mnFound >> mfMinDistance >> mfMaxDistance >> mpRefKFId;

		//static
		MapPoint::nNextId = nNextId;
		//public
		tmpMP->mnId = mnId;
		tmpMP->mnFirstKFid = mnFirstKFid;
		tmpMP->mTrackProjX = mTrackProjX;
		tmpMP->mTrackProjY = mTrackProjY;
		tmpMP->mbTrackInView = mbTrackInView;
		tmpMP->mnTrackScaleLevel = mnTrackScaleLevel;
		tmpMP->mTrackViewCos = mTrackViewCos;
		tmpMP->mnTrackReferenceForFrame = mnTrackReferenceForFrame;
		tmpMP->mnLastFrameSeen = mnLastFrameSeen;
		tmpMP->mnBALocalForKF = mnBALocalForKF;
		tmpMP->mnFuseCandidateForKF = mnFuseCandidateForKF;
		tmpMP->mnLoopPointForKF = mnLoopPointForKF;
		tmpMP->mnCorrectedByKF = mnCorrectedByKF;
		tmpMP->mnCorrectedReference = mnCorrectedReference;
		//protected
		tmpMP->SetWorldPos(mWorldPos);
		tmpMP->SetNormalVec(mNormalVector);
		tmpMP->SetDescriptor(mDescriptor);
		tmpMP->SetmnVisible(mnVisible);
        tmpMP->SetmnFound(mnFound);
		tmpMP->SetMinDistance(mfMaxDistance);
		tmpMP->SetMaxDistance(mfMaxDistance);

		// add to the mapping from index to pointer
		if(mpIdxPtMap->count(mnId)>0)
			cerr<<"exist? shouldn't!!"<<endl;
		(*mpIdxPtMap)[mnId] = tmpMP;

//		//to be added
//		tmpMP->mObservations;
//		tmpMP->mpRefKF;


		// increment count
		linecnt++;
	}

	cout<<"total "<<linecnt<<" MapPoint loaded."<<endl;

    delete tmpKF;
	
	return true;
}



bool loadKFVariables(KeyFrameDatabase *db, Map *wd, MapKFIndexPointer *kfIdxPtMap)
{
	ifstream ifkfVar,ifkfKeys,ifkfKeysUn,ifkfDes,ifkfMPids,ifGlobal;
	if(	!myOpenFile(ifkfVar,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.txt")) 	||
		!myOpenFile(ifkfKeys,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.txt")) 	||
		!myOpenFile(ifkfKeysUn,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.txt")) 	||
		!myOpenFile(ifkfDes,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.txt"))	||
		!myOpenFile(ifkfMPids,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.txt"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"))    )
	{	
		return false;
	}

	long unsigned int gnNextId;
	float mfGridElementWidthInv, mfGridElementHeightInv, fx, fy, cx, cy;
	int mnScaleLevels0, mnScaleLevelsOther;
    std::vector<float> mvScaleFactors0,mvScaleFactorsOther;
    std::vector<float> mvLevelSigma20,mvLevelSigma2Other;
    std::vector<float> mvInvLevelSigma20,mvInvLevelSigma2Other;
	int mnMinX,mnMinY,mnMaxX,mnMaxY;
	{
	string slg;
	stringstream ssg;
	//Line0, MP.nNextId
	getline(ifGlobal, slg);	
	//Line1, KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy
	getline(ifGlobal, slg);	
	ssg<<slg;
	ssg>>gnNextId>>mfGridElementWidthInv>>mfGridElementHeightInv>>fx>>fy>>cx>>cy>>mnMinX>>mnMinY>>mnMaxX>>mnMaxY;
	//Line2, mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for the first 2 KFs. 
	getline(ifGlobal, slg);	
	ssg<<slg;
	ssg>>mnScaleLevels0;
	mvScaleFactors0.resize(mnScaleLevels0);
	mvLevelSigma20.resize(mnScaleLevels0);
	mvInvLevelSigma20.resize(mnScaleLevels0);
	for(int i=0;i<mnScaleLevels0;i++)
	{
		ssg>>mvScaleFactors0[i];
	}
	for(int i=0;i<mnScaleLevels0;i++)
	{
		ssg>>mvLevelSigma20[i];
		mvInvLevelSigma20[i]=1/mvLevelSigma20[i];
	}
	//Line3, mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
	getline(ifGlobal, slg);
	ssg<<slg;
	ssg>>mnScaleLevelsOther;
	mvScaleFactorsOther.resize(mnScaleLevelsOther);
	mvLevelSigma2Other.resize(mnScaleLevelsOther);
	mvInvLevelSigma2Other.resize(mnScaleLevelsOther);
	for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ssg>>mvScaleFactorsOther[i];
	}
	for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ssg>>mvLevelSigma2Other[i];
		mvInvLevelSigma2Other[i]=1/mvLevelSigma2Other[i];
	}
	}

	unsigned long int nNextId;
	
	Frame tmpFrame;
	
	unsigned long int linecnt=0;
	while(!ifkfVar.eof())
	{
		KeyFrame* tmpKF = new KeyFrame(tmpFrame, wd, db);
	
		string slVar,slKeys,slKeysUn,slDes,slMPids;
		stringstream ssVar,ssKeys,ssKeysUn,ssDes,ssMPids;
			

		//in kfVariables.txt
		getline(ifkfVar,slVar);
		ssVar << slVar;	
		//public
		long unsigned int mnId,mnFrameId,mnTrackReferenceForFrame, mnFuseTargetForKF,mnBALocalForKF,mnBAFixedForKF;
		long unsigned int mnLoopQuery,mnRelocQuery;
		double mTimeStamp;
		int mnLoopWords,mnRelocWords;
		float mLoopScore,mRelocScore;
		//protected
		cv::Mat Rcwi = Mat::eye(3, 3, CV_32F);
		cv::Mat tcwi = Mat::zeros(3, 1, CV_32F);
		cv::Mat Owi  = Mat::zeros(3, 1, CV_32F);

		ssVar>>nNextId>>mnId>>mnFrameId>>mTimeStamp>>mnTrackReferenceForFrame;
		ssVar>>mnFuseTargetForKF>>mnBALocalForKF>>mnBAFixedForKF>>mnLoopQuery>>mnLoopWords;
		ssVar>>mLoopScore>>mnRelocQuery>>mnRelocWords>>mRelocScore;
		for(int ti=0;ti<3;ti++)		{
			for(int tj=0;tj<3;tj++)		{
				ssVar >> Rcwi.at<float>(ti,tj);	}	}
		for(int ti=0;ti<3;ti++)		{
			ssVar >> tcwi.at<float>(ti);			}
		for(int ti=0;ti<3;ti++)		{
			ssVar >> Owi.at<float>(ti);				}

		//evaluate 
		KeyFrame::nNextId = nNextId;
		tmpKF->mnId = mnId;
		tmpKF->mnFrameId = mnFrameId;
		tmpKF->mTimeStamp = mTimeStamp;
		tmpKF->mnTrackReferenceForFrame = mnTrackReferenceForFrame;
		tmpKF->mnFuseTargetForKF = mnFuseTargetForKF;
		tmpKF->mnBALocalForKF = mnBALocalForKF;
		tmpKF->mnBAFixedForKF = mnBAFixedForKF;
		tmpKF->mnLoopQuery = mnLoopQuery;
		tmpKF->mnLoopWords = mnLoopWords;
		tmpKF->mLoopScore = mLoopScore;
		tmpKF->mnRelocQuery = mnRelocQuery;
		tmpKF->mnRelocWords = mnRelocWords;
		tmpKF->mRelocScore = mRelocScore;
		tmpKF->SetPose(Rcwi,tcwi);
		
		//there're some global params
		//remember to evaluate them
		tmpKF->mfGridElementWidthInv = mfGridElementWidthInv;
		tmpKF->mfGridElementHeightInv = mfGridElementHeightInv;
		tmpKF->fx = fx;
		tmpKF->fy = fy;
		tmpKF->cx = cx;
		tmpKF->cy = cy;
	    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
	    K.at<float>(0,0) = fx;
	    K.at<float>(1,1) = fy;
	    K.at<float>(0,2) = cx;
	    K.at<float>(1,2) = cy;
	    tmpKF->SetmK(K);
		tmpKF->SetMinMaxXY(mnMinX,mnMinY,mnMaxX,mnMaxY);

		if(mnId<=1 && mnId>=0)
		{
			tmpKF->SetScaleParams(mnScaleLevels0,mvScaleFactors0,mvLevelSigma20);
		}
		else
		{
			tmpKF->SetScaleParams(mnScaleLevelsOther,mvScaleFactorsOther,mvLevelSigma2Other);
		}
		
		//here
		//here
		//here
		//here
		//here
		//here
		//here
		//here
		//here
		//here
		//here

		
		getline(ifkfKeys,slKeys);
		ssKeys << slKeys;
		getline(ifkfKeysUn,slKeysUn);
		ssKeysUn << slKeysUn;
		getline(ifkfDes,slDes);
		ssDes << slDes;
		getline(ifkfMPids,slMPids);
		ssMPids << slMPids;

		

		//to be added
//		cv::Mat im;
//		DBoW2::BowVector mBowVec;
//		std::vector<cv::KeyPoint> mvKeys;
//		std::vector<cv::KeyPoint> mvKeysUn;
//		cv::Mat mDescriptors;
//		std::vector<MapPoint*> mvpMapPoints;
//		KeyFrameDatabase* mpKeyFrameDB;
//		ORBVocabulary* mpORBvocabulary;
//		DBoW2::FeatureVector mFeatVec;
//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
//		std::vector<int> mvOrderedWeights;
//		bool mbFirstConnection;
//		KeyFrame* mpParent;
//		std::set<KeyFrame*> mspChildrens;
//		std::set<KeyFrame*> mspLoopEdges;
//    	Map* mpMap;

	}



	return true;
}


//bool loadKFDatabase(KeyFrameDatabase *db)
//{
//	ifstream ifs;
//	string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"KeyFrameDatabase.txt";
//	ifs.open(strFile.c_str());
//	if(!ifs.is_open() || ifs.eof())
//	{
//		cout<<"database file open failed."<<endl;
//		return false;
//	}

//	db->add(

//	return true;
//}


#endif

