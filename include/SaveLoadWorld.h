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

//void loadKFDatabase(KeyFrameDatabase *kfdatabase);

void loadKFVariables(KeyFrameDatabase *db, Map *wd, MapKFIndexPointer *kfIdxPtMap);

void loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap);


//void loadDatafromfile(KeyFrameDatabase *db, Map *wd)
//{
//	// step 1. load keyframe database
//	loadKFDatabase(db);
//}


//void loadKFDatabase(KeyFrameDatabase *db)
//{
//	
//}



// load plain variables of mappoints from file
void loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap)
{
	ifstream ifs;
	string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.txt";
	ifs.open(strFile.c_str());

	unsigned long int nNextId;

	Frame tmpFrame;
	KeyFrame tmpKF(tmpFrame, &wd, &db);


	unsigned long int linecnt=0;
	while(!ifs.eof())
	{
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

		MapPoint* tmpMP = new MapPoint(mWorldPos, &tmpKF, &wd);
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
		tmpMP->setmnFound(mnFound);
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
}


void loadKFVariables(KeyFrameDatabase *db, Map *wd, MapKFIndexPointer *kfIdxPtMap)
{

}



#endif

