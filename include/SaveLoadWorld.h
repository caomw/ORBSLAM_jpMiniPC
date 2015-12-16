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
typedef vector<long unsigned int> VecUL;

// load MapPoint data
bool loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap, 
		VecUL &_VecMPmnId, VecUL &_vRefKFIdInMP);

// load KeyFrame data
bool loadKFVariables(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc,
	MapKFIndexPointer *kfIdxPtMap, VecUL &_VecKFmnId);

// load voc-keyframe invert index, after keyframe data is loaded
bool loadKFDatabase(KeyFrameDatabase *db);

// associate pointers in MPs and KFs
void loadMPKFPointers(MapMPIndexPointer *mpIdxPtMap, MapKFIndexPointer *kfIdxPtMap, 
		const VecUL &vRefKFIdInMP);


// load all data. main function
bool LoadWroldFromFile(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc);


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
bool loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap, 
		VecUL &_VecMPmnId, VecUL &_vRefKFIdInMP)
{
	ifstream ifs,ifGlobal;
	if(	!myOpenFile(ifs, string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.txt"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"))   )
		return false;

	//save mappoint id in each KF
	long unsigned int kfSaveCnt,mpSaveCnt;
	{
	long unsigned int gnNExtIdMP;
	string slg;
	stringstream ssg;
	//Line0, MP.nNextId
	getline(ifGlobal, slg); 
	ssg<<slg;
	ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt;
	}

	//record the reference KF's mnId, in the order of lines saved in file
	VecUL vRefKFIdInMP(mpSaveCnt);
	VecUL VecMPmnId(mpSaveCnt);

	unsigned long int nNextId;

	Frame tmpFrame;
    KeyFrame * tmpKF = new KeyFrame(tmpFrame, wd, db);

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


		// new MapPoint memory space and pointer
        MapPoint* tmpMP = new MapPoint(mWorldPos, tmpKF, wd);


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


		//record reference KF id of this mappoint
		vRefKFIdInMP[linecnt] = mpRefKFId;
		//record mnId
		VecMPmnId[linecnt] = mnId;

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

	//evaluate refId and mnId vector
	_vRefKFIdInMP = vRefKFIdInMP;
	_VecMPmnId = VecMPmnId;

	//
	cout<<"total "<<linecnt<<" MapPoints loaded."<<endl;
	if(linecnt!=mpSaveCnt)
		cerr<<"linecnt != mpSaveCnt, shouldn't"<<endl;

	//close file
	ifs.close();
	ifGlobal.close();

    delete tmpKF;
	
	return true;
}

bool loadKFVariables(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc,
	MapKFIndexPointer *kfIdxPtMap, VecUL &_VecKFmnId)
{
	ifstream ifkfVar,ifkfKeys,ifkfKeysUn,ifkfDes,ifGlobal;
	if(	!myOpenFile(ifkfVar,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.txt")) 	||
		!myOpenFile(ifkfKeys,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.txt")) 	||
		!myOpenFile(ifkfKeysUn,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.txt")) 	||
		!myOpenFile(ifkfDes,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.txt"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"))   )
	{	
		return false;
	}

	//save mappoint id in each KF
	long unsigned int kfSaveCnt,mpSaveCnt;
	vector<VecUL > VecKFmnId;

	long unsigned int gnNextIdKF,gnNExtIdMP;
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
	ssg<<slg;
	ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt;
	cout<<"total "<<mpSaveCnt<<" MapPoints saved."<<endl;
	cout<<"total "<<mpSaveCnt<<" KeyFrames saved."<<endl;
	VecKFmnId.resize(kfSaveCnt);
	//Line1, KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy
	getline(ifGlobal, slg);	
	ssg<<slg;
	ssg>>gnNextIdKF>>mfGridElementWidthInv>>mfGridElementHeightInv>>fx>>fy>>cx>>cy>>mnMinX>>mnMinY>>mnMaxX>>mnMaxY;
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
	if(ssg.fail())
		cerr<<"ssg fail. shouldn't"<<endl;
	}

	unsigned long int nNextId;

	// create a temperary Frame, for global or static params of KeyFrames
	Frame tmpFrame;
	Frame::fx = fx;
	Frame::fy = fy;
	Frame::cx = cx;
	Frame::cy = cy;
	Frame::mfGridElementWidthInv = mfGridElementWidthInv;
	Frame::mfGridElementHeightInv = mfGridElementHeightInv;
	Frame::mnMinX = mnMinX;
	Frame::mnMinY = mnMinY;
	Frame::mnMaxX = mnMaxX;
	Frame::mnMaxY = mnMaxY;
	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(tmpFrame.mK);
	tmpFrame.mpORBvocabulary = mpvoc;
	tmpFrame.mnScaleLevels = mnScaleLevelsOther;
	tmpFrame.mvScaleFactors = mvScaleFactorsOther;
	tmpFrame.mvLevelSigma2 = mvLevelSigma2Other;
	tmpFrame.mvInvLevelSigma2 = mvInvLevelSigma2Other;

	// KeyFrame 0&1 is different. ORBextractor settings are fixed as 2000/1.2/8
	Frame tmpFrame0(tmpFrame);
	tmpFrame0.mnScaleLevels = mnScaleLevels0;
	tmpFrame0.mvScaleFactors = mvScaleFactors0;
	tmpFrame0.mvLevelSigma2 = mvLevelSigma20;
	tmpFrame0.mvInvLevelSigma2 = mvInvLevelSigma20;

	// read each row of the files
	unsigned long int linecnt=0;
	while(!ifkfVar.eof())
	{	
		string slVar,slKeys,slKeysUn,slDes;
		stringstream ssVar,ssKeys,ssKeysUn,ssDes;

		//1 kfVariables.txt
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
		if(ssVar.fail())
			cerr<<"ssVar fail. shouldn't"<<endl;


		//new keyframe from tmp frame
		KeyFrame* tmpKF;
		if(mnId>=0 && mnId<=1)
			tmpKF = new KeyFrame(tmpFrame0, wd, db);	
		else
			tmpKF = new KeyFrame(tmpFrame, wd, db);

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

		//2 kfKeyPoints.txt
		{
		getline(ifkfKeys,slKeys);
		ssKeys << slKeys;
		size_t kpn;
		ssKeys >> kpn;
		vector<cv::KeyPoint> tmvKeys(kpn);
		for(vector<cv::KeyPoint>::iterator vit=tmvKeys.begin(), vend=tmvKeys.end();vit!=vend;vit++)
		{				
			float ptx,pty,size,angle,response;
			int octave,classid;
			ssKeys >> ptx>>pty>>size>>angle>>response>>octave>>classid;
			*vit = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			if(ssKeys.fail())
				cerr<<"ssKeys fail. shouldn't"<<endl;
		}
		tmpKF->SetKeyPoints(tmvKeys);
//		std::vector<cv::KeyPoint> mvKeys;
		}

		//2 kfKeyPointsUn.txt
		{
		getline(ifkfKeysUn,slKeysUn);
		ssKeysUn << slKeysUn;
		size_t kpun;
		ssKeysUn >> kpun;
		vector<cv::KeyPoint> tmvKeysUn(kpun);
		for(vector<cv::KeyPoint>::iterator vit=tmvKeysUn.begin(), vend=tmvKeysUn.end();vit!=vend;vit++)
		{				
			float ptx,pty,size,angle,response;
			int octave,classid;
			ssKeysUn >> ptx>>pty>>size>>angle>>response>>octave>>classid;
			*vit = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			if(ssKeysUn.fail())
				cerr<<"ssKeysUn fail. shouldn't"<<endl;
		}
		tmpKF->SetKeyPointsUn(tmvKeysUn);
//		std::vector<cv::KeyPoint> mvKeysUn;
		}

		//3 kfDescriptors.txt
		{
		getline(ifkfDes,slDes);
		ssDes << slDes;
		size_t ndes;
		ssDes >> ndes;
		cv::Mat matDes = cv::Mat::zeros(ndes,32,cv::CV_8U);
		for(int ti=0;ti<ndes;ti++)
		{
			cv::Mat tmpDes = cv::Mat::zeros(1,32,cv::CV_8U);
			uint32_t *pdes = tmpDes.ptr<uint32_t>();
			for(int i=0;i<8;i++)
			{
				uint32_t tmpi;
				ssDes>>tmpi;
				pdes[i]=tmpi;
				if(ssDes.fail())
					cerr<<"ssDes fail. shouldn't"<<endl;
			}
			tmpDes.copyTo(matDes.row(ti));
		}
		tmpKF->SetDescriptors(matDes);
		tmpKF->ComputeBoW();	//get mBowVec and mFeatVec
//		cv::Mat mDescriptors;
//		DBoW2::BowVector mBowVec;
//		DBoW2::FeatureVector mFeatVec;
		}


		//after all mappoints loaded
		//4. kfMapPointsID
//		std::vector<MapPoint*> mvpMapPoints;

		//after all keyframes loaded
		//5. kfLoopEdges, load kf id
//		std::set<KeyFrame*> mspLoopEdges;

		//after all keyframes and mappoints loaded
//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
//		std::vector<int> mvOrderedWeights;
//		KeyFrame* mpParent;
//		std::set<KeyFrame*> mspChildrens;

		//ignored. neither saved or loaded
//		cv::Mat im;


		//record the mnId order
		VecKFmnId[linecnt] = mnId;

		
		// add to the mapping from index to pointer
		if(kfIdxPtMap->count(mnId)>0)
			cerr<<"exist? shouldn't!!"<<endl;
		(*kfIdxPtMap)[mnId] = tmpKF;

		linecnt++;
	}

	//evaluate mnId vector
	_VecKFmnId = VecKFmnId;

	//
	cout<<"total "<<linecnt<<" KeyFrames loaded."<<endl;
	if(linecnt!=kfSaveCnt)
		cerr<<"linecnt != kfSaveCnt, shouldn't"<<endl;

	//close file
	ifkfVar.close();
	ifkfKeys.close();
	ifkfKeysUn.close();
	ifkfDes.close();
	ifGlobal.close();

	return true;
}


void loadMPKFPointers(const MapMPIndexPointer &mpIdxPtMap, const MapKFIndexPointer &kfIdxPtMap,
		const VecUL& VecKFmnId, const VecUL& VecMPmnId, const VecUL &vRefKFIdInMP)
{
	ifstream ifkfMPids,ifkfLPEGs,ifGlobal,ifmpObs;
	if(	!myOpenFile(ifkfMPids,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.txt"))	||
		!myOpenFile(ifkfLPEGs,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfLoopEdges.txt"))   	||
		!myOpenFile(ifmpObs,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpObservations.txt"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"))   	)
	{	
		return false;
	}
	string slg,slMPids,slLPEGs,slObs;
	stringstream ssg,ssMPids,ssLPEGs,ssObs;
	
	long unsigned int mpSaveCnt,kfSaveCnt;
	{
	long unsigned int gnNExtIdMP;
	getline(ifGlobal,slg);
	ssg << slg;
	ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt;
	}

	//------------------------------
	//1 MapPoint pointers
	if(VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || MapMPIndexPointer.size()!=mpSaveCnt)
		cerr<<"VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || MapMPIndexPointer.size()!=mpSaveCnt, shouldn't"<<endl;
	// MapPoint mnId , line order in file
	for(size_t i=0;i<mpSaveCnt;i++)
	{
		long unsigned int tMPmnid = VecMPmnId[i];	
		MapPoint* pMP = mpIdxPtMap[tMPmnid];	//MapPoint pointer of mnId
		pMP->SetRefKFPointer(vRefKFIdInMP[i]);	//set mpRefKF
		//	tmpMP.mpRefKF;

		// observation data
		getline(ifmpObs, slObs);
		ssObs<<slObs;
		size_t obN;
		ssObs>>obN;	//number of observations of this mappoint
		if(obN==0)	cerr<<"obN==0, shouldn't"<<endl;
		for(size_t j=0;j<obN;j++)
		{
			long unsigned int kfIdj; size_t obIdj;	//mnId of KF see this MP, and id of observation
			ssObs>>kfIdj>>obIdj;
			KeyFrame* pKF = kfIdxPtMap[kfIdj];	//KF
			pMP->AddObservation(pKF,obIdj)		//add observation
		}
		//tmpMP.mObservations;
	}


	//------------------------------
	//2 KeyFrame pointers
	for(long unsigned int linecnt=0;linecnt<kfSaveCnt;linecnt++)
	{
		//mnId corresponding to this line in file
		long unsigned int kfmnId=VecKFmnId[linecnt];
		//corresponding KeyFrame
		KeyFrame* pKF = kfIdxPtMap[kfmnId];
		
		//after all mappoints loaded
		//4. kfMapPointsID, for pKF->mvpMapPoints
		{
		getline(ifkfMPids,slMPids);
		ssMPids << slMPids;
		size_t nMPid,tvpMPidx;
		long unsigned int kfmnIdread,tmpid;
		ssMPids >> kfmnIdread >> nMPid;
		if(kfmnId!=kfmnIdread)	cerr<<"mpid: kfmnId!=VecKFmnId[linecnt], shouldn't"<<endl;
		if(nMPid==0) 	cerr<<"line " << linecnt<<" nMPid=0. shouldn't"<<endl;
		//KeyPoint number, size of mvpMapPoints
		size_t kpN = pKF->mvKeys.size();
		vector<MapPoint*> mvpMPs = vector<MapPoint*>(kpN,static_cast<MapPoint*>(NULL));
		pKF->SetmvpMapPoints(mvpMPs);	//init mvpMapPoint as NULL (size = KeyPoints.size())
		//for each KeyFrame, set 
		for(size_t i=0;i<nMPid;i++)
		{
			ssMPids>>tmpid>>tvpMPidx;
			MapPoint* pMP = mpIdxPtMap[tmpid];
			pKF->AddMapPoint(pMP,tvpMPidx);
			if(tvpMPidx!=pMP->GetIndexInKeyFrame(pKF))	
				cerr<<tvpMPidx<<" "<<pMP->GetIndexInKeyFrame(pKF)<<"\ntvpMPidx!=pMP->GetIndexInKeyFrame(pKF), shouldn't"<<endl;
			if(ssMPids.fail())	cerr<<"ssMPids fail. shouldn't"<<endl;
		}
		//pKF->mvpMapPoints
		//		std::vector<MapPoint*> mvpMapPoints;
		}

		//after all keyframes loaded
		//5. kfLoopEdges, load kf id
		{
		getline(ifkfLPEGs,slLPEGs);
		ssLPEGs << slLPEGs;
		long unsigned int kfmnIdread;
		size_t nLPEGid;
		ssLPEGs>>kfmnIdread>>nLPEGid;
		if(kfmnId!=kfmnIdread) cerr<<"lpeg: kfmnId!=VecKFmnId[linecnt], shouldn't"<<endl;
		for(size_t i=0;i<nLPEGid;i++)
		{
			long unsigned int tkfmnId;
			ssLPEGs>>tkfmnId;
			pKF->AddLoopEdge(kfIdxPtMap[tkfmnId]);
		}
		//pKF->mspLoopEdges
		//		std::set<KeyFrame*> mspLoopEdges;
		}

	}


	
	//after all keyframes and mappoints loaded
	for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
	{
		KeyFrame* pKFm=mit->second;
		pKFm->UpdateConnections();
	}
	//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
	//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
	//		std::vector<int> mvOrderedWeights;
	//		KeyFrame* mpParent;
	//		std::set<KeyFrame*> mspChildrens;

	ifkfMPids.close();
	ifkfLPEGs.close();
	ifmpObs.close();
	ifGlobal.close();
}

bool loadKFDatabase(KeyFrameDatabase *db)
{
//	ifstream ifs;
//	string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"KeyFrameDatabase.txt";
//	ifs.open(strFile.c_str());
//	if(!ifs.is_open() || ifs.eof())
//	{
//		cout<<"database file open failed."<<endl;
//		return false;
//	}

//	db->add(

	return true;
}


void SaveWorldToFile(const Map& World)
{
	string strFile;
	long unsigned int mpSaveCnt,kfSaveCnt;
	mpSaveCnt=0;
	kfSaveCnt=0;

//	//------------------------------------------------
//	//save world data
//	{
//	ofstream fwd;
//	cout << endl << "Saving World data" << endl;
//	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"WorldData.txt";
//	fwd.open(strFile.c_str());
//	fwd<<fixed;
//	fwd<<World.mnMaxKFid<<World.mbMapUpdated<<endl;
//	
//	}
	
	//------------------------------------------------
	//save keyframe database
	{
	ofstream f;
	cout << endl << "Saving KeyFrameDatabase" << endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"KeyFrameDatabase.txt";
	f.open(strFile.c_str());
	f << fixed;
	int tmpIdx = 0;
	for(std::vector<list<KeyFrame*> >::iterator vit=Database.mvInvertedFile.begin(), vend=Database.mvInvertedFile.end(); vit!=vend; vit++, tmpIdx++)
	{
		if(vit==Database.mvInvertedFile.begin() || vit==(Database.mvInvertedFile.begin()+1))
			cout<<tmpIdx<<endl;
	
		list<KeyFrame*> plKF = *vit;
		int listsize = plKF.size();
		if(listsize > 0)	//only save the word seen in KeyFrames.
		{
			f << tmpIdx << " " << listsize << " ";	//save wordID,	and number of KFs see this word
			for(list<KeyFrame*>::iterator lit=plKF.begin(), lend=plKF.end(); lit!=lend; lit++)
			{
				KeyFrame* pKFi = *lit;
				f << pKFi->mnId <<" ";				//save ID of KFs see the word
			}
			f << endl;
		}
	}
	f.close();
	}

	//------------------------------------------------
	/*
	save mappoint files
	*/
	{
	ofstream fmpVar,fmpObs;
	cout << endl << "Saving MapPoint" << endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.txt";
	fmpVar.open(strFile.c_str());
	fmpVar << fixed;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpObservations.txt";
	fmpObs.open(strFile.c_str());
	fmpObs << fixed;
	
	int tmpIdx=0;
	vector<MapPoint*> vMapPoints = World.GetAllMapPoints();
//	  bool printflag=true;
	for(std::vector<MapPoint*>::iterator vit=vMapPoints.begin(), vend=vMapPoints.end(); vit!=vend; vit++, tmpIdx++)
	{
		MapPoint* pMPi = *vit;
		if(!pMPi->isBad())	//only save those not bad
		{
			// 1. save plain variable
			// public, 15
			fmpVar << pMPi->nNextId << " ";
			fmpVar << pMPi->mnId << " ";
			fmpVar << pMPi->mnFirstKFid << " ";
			fmpVar << pMPi->mTrackProjX << " ";
			fmpVar << pMPi->mTrackProjY << " ";
			fmpVar << pMPi->mbTrackInView << " ";
			fmpVar << pMPi->mnTrackScaleLevel << " ";
			fmpVar << pMPi->mTrackViewCos << " ";
			fmpVar << pMPi->mnTrackReferenceForFrame << " ";
			fmpVar << pMPi->mnLastFrameSeen << " ";
			fmpVar << pMPi->mnBALocalForKF << " ";
			fmpVar << pMPi->mnFuseCandidateForKF << " ";
			fmpVar << pMPi->mnLoopPointForKF << " ";
			fmpVar << pMPi->mnCorrectedByKF << " ";
			fmpVar << pMPi->mnCorrectedReference << " ";
			// protected
			fmpVar << setprecision(10);
			cv::Mat twp = pMPi->GetWorldPos();
			fmpVar << twp.at<float>(0) <<" "<< twp.at<float>(1) <<" "<< twp.at<float>(2) <<" ";
			fmpVar << setprecision(7);
			cv::Mat tnv = pMPi->GetNormal();
			fmpVar << tnv.at<float>(0) <<" "<< tnv.at<float>(1) <<" "<< tnv.at<float>(2) <<" ";
			cv::Mat tdes = pMPi->GetDescriptor();	//256b, 8*uint32_t
			const uint32_t *tpdes = tdes.ptr<uint32_t>();
			for(int ti=0; ti<8; ti++)
			{
				fmpVar << tpdes[ti] <<" ";
			}
//			  if(printflag)
//			  {
//				  const unsigned char *tp = tdes.ptr();
//				  for(int ti=0;ti<32;ti++)
//					  cout<<(int)tp[ti]<<" ";
//				  cout<<endl;
//				  cout<<"cout worldpos: "<<twp<<endl;
//				  cout<<"normal vector: "<<tnv<<endl;
//				  cout<<"cout descriptor: "<<tdes<<endl;
//				  printflag = false;
//			  }
			fmpVar << pMPi->GetmnVisible() <<" ";
			fmpVar << pMPi->GetmnFound() <<" ";
			fmpVar << pMPi->GetMinDistanceInvariance() <<" ";
			fmpVar << pMPi->GetMaxDistanceInvariance() <<" ";
			fmpVar << pMPi->GetReferenceKeyFrame()->mnId <<" ";
			fmpVar << std::endl;

			// 2. save observations
			map<KeyFrame*,size_t> observations = pMPi->GetObservations();
			int nObs = observations.size();
			fmpObs << nObs << " ";	//total number 
			for(std::map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
			{
				KeyFrame* pKFm = mit->first;
				size_t idMPinKF = mit->second;
				fmpObs << pKFm->mnId << " " << idMPinKF << " "; //KF id and MP index in KF
			}

			fmpObs << std::endl;

			mpSaveCnt++;
		}
		
	}

	fmpVar.close();
	fmpObs.close();
	
	cout<<"total "<<mpSaveCnt<<" MapPoints saved."<<endl;
	if(mpSaveCnt==tmpIdx)
		cerr<<"mpSaveCnt = tmpIdx. impossible!"<<endl;
	}


 
	//------------------------------------------------
	/*
	save keyframe files
	*/
	{
	ofstream fkfVar,fkfKeys,fkfKeysUn,fkfDes,fkfMPids,fkfLPEGs;
	cout << endl << "Saving KeyFrames" << endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.txt";
	fkfVar.open(strFile.c_str());
	fkfVar << fixed;
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.txt";
	fkfKeys.open(strFile.c_str());
	fkfKeys << fixed;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.txt";
	fkfKeysUn.open(strFile.c_str());
	fkfKeysUn << fixed;
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.txt";
	fkfDes.open(strFile.c_str());
	fkfDes << fixed;

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.txt";
	fkfMPids.open(strFile.c_str());
	fkfMPids << fixed;

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfLoopEdges.txt";
	fkfLPEGs.open(strFile.c_str());
	fkfLPEGs << fixed;

	
	fkfVar << setprecision(7);
	fkfKeys << setprecision(7);
	fkfKeysUn << setprecision(7);
	fkfDes << setprecision(7);
	fkfMPids << setprecision(7);

	int tmpIdx=0;
	vector<KeyFrame*> vKeyFrames = World.GetAllKeyFrames();
	bool printflag=true;
	for(std::vector<KeyFrame*>::iterator vitKFs=vKeyFrames.begin(), vendKFs=vKeyFrames.end(); vitKFs!=vendKFs; vitKFs++, tmpIdx++)
	{
		KeyFrame* pKFi = *vitKFs;
		if(!pKFi->isBad())
		{
			//1. save plain variables
			// public
			fkfVar << setprecision(7);
			fkfVar << pKFi->nNextId <<" ";
			fkfVar << pKFi->mnId <<" ";
			fkfVar << pKFi->mnFrameId <<" ";
			fkfVar << pKFi->mTimeStamp <<" ";
			fkfVar << pKFi->mnTrackReferenceForFrame <<" ";
			
			fkfVar << pKFi->mnFuseTargetForKF <<" ";
			fkfVar << pKFi->mnBALocalForKF <<" ";
			fkfVar << pKFi->mnBAFixedForKF <<" ";
			fkfVar << pKFi->mnLoopQuery <<" ";
			fkfVar << pKFi->mnLoopWords <<" ";
			
			fkfVar << pKFi->mLoopScore <<" ";
			fkfVar << pKFi->mnRelocQuery <<" ";
			fkfVar << pKFi->mnRelocWords <<" ";
			fkfVar << pKFi->mRelocScore <<" ";
			// protected
			fkfVar << setprecision(10);
			cv::Mat Rcwi=pKFi->GetRotation();
			for(int ti=0;ti<3;ti++)
			{
				for(int tj=0;tj<3;tj++)
				{
					fkfVar << Rcwi.at<float>(ti,tj) <<" ";
				}
			}
			cv::Mat tcwi = pKFi->GetTranslation();
			for(int ti=0;ti<3;ti++)
			{
				fkfVar << tcwi.at<float>(ti) <<" ";
			}
			cv::Mat Owi = pKFi->GetCameraCenter();
			for(int ti=0;ti<3;ti++)
			{
				fkfVar << Owi.at<float>(ti) <<" ";
			}
			fkfVar << endl;

			// 2. save KeyPoints and KeyPointsUn
			vector<cv::KeyPoint> mvKeysi = pKFi->GetKeyPoints();
			fkfKeys << mvKeysi.size() <<" ";

			for(vector<cv::KeyPoint>::iterator vitkeys=mvKeysi.begin(), vendkeys=mvKeysi.end(); vitkeys!=vendkeys; vitkeys++)
			{
				cv::KeyPoint kpi = *vitkeys;
				fkfKeys << kpi.pt.x <<" "<< kpi.pt.y <<" "<< kpi.size <<" "<< kpi.angle <<" ";
				fkfKeys << kpi.response << " " << kpi.octave << " " << kpi.class_id <<" ";
			}
			fkfKeys <<endl;
			
			vector<cv::KeyPoint> mvKeysiUn = pKFi->GetKeyPointsUn();
			fkfKeysUn << mvKeysiUn.size() <<" ";
			for(vector<cv::KeyPoint>::iterator vitkeysun=mvKeysiUn.begin(), vendkeysun=mvKeysiUn.end(); vitkeysun!=vendkeysun; vitkeysun++)
			{
				cv::KeyPoint kpi = *vitkeysun;
				fkfKeysUn << kpi.pt.x <<" "<< kpi.pt.y <<" "<< kpi.size <<" "<< kpi.angle <<" ";
				fkfKeysUn << kpi.response << " " << kpi.octave << " " << kpi.class_id <<" ";
			}
			fkfKeysUn <<endl;

			// 3. save descriptors
			cv::Mat descriptorsi = pKFi->GetDescriptors();
			int desnumi = descriptorsi.rows;
			fkfDes << desnumi <<" ";	//number of descriptors
			for(int ti=0;ti<desnumi;ti++)
			{
				cv::Mat tdes = descriptorsi.row(ti);
				const uint32_t *tpdes = tdes.ptr<uint32_t>();
				for(int tj=0; tj<8; tj++)
				{
					fkfDes << tpdes[tj] <<" ";
				}
			}
			fkfDes <<endl;

			// 4. save mappoint id
			vector<MapPoint*> vpsi = pKFi->GetMapPointMatches(); 
			fkfMPids << pKFi->mnId << " "<<vpsi.size() <<" ";	//mnId & number of mappoints
			size_t scnt=0;
//			for(vector<MapPoint*>::iterator vit=vpsi.begin(), vend=vpsi.end(); vit!=vend; vit++,mvpMPidx++)
			for(size_t mvpMPidx=0, iend=vpsi.size();mvpMPidx<iend;mvpMPidx++)
			{
				MapPoint* vit = vpsi[mvpMPidx];
				if(!vit || vit->isBad()) continue;
				fkfMPids << vit->mnId <<" "<<mvpMPidx<<" ";
				scnt++;
			}
			fkfMPids <<endl;
			if(scnt!=pKFi->GetMapPoints().size())
				cerr<<"scnt!=pKFi->GetMapPoints().size(), shouldn't"<<endl;
	
			// 5. save loopedges
			set<KeyFrame*> lpedges = pKFi->GetLoopEdges();
			size_t nlpegs = lpedges.size();
			fkfLPEGs << pKFi->mnId << " "<<nlpegs<<" ";
			if(nlpegs>0)
			{
				for(set<KeyFrame*> sit=lpedges.begin(), send=lpedges.end(); sit!=send; sit++)
				{
					fkfLPEGs << sit->mnId <<" ";
				}
			}
			fkfLPEGs <<endl;
			
			// xxx. for test
			if(printflag && tmpIdx>=4)
			{
				cout<<"print for test"<<endl;
				for(int ti=0;ti<3;ti++)
				{
					for(int tj=0;tj<3;tj++) 				
						cout << Rcwi.at<float>(ti,tj) <<" ";
					cout<<endl;
				}
				for(int ti=0;ti<3;ti++)
					cout<<tcwi.at<float>(ti)<<" ";
				cout<<endl;
				cout<<"Pose Tcw: "<<pKFi->GetPose()<<endl;

				const unsigned char *tp = descriptorsi.row(0).ptr();
				for(int ti=0;ti<32;ti++)
					cout<<(int)tp[ti]<<" ";
				cout<<endl;
				cout<<"descriptor0/0: "<<descriptorsi.row(0)<<endl;
				
				printflag=false;
			}

			kfSaveCnt++;
			
		}
		
	}
	
	fkfVar.close();
	fkfKeys.close();
	fkfKeysUn.close();
	fkfDes.close();
	fkfMPids.close();
	fkfLPEGs.close();

	cout<<"total "<<kfSaveCnt<<" KeyFrames saved."<<endl;
	if(kfSaveCnt==tmpIdx)
		cerr<<"kfSaveCnt = tmpIdx. impossible!"<<endl;
	}

	
	//------------------------------------------------
	//save global parameters
	{
	ofstream f;
	cout<<endl<<"Saving global params"<<endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt";
	f.open(strFile.c_str());
	f<<fixed;
	f<<setprecision(10);
	
	//MapPoint data
	//Line0. MP.nNextID
	f<<MapPoint::nNextId<< " "<<mpSaveCnt<<" "<<kfSaveCnt<<" "<<endl;
	
	//KeyFrame data
	vector<KeyFrame*> vpKFt = World.GetAllKeyFrames();
	KeyFrame* pKF0 = vpKFt[0];
	//Line1.  KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy,
	//(cont.) mnMinX,mnMinY,mnMaxX,mnMaxY,mK
	f<<KeyFrame::nNextId<<" ";
	f<<pKF0->mfGridElementWidthInv<<" "<<pKF0->mfGridElementHeightInv<<" ";
	f<<pKF0->fx<<" "<<pKF0->fy<<" "<<pKF0->cx<<" "<<pKF0->cy<<" ";
	vector<int> tv=pKF0->GetMinMaxXY();
	f<<tv[0]<<" "<<tv[1]<<" "<<tv[2]<<" "<<tv[3]<<" ";
	
	f<<endl;
	//Line2. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for the first 2 KFs. 
	f<<pKF0->GetScaleLevels()<<" ";
	vector<float> sfactors =  pKF0->GetScaleFactors();
	for(int i=0;i<sfactors.size();i++)
		f<<sfactors[i]<<" ";
	vector<float> lsigma2 = pKF0->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
	for(int i=0;i<lsigma2.size();i++)
		f<<lsigma2[i]<<" ";
	f<<endl; 
	//Line3. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
	KeyFrame* pKFg;
	for(vector<KeyFrame*>::iterator kfit=vpKFt.begin(), kfend=vpKFt.end(); kfit!=kfend; kfit++)
	{
		pKFg=*kfit;
		if(pKFg->mnId>2 && pKFg && !pKFg.isBad())
			break;
	}
	f<<pKFg->GetScaleLevels()<<" ";
	vector<float> sfactors =  pKFg->GetScaleFactors();
	for(int i=0;i<sfactors.size();i++)
		f<<sfactors[i]<<" ";
	vector<float> lsigma2 = pKFg->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
	for(int i=0;i<lsigma2.size();i++)
		f<<lsigma2[i]<<" ";
	f<<endl;

	//other data

	//f<<endl;
	
	f.close();
	}

}


bool LoadWroldFromFile(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc)
{
	MapMPIndexPointer mpIdxPtMap;
	MapKFIndexPointer kfIdxPtMap;
	VecUL vRefKFIdInMP;
	VecUL vKFmnId,vMPmnId;
	bool ret,ret1,ret2,ret3,ret4;

	//step 1. load and create all mappoints
	ret1=loadMPVariables(db,wd,&mpIdxPtMap,vMPmnId,vRefKFIdInMP);

	//step 2. load and craete all keyframes
	ret2=loadKFVariables(db,wd,mpvoc,&kfIdxPtMap,vKFmnId);

	//step 3. associate pointers in MPs and KFs
	ret3=loadMPKFPointers(mpIdxPtMap, kfIdxPtMap, vRefKFIdInMP);

	//step 4. associate pointers in invertfile of vocabulary
	ret4=loadKFDatabase(db);

	//step 5. world
	if(ret1&&ret2&&ret3&&ret4)
	{
		for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddKeyFrame(mit->second);
		}
		for(MapMPIndexPointer::iterator mit=mpIdxPtMap.begin(), mend=mpIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddMapPoint(mit->second);
		}
	}	

	return (ret1&&ret2&&ret3&&ret4);
}


#endif

