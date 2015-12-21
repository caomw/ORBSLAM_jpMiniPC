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
#include <LocalMapping.h>

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

//	// load voc-keyframe invert index, after keyframe data is loaded
//bool loadKFDatabase(KeyFrameDatabase *db, MapKFIndexPointer &kfIdxPtMap);

// associate pointers in MPs and KFs
bool loadMPKFPointers(MapMPIndexPointer &mpIdxPtMap, MapKFIndexPointer &kfIdxPtMap,
                      const VecUL& VecKFmnId, const VecUL& VecMPmnId, const VecUL &vRefKFIdInMP);


// load all data. main function
bool LoadWroldFromFile(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc, KeyFrame *pLastKF);


// --------------------------------------------------
// --------------------------------------------------
// --------------------------------------------------

/*
static bool myOpenFile(ifstream &ifs, string strFile)
{
	ifs.open(strFile.c_str());
	if(!ifs.is_open())
	{
		cout<<strFile<<" open failed."<<endl;
		return false;
	}
	if(ifs.eof())
	{
		cout<<strFile<<" empty."<<endl;
		ifs.close();
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
	long unsigned int kfSaveCnt,mpSaveCnt,frameNextId,kfNextId;
	{
	long unsigned int gnNExtIdMP;
	string slg;
	stringstream ssg;
	//Line0, MP.nNextId
	getline(ifGlobal, slg); 
	ssg<<slg;
    ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt>>frameNextId>>kfNextId;
	
	MapPoint::nNextId = gnNExtIdMP;
	Frame::nNextId = frameNextId;
	KeyFrame::nNextId = kfNextId;
	}

	//record the reference KF's mnId, in the order of lines saved in file
	VecUL vRefKFIdInMP(mpSaveCnt);
	VecUL VecMPmnId(mpSaveCnt);

	Frame tmpFrame;
    tmpFrame.mTcw = Mat::zeros(4,4,CV_32F);
    KeyFrame * tmpKF = new KeyFrame(tmpFrame, wd, db);

	unsigned long int linecnt=0;
    for(unsigned long int smpcnt=0;smpcnt<mpSaveCnt;smpcnt++)
    {
		string sl;
		stringstream ss;
		
        getline(ifs,sl);
		ss << sl;

		// plain variables in MapPoint
		unsigned long int nNextId;
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
        if(ss.fail()) cerr<<"ssfail in mpVariables, shouldn't."<<endl;
		if(mnFirstKFid<0) cerr<<"mnFirstKFid<0, shouldn't."<<endl;

		// new MapPoint memory space and pointer
        MapPoint* tmpMP = new MapPoint(mWorldPos, tmpKF, wd);
        MapPoint::nNextId = nNextId;

		//public
		tmpMP->mnId = mnId;
		tmpMP->mnFirstKFid = mnFirstKFid;
		
		tmpMP->mTrackProjX = mTrackProjX;
		tmpMP->mTrackProjY = mTrackProjY;
		tmpMP->mbTrackInView = mbTrackInView;
		tmpMP->mnTrackScaleLevel = mnTrackScaleLevel;
		tmpMP->mTrackViewCos = mTrackViewCos;
		
		tmpMP->mnTrackReferenceForFrame = 0;//mnTrackReferenceForFrame;
		tmpMP->mnLastFrameSeen = 0;//mnLastFrameSeen;
		
		tmpMP->mnBALocalForKF = 0;//mnBALocalForKF;
		tmpMP->mnFuseCandidateForKF = 0;//mnFuseCandidateForKF;
		
		tmpMP->mnLoopPointForKF = 0;//mnLoopPointForKF;
		tmpMP->mnCorrectedByKF = 0;//mnCorrectedByKF;
		tmpMP->mnCorrectedReference = 0;//mnCorrectedReference;
		//protected
		tmpMP->SetWorldPos(mWorldPos);
//		tmpMP->SetNormalVec(mNormalVector);		//set by UpdateNormalAndDepth(). need mObservations/mpRefKF/mWorldPos and KF.Tcw
		tmpMP->SetDescriptor(mDescriptor);
//		tmpMP->SetmnVisible(1);//(mnVisible);
//      tmpMP->SetmnFound(1);//(mnFound);
//		tmpMP->SetMinDistance(mfMaxDistance);
//		tmpMP->SetMaxDistance(mfMaxDistance);


		//record reference KF id of this mappoint
		vRefKFIdInMP[linecnt] = mpRefKFId;
		//record mnId
		VecMPmnId[linecnt] = mnId;

		// add to the mapping from index to pointer
		if(mpIdxPtMap->count(mnId)>0)
            cerr<<mnId<<" mappoint count "<< mpIdxPtMap->count(mnId)<<"exist? shouldn't!!"<<endl;
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
    long unsigned int kfSaveCnt,mpSaveCnt,tmp1,tmp2;
    VecUL VecKFmnId;

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
	//Line1, MP.nNextId
	getline(ifGlobal, slg);	
	ssg<<slg;
    ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt>>tmp1>>tmp2;
	cout<<"total "<<mpSaveCnt<<" MapPoints saved."<<endl;
	cout<<"total "<<mpSaveCnt<<" KeyFrames saved."<<endl;
	VecKFmnId.resize(kfSaveCnt);
	//Line2, KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy
	getline(ifGlobal, slg);	
	ssg<<slg;
	ssg>>gnNextIdKF>>mfGridElementWidthInv>>mfGridElementHeightInv>>fx>>fy>>cx>>cy>>mnMinX>>mnMinY>>mnMaxX>>mnMaxY;
	if(gnNextIdKF!=tmp2) cerr<<"2 nNextId of KF is different, shouldn't"<<endl;
	//Line3, mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for the first 2 KFs. 
	getline(ifGlobal, slg);	
	ssg<<slg;
	ssg>>mnScaleLevels0;	cout<<mnScaleLevels0<<" ";
	mvScaleFactors0.resize(mnScaleLevels0);
	mvLevelSigma20.resize(mnScaleLevels0);
	mvInvLevelSigma20.resize(mnScaleLevels0);
	for(int i=0;i<mnScaleLevels0;i++)
	{
		ssg>>mvScaleFactors0[i];		cout<<mvScaleFactors0[i]<<" ";
	}
	for(int i=0;i<mnScaleLevels0;i++)
	{
		ssg>>mvLevelSigma20[i];			cout<<mvLevelSigma20[i]<<" ";
		mvInvLevelSigma20[i]=1/mvLevelSigma20[i];	cout<<mvInvLevelSigma20[i]<<" ";
	}
	//Line4, mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
	getline(ifGlobal, slg);
	ssg<<slg;
	ssg>>mnScaleLevelsOther;	cout<<mnScaleLevelsOther<<endl;
	mvScaleFactorsOther.resize(mnScaleLevelsOther);
	mvLevelSigma2Other.resize(mnScaleLevelsOther);
	mvInvLevelSigma2Other.resize(mnScaleLevelsOther);
	for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ssg>>mvScaleFactorsOther[i];		cout<<mvScaleFactorsOther[i]<<" ";
	}
	for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ssg>>mvLevelSigma2Other[i];			cout<<mvLevelSigma2Other[i]<<" ";
		mvInvLevelSigma2Other[i]=1/mvLevelSigma2Other[i];	cout<<mvInvLevelSigma2Other[i]<<" ";
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
    tmpFrame.mTcw = Mat::zeros(4,4,CV_32F);

	// KeyFrame 0&1 is different. ORBextractor settings are fixed as 2000/1.2/8
	Frame tmpFrame0(tmpFrame);
	tmpFrame0.mnScaleLevels = mnScaleLevels0;
	tmpFrame0.mvScaleFactors = mvScaleFactors0;
	tmpFrame0.mvLevelSigma2 = mvLevelSigma20;
	tmpFrame0.mvInvLevelSigma2 = mvInvLevelSigma20;

	// read each row of the files
	unsigned long int linecnt=0;
//    while(!ifkfVar.eof())
    for(unsigned long int skfcnt=0;skfcnt<kfSaveCnt;skfcnt++)
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

		ssVar>>nNextId>>mnId>>mnFrameId>>mTimeStamp;

		//To be deleted.
		ssVar>>mnTrackReferenceForFrame;
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
        {
			cerr<<"ssVar fail. shouldn't"<<endl;
            cout<<Rcwi<<endl;
            cout<<tcwi<<endl;
            cout<<Owi<<endl;

        }


		//new keyframe from tmp frame
		KeyFrame* tmpKF;
        if(mnId<=1) //mnId>=0 &&  always >=0
			tmpKF = new KeyFrame(tmpFrame0, wd, db);	
		else
			tmpKF = new KeyFrame(tmpFrame, wd, db);

        KeyFrame::nNextId = nNextId;
		//evaluate 
		tmpKF->mnId = mnId;
		tmpKF->mnFrameId = mnFrameId;
		tmpKF->mTimeStamp = mTimeStamp;


        tmpKF->mnTrackReferenceForFrame = 0;//mnTrackReferenceForFrame;
		tmpKF->mnFuseTargetForKF = 0;//mnFuseTargetForKF;
		tmpKF->mnBALocalForKF = 0;//mnBALocalForKF;
		tmpKF->mnBAFixedForKF = 0;//mnBAFixedForKF;
		tmpKF->mnLoopQuery = 0;//mnLoopQuery;
		tmpKF->mnLoopWords = 0;//mnLoopWords;
		tmpKF->mLoopScore = 0;//mLoopScore;
		tmpKF->mnRelocQuery = 0;//mnRelocQuery;
		tmpKF->mnRelocWords = 0;//mnRelocWords;
		tmpKF->mRelocScore = 0;//mRelocScore;
		
		tmpKF->SetPose(Rcwi,tcwi);

		//2 kfKeyPoints.txt
		{
		//		fkfKeys << kpi.pt.x <<" "<< kpi.pt.y <<" "<< kpi.size <<" "<< kpi.angle <<" ";
		//		fkfKeys << kpi.response << " " << kpi.octave << " " << kpi.class_id <<" ";
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
            if(ssKeys.fail())  cerr<<"loopcnt: "<<skfcnt<<" mnId "<<mnId<<" ssKeys fail. shouldn't"<<endl;
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
        cv::Mat matDes = cv::Mat::zeros(ndes,32,CV_8U);
        for(size_t ti=0;ti<ndes;ti++)
		{
            cv::Mat tmpDes = cv::Mat::zeros(1,32,CV_8U);
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
            cerr<<mnId<<" KF count "<<kfIdxPtMap->count(mnId)<<" exist? shouldn't!!"<<endl;
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


bool loadMPKFPointers(MapMPIndexPointer &mpIdxPtMap, MapKFIndexPointer &kfIdxPtMap,
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
	
    long unsigned int mpSaveCnt,kfSaveCnt,tmp1,tmp2;
	{
	string slg;
	stringstream ssg;
	long unsigned int gnNExtIdMP;
	getline(ifGlobal,slg);
	ssg << slg;
    ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt>>tmp1>>tmp2;
    if(ssg.fail()) cerr<<"ssg fail in loadMPKFPointers, shouldn't."<<endl;
	}

	//------------------------------
	//1 MapPoint pointers
    if(VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || mpIdxPtMap.size()!=mpSaveCnt)
		cerr<<"VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || MapMPIndexPointer.size()!=mpSaveCnt, shouldn't"<<endl;
	// MapPoint mnId , line order in file
	for(size_t i=0;i<mpSaveCnt;i++)
	{
		long unsigned int tMPmnid = VecMPmnId[i];	
		MapPoint* pMP = mpIdxPtMap[tMPmnid];	//MapPoint pointer of mnId
        KeyFrame* pkFref = kfIdxPtMap[vRefKFIdInMP[i]];    //Keyframe pointer to ReferenceKF
        pMP->SetRefKFPointer(pkFref);	//set mpRefKF
		//	tmpMP.mpRefKF;

		// observation data
		string slObs;
		stringstream ssObs;
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
            pMP->AddObservation(pKF,obIdj);		//add observation
		}
        if(ssObs.fail()) cerr<<"ssObs fail, shouldn't."<<endl;
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
		string slMPids;
		stringstream ssMPids;
		getline(ifkfMPids,slMPids);
		ssMPids << slMPids;
		size_t nMPid,tvpMPidx;
		long unsigned int kfmnIdread,tmpid;
		ssMPids >> kfmnIdread >> nMPid;
		if(kfmnId!=kfmnIdread)	cerr<<"mpid: kfmnId!=VecKFmnId[linecnt], shouldn't"<<endl;
		if(nMPid==0) 	cerr<<"line " << linecnt<<" nMPid=0. shouldn't"<<endl;
		//KeyPoint number, size of mvpMapPoints
        size_t kpN = pKF->GetKeyPoints().size();
		vector<MapPoint*> mvpMPs = vector<MapPoint*>(kpN,static_cast<MapPoint*>(NULL));
		pKF->SetmvpMapPoints(mvpMPs);	//init mvpMapPoint as NULL (size = KeyPoints.size())
		//for each KeyFrame, set 
		for(size_t i=0;i<nMPid;i++)
		{
			ssMPids>>tmpid>>tvpMPidx;
			MapPoint* pMP = mpIdxPtMap[tmpid];
			pKF->AddMapPoint(pMP,tvpMPidx);
            if((int)tvpMPidx!=pMP->GetIndexInKeyFrame(pKF))
				cerr<<tvpMPidx<<" "<<pMP->GetIndexInKeyFrame(pKF)<<"\ntvpMPidx!=pMP->GetIndexInKeyFrame(pKF), shouldn't"<<endl;
            if(ssMPids.fail()) cerr<<"kfmnId:"<<kfmnId<<" linecnt: "<<linecnt<<"ssMPids fail. shouldn't"<<endl;
		}
		//pKF->mvpMapPoints
		//		std::vector<MapPoint*> mvpMapPoints;
		}

        //after all keyframes loaded
        //5. kfLoopEdges, load kf id
        {
		string slLPEGs;
		stringstream ssLPEGs;
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
	
	//build spanning tree. in increasing order of KeyFrame mnId
	{
	unsigned long int preidx=0;
	for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
	{
		KeyFrame* pKFm=mit->second;
		pKFm->UpdateConnections();
		if(preidx>pKFm->mnId)
			cerr<<"KeyFrame pre Id > cur Id, shouldn't."<<endl;
		preidx=pKFm->mnId;
	}
	}
	//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
	//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
	//		std::vector<int> mvOrderedWeights;
	//		KeyFrame* mpParent;
	//		std::set<KeyFrame*> mspChildrens;

	//UpdateNormalAndDepth, need the mObservations/mpRefKF/mWorldPos
	{
	unsigned long int preidxmp=0;
	for(MapMPIndexPointer::iterator mit=mpIdxPtMap.begin(), mend=mpIdxPtMap.end(); mit!=mend; mit++)
	{
		MapPoint* pMPm=mit->second;
		pMPm->UpdateNormalAndDepth();
		if(preidxmp>pMPm->mnId)
			cerr<<"MapPoint pre Id > cur Id, shouldn't."<<endl;
		preidxmp=pMPm->mnId;
	}
	}
	
	ifkfMPids.close();
	ifkfLPEGs.close();
	ifmpObs.close();
	ifGlobal.close();

    return true;
}

//bool loadKFDatabase(KeyFrameDatabase *db, MapKFIndexPointer &kfIdxPtMap)
//{
//    ifstream ifs;
//    string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"KeyFrameDatabase.txt";
//    ifs.open(strFile.c_str());
//    if(!ifs.is_open() || ifs.eof())
//    {
//        cout<<"database file open failed."<<endl;
//        return false;
//    }

//	while(!ifs.eof())
//	{
//		string slkfdb;
//		stringstream sskfdb;
//		getline(ifs,slkfdb);
//		sskfdb<<slkfdb;
//		size_t listsize, tmpIdx;
//		sskfdb>>tmpIdx>>listsize;
//		for(size_t i=0;i<listsize;i++)
//		{
//			long unsigned int kfmnId;
//			sskfdb>>kfmnId;
//			db->add(kfIdxPtMap[kfmnId]);
//			if(sskfdb.fail()) cerr<<"sskfdb.fail(), shouldn't"<<endl;
//		}
//	}

//	return true;
//}


void SaveWorldToFile( Map& World, KeyFrameDatabase& Database)
{
	string strFile;
	long unsigned int mpSaveCnt,kfSaveCnt;
	mpSaveCnt=0;
	kfSaveCnt=0;
	
//	//1 1. save keyframe database
//	//1 -----------------------------------------------
//	{
//	ofstream f;
//	cout << endl << "Saving KeyFrameDatabase" << endl;
//	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"KeyFrameDatabase.txt";
//	f.open(strFile.c_str());
//	f << fixed;
//	int tmpIdx = 0;
//    for(std::vector<list<KeyFrame*> >::iterator vit=Database.mvInvertedFile.begin(), vend=Database.mvInvertedFile.end(); vit!=vend; vit++, tmpIdx++)
//	{
//		if(vit==Database.mvInvertedFile.begin() || vit==(Database.mvInvertedFile.begin()+1))
//			cout<<tmpIdx<<endl;
//	
//		list<KeyFrame*> plKF = *vit;
//		int listsize = plKF.size();
//		if(listsize > 0)	//only save the word seen in KeyFrames.
//		{
//			f << tmpIdx << " " << listsize << " ";	//save wordID,	and number of KFs see this word
//			for(list<KeyFrame*>::iterator lit=plKF.begin(), lend=plKF.end(); lit!=lend; lit++)
//			{
//				KeyFrame* pKFi = *lit;
//				f << pKFi->mnId <<" ";				//save ID of KFs see the word
//			}
//			f << endl;
//		}
//	}
//	f.close();
//	}
//	//1 -----------------------------------------------


	//1 2. save mappoint files
	//1 -----------------------------------------------
	{
	ofstream fmpVar,fmpObs;
	cout << endl << "Saving MapPoint" << endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.txt";
	fmpVar.open(strFile.c_str());
	fmpVar << fixed;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpObservations.txt";
	fmpObs.open(strFile.c_str());
	fmpObs << fixed;
	
    long unsigned int tmpIdx=0;
    vector<MapPoint*> vMapPoints = World.GetAllMapPoints();
//	  bool printflag=true;
    for(std::vector<MapPoint*>::iterator vit=vMapPoints.begin(), vend=vMapPoints.end(); vit!=vend; vit++, tmpIdx++)
	{
		MapPoint* pMPi = *vit;
		if(!pMPi)
			cerr<<"MapPoint pointer = NULL. shouldn't"<<endl;
		if(!pMPi->isBad())	//only save those not bad
		{
			//2 2.1 save plain variable
			//2 ---------------------------
			// public, 15
			{
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
			fmpVar << pMPi->GetmnVisible() <<" ";
			fmpVar << pMPi->GetmnFound() <<" ";
			fmpVar << pMPi->GetMinDistanceInvariance() <<" ";
			fmpVar << pMPi->GetMaxDistanceInvariance() <<" ";
			fmpVar << pMPi->GetReferenceKeyFrame()->mnId <<" ";
			fmpVar << std::endl;
			}
			//2 ---------------------------


			//2 2.2 save observations
			//2 ---------------------------
			{
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
			}
			//2 ---------------------------

			mpSaveCnt++;
		}
		
	}

	fmpVar.close();
	fmpObs.close();
	
	cout<<"total "<<mpSaveCnt<<" MapPoints saved."<<endl;
	}
	//1 -----------------------------------------------


 
	//1 3. save keyframe files
	//1 ------------------------------------------------
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

    long unsigned int tmpIdx=0;
	vector<KeyFrame*> vKeyFrames = World.GetAllKeyFrames();
	bool printflag=true;
	for(std::vector<KeyFrame*>::iterator vitKFs=vKeyFrames.begin(), vendKFs=vKeyFrames.end(); vitKFs!=vendKFs; vitKFs++, tmpIdx++)
	{
		KeyFrame* pKFi = *vitKFs;
		if(!pKFi)
			cerr<<"KeyFrame pointer = NULL, shouldn't."<<endl;
		if(!pKFi->isBad())
		{
			//2 3.1 save plain variables
			//2 -------------------------------
			// public
			fkfVar << setprecision(7);
			fkfVar << pKFi->nNextId <<" ";
			fkfVar << pKFi->mnId <<" ";
			fkfVar << pKFi->mnFrameId <<" ";
			fkfVar << pKFi->mTimeStamp <<" ";

			
			//To be deleted.
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
			//end deleted
			
			
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
			//2 -------------------------------

			//2 3.2 save KeyPoints and KeyPointsUn
			//2 -------------------------------
			{
			vector<cv::KeyPoint> mvKeysi = pKFi->GetKeyPoints();
			fkfKeys << mvKeysi.size() <<" ";

			for(vector<cv::KeyPoint>::iterator vitkeys=mvKeysi.begin(), vendkeys=mvKeysi.end(); vitkeys!=vendkeys; vitkeys++)
			{
				cv::KeyPoint kpi = *vitkeys;
				fkfKeys << kpi.pt.x <<" "<< kpi.pt.y <<" "<< kpi.size <<" "<< kpi.angle <<" ";
				fkfKeys << kpi.response << " " << kpi.octave << " " << kpi.class_id <<" ";
			}
			fkfKeys <<endl;
			}
			{
			vector<cv::KeyPoint> mvKeysiUn = pKFi->GetKeyPointsUn();
			fkfKeysUn << mvKeysiUn.size() <<" ";
			for(vector<cv::KeyPoint>::iterator vitkeysun=mvKeysiUn.begin(), vendkeysun=mvKeysiUn.end(); vitkeysun!=vendkeysun; vitkeysun++)
			{
				cv::KeyPoint kpi = *vitkeysun;
				fkfKeysUn << kpi.pt.x <<" "<< kpi.pt.y <<" "<< kpi.size <<" "<< kpi.angle <<" ";
				fkfKeysUn << kpi.response << " " << kpi.octave << " " << kpi.class_id <<" ";
			}
			fkfKeysUn <<endl;
			}
			//2 -------------------------------

			//2 3.3 save descriptors
			//2 -------------------------------
			{
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
			}
			//2 -------------------------------


			//2 3.4 save mappoint id
			//2 -------------------------------
			{
			vector<MapPoint*> vpsi = pKFi->GetMapPointMatches();
            size_t nMPcnt=0;
            //compute valid mappoint number
			for(size_t mvpMPidx=0, iend=vpsi.size();mvpMPidx<iend;mvpMPidx++)
			{
				MapPoint* vit = vpsi[mvpMPidx];
                if(vit)
					if(!vit->isBad()) 
            		{
            			nMPcnt++;
					}
			}

            fkfMPids << pKFi->mnId << " "<<nMPcnt <<" ";	//mnId & number of mappoints
            size_t scnt=0;
            for(size_t mvpMPidx=0, iend=vpsi.size();mvpMPidx<iend;mvpMPidx++)
            {
                MapPoint* vit = vpsi[mvpMPidx];
				if(vit)
					if(!vit->isBad())
					{
		                fkfMPids << vit->mnId <<" "<<mvpMPidx<<" ";
		                scnt++;
					}
            }
			fkfMPids <<endl;
			if(scnt!=pKFi->GetMapPoints().size() || scnt!=nMPcnt)
				cerr<<"scnt!=pKFi->GetMapPoints().size(), shouldn't"<<endl;
			}
			//2 -------------------------------

			
			//2 3.5 save loopedges
			//2 -------------------------------
			{
			set<KeyFrame*> lpedges = pKFi->GetLoopEdges();
			size_t nlpegs = lpedges.size();
			fkfLPEGs << pKFi->mnId << " "<<nlpegs<<" ";
			if(nlpegs>0)
			{
                for(set<KeyFrame*>::iterator sit=lpedges.begin(), send=lpedges.end(); sit!=send; sit++)
				{
                    KeyFrame* tpKFlpeg=(*sit);
                    fkfLPEGs << tpKFlpeg->mnId <<" ";
				}
			}
			fkfLPEGs <<endl;
			}
			//2 -------------------------------
			
//			//3 xxx. for test
//			if(printflag && tmpIdx>=4)
//			{
//				cout<<"print for test"<<endl;
//				for(int ti=0;ti<3;ti++)
//				{
//					for(int tj=0;tj<3;tj++) 				
//						cout << Rcwi.at<float>(ti,tj) <<" ";
//					cout<<endl;
//				}
//				for(int ti=0;ti<3;ti++)
//					cout<<tcwi.at<float>(ti)<<" ";
//				cout<<endl;
//				cout<<"Pose Tcw: "<<pKFi->GetPose()<<endl;

//				const unsigned char *tp = descriptorsi.row(0).ptr();
//				for(int ti=0;ti<32;ti++)
//					cout<<(int)tp[ti]<<" ";
//				cout<<endl;
//				cout<<"descriptor0/0: "<<descriptorsi.row(0)<<endl;
//				
//				printflag=false;
//			}

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
	}
	//1 ------------------------------------------------

	
	//1 4. save global parameters
	//1 ------------------------------------------------
	{
	ofstream f;
	cout<<endl<<"Saving global params"<<endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt";
	f.open(strFile.c_str());
	f<<fixed;
	f<<setprecision(10);

	
	//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
	//3 ------------------------------------------------
    f<<MapPoint::nNextId<< " "<<mpSaveCnt<<" "<<kfSaveCnt<<" "<<Frame::nNextId<<" "<<KeyFrame::nNextId<<" "<<endl;
	
	//KeyFrame data
	vector<KeyFrame*> vpKFt = World.GetAllKeyFrames();
	KeyFrame* pKF0 = vpKFt[0];
	//3 ------------------------------------------------
	
	//3 Line2.  KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy,
	//3 (cont.) mnMinX,mnMinY,mnMaxX,mnMaxY,mK
	//3 ------------------------------------------------
	f<<KeyFrame::nNextId<<" ";
	f<<pKF0->mfGridElementWidthInv<<" "<<pKF0->mfGridElementHeightInv<<" ";
	f<<pKF0->fx<<" "<<pKF0->fy<<" "<<pKF0->cx<<" "<<pKF0->cy<<" ";
	vector<int> tv=pKF0->GetMinMaxXY();
	f<<tv[0]<<" "<<tv[1]<<" "<<tv[2]<<" "<<tv[3]<<" ";
	f<<endl;
	//3 ------------------------------------------------
	
	//3 Line3. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for the first 2 KFs. 
	//3 ------------------------------------------------
	f<<pKF0->GetScaleLevels()<<" ";
	vector<float> sfactors =  pKF0->GetScaleFactors();
    for(size_t i=0;i<sfactors.size();i++)
		f<<sfactors[i]<<" ";
	vector<float> lsigma2 = pKF0->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
    for(size_t i=0;i<lsigma2.size();i++)
		f<<lsigma2[i]<<" ";
	f<<endl; 
	//3 ------------------------------------------------
	
	//3 Line4. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
	//3 ------------------------------------------------
    KeyFrame* pKFg=static_cast<KeyFrame*>(NULL);
	for(vector<KeyFrame*>::iterator kfit=vpKFt.begin(), kfend=vpKFt.end(); kfit!=kfend; kfit++)
	{
		pKFg=*kfit;
        if(pKFg)
            if(pKFg->mnId>2 && !pKFg->isBad())
                break;
	}
    if(!pKFg) cerr<<"pKFg=NULL, shouldn't"<<endl;
	f<<pKFg->GetScaleLevels()<<" ";
    vector<float> sfactorsg =  pKFg->GetScaleFactors();
    for(size_t i=0;i<sfactorsg.size();i++)
        f<<sfactorsg[i]<<" ";
    vector<float> lsigma2g = pKFg->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
    for(size_t i=0;i<lsigma2g.size();i++)
        f<<lsigma2g[i]<<" ";
	f<<endl;
	//3 ------------------------------------------------

	
	f.close();
	}
	//1 ------------------------------------------------

}


bool LoadWroldFromFile(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc, KeyFrame *pLastKF)
{
	MapMPIndexPointer mpIdxPtMap;
	MapKFIndexPointer kfIdxPtMap;
	VecUL vRefKFIdInMP;
	VecUL vKFmnId,vMPmnId;
    bool ret1,ret2,ret3,ret4;

    long unsigned int maxKFid=0;

	//1 step 1. load and create all mappoints
    cout<<"loading step 1.."<<endl;
	ret1=loadMPVariables(db,wd,&mpIdxPtMap,vMPmnId,vRefKFIdInMP);

	//1 step 2. load and craete all keyframes
    cout<<"loading step 2.."<<endl;
	ret2=loadKFVariables(db,wd,mpvoc,&kfIdxPtMap,vKFmnId);

	//1 step 3. associate pointers in MPs and KFs
    cout<<"loading step 3.."<<endl;
    ret3=loadMPKFPointers(mpIdxPtMap, kfIdxPtMap, vKFmnId, vMPmnId, vRefKFIdInMP );

	//1 step 4. associate pointers in invertfile of vocabulary
    cout<<"loading step 4.."<<endl;
//	ret4=loadKFDatabase(db, kfIdxPtMap);
	//to check
	for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
	{
		KeyFrame* pKF = mit->second;
		db->add(pKF);
        if(maxKFid<pKF->mnId)
            maxKFid = pKF->mnId;
	}

    //1 step5. evaluate nNextId for Frame/MapPoint/KeyFrame
    ifstream ifGlobal;
    ret4 = myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"));
    if(ret4)
    {
        cout<<"loading step 5.."<<endl;
        long unsigned int gnNExtIdMP,kfSaveCnt,mpSaveCnt,frameNextId,kfNextId;
        string slg;	stringstream ssg;
        getline(ifGlobal, slg);
        ssg<<slg;
        ssg>>gnNExtIdMP>>mpSaveCnt>>kfSaveCnt>>frameNextId>>kfNextId;

        MapPoint::nNextId = gnNExtIdMP;
        Frame::nNextId = frameNextId;
        KeyFrame::nNextId = kfNextId;
    }
    ifGlobal.close();

    //1 step 6. world
	if(ret1&&ret2&&ret3&&ret4)
	{
        cout<<"loading step 6.."<<endl;
		for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddKeyFrame(mit->second);
		}
		for(MapMPIndexPointer::iterator mit=mpIdxPtMap.begin(), mend=mpIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddMapPoint(mit->second);
		}
	}	

    pLastKF = kfIdxPtMap[maxKFid];

	
	return (ret1&&ret2&&ret3&&ret4);
}*/


static bool myOpenFile(ifstream &ifs, string strFile)
{
    ifs.open(strFile.c_str(), ios::in | ios::binary);
	if(!ifs.is_open())
	{
		cout<<strFile<<" open failed."<<endl;
		return false;
	}
	if(ifs.eof())
	{
		cout<<strFile<<" empty."<<endl;
		ifs.close();
		return false;
	}
	return true;
}


void SaveWorldToFile( Map& World, KeyFrameDatabase& Database)
{
    unsigned char saveHeader[2]={0xeb,0x90};
	
	string strFile;
	long unsigned int mpSaveCnt,kfSaveCnt;
	mpSaveCnt=0;
	kfSaveCnt=0;
	

	//1 2. save mappoint files
	//1 -----------------------------------------------
	{
	fstream fmpVar,fmpObs;
	cout << endl << "Saving MapPoint" << endl;
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.bin";
	fmpVar.open(strFile.c_str(), ios::out | ios::binary);
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpObservations.bin";
	fmpObs.open(strFile.c_str(), ios::out | ios::binary);
	
    long unsigned int tmpIdx=0;
    vector<MapPoint*> vMapPoints = World.GetAllMapPoints();
    for(std::vector<MapPoint*>::iterator vit=vMapPoints.begin(), vend=vMapPoints.end(); vit!=vend; vit++, tmpIdx++)
	{
		MapPoint* pMPi = *vit;
		if(!pMPi)
			cerr<<"MapPoint pointer = NULL. shouldn't"<<endl;
		if(!pMPi->isBad())	//only save those not bad
		{
			//2 2.1 save plain variable
			//2 ---------------------------
			// 0xeb,0x90,nNextId,mnId,mnFirstKFid,WorldPos,mDescriptor,mpRefKF
			{
            fmpVar.write(reinterpret_cast<char*>(saveHeader),2);
			fmpVar.write(reinterpret_cast<char*>(&pMPi->nNextId),sizeof(long unsigned int));
			fmpVar.write(reinterpret_cast<char*>(&pMPi->mnId),sizeof(long unsigned int));
			fmpVar.write(reinterpret_cast<char*>(&pMPi->mnFirstKFid),sizeof(long int));
			// protected
			cv::Mat twp = pMPi->GetWorldPos();
			for(int ti=0;ti<3;ti++)
				fmpVar.write(reinterpret_cast<char*>(&twp.at<float>(ti)),sizeof(float));
			cv::Mat tdes = pMPi->GetDescriptor();	//256b, 8*uint32_t. cv::Mat::zeros(1,32,CV_8U);
			for(int ti=0;ti<32;ti++)
				fmpVar.write(reinterpret_cast<char*>(&tdes.at<unsigned char>(ti)),sizeof(unsigned char));
			fmpVar.write(reinterpret_cast<char*>(&pMPi->GetReferenceKeyFrame()->mnId),sizeof(long unsigned int));
			}
			//2 ---------------------------

			//2 2.2 save observations
			//2 ---------------------------
			// 0xeb,0x90,Nobs,Nobs*(ob.kfmnId,ob.mpIdx)
			{
            fmpObs.write(reinterpret_cast<char*>(saveHeader),2);
			map<KeyFrame*,size_t> observations = pMPi->GetObservations();
			size_t nObs = observations.size();
			if(nObs==0)	cerr<<"save nObs=0, shouldn't"<<endl;
			fmpObs.write(reinterpret_cast<char*>(&nObs),sizeof(size_t));
			size_t saveObsCnt=0;
			for(std::map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++, saveObsCnt++)
			{
				KeyFrame* pKFm = mit->first;
				size_t idMPinKF = mit->second;
				fmpObs.write(reinterpret_cast<char*>(&pKFm->mnId), sizeof(long unsigned int));
				fmpObs.write(reinterpret_cast<char*>(&idMPinKF), sizeof(size_t));	//KF id and MP index in KF
			}
			if(saveObsCnt!=nObs)
				cerr<<"saveObsCnt!=nObs, shouldn't"<<endl;
			}
			//2 ---------------------------

			mpSaveCnt++;
		}
		
	}

	fmpVar.close();
	fmpObs.close();
	
	cout<<"total "<<mpSaveCnt<<" MapPoints saved."<<endl;
	}
	//1 -----------------------------------------------


 
	//1 3. save keyframe files
	//1 ------------------------------------------------
	{
	fstream fkfVar,fkfKeys,fkfKeysUn,fkfDes,fkfMPids,fkfLPEGs,fkfmGrid;
	cout << endl << "Saving KeyFrames" << endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.bin";
	fkfVar.open(strFile.c_str(), ios::out | ios::binary);
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.bin";
	fkfKeys.open(strFile.c_str(), ios::out | ios::binary);

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.bin";
	fkfKeysUn.open(strFile.c_str(), ios::out | ios::binary);
	
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.bin";
	fkfDes.open(strFile.c_str(), ios::out | ios::binary);

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.bin";
	fkfMPids.open(strFile.c_str(), ios::out | ios::binary);

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfLoopEdges.bin";
	fkfLPEGs.open(strFile.c_str(), ios::out | ios::binary);

	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfmGrid.bin";
	fkfmGrid.open(strFile.c_str(), ios::out | ios::binary);

    long unsigned int tmpIdx=0;
	vector<KeyFrame*> vKeyFrames = World.GetAllKeyFrames();
	for(std::vector<KeyFrame*>::iterator vitKFs=vKeyFrames.begin(), vendKFs=vKeyFrames.end(); vitKFs!=vendKFs; vitKFs++, tmpIdx++)
	{
		KeyFrame* pKFi = *vitKFs;
		if(!pKFi)
			cerr<<"KeyFrame pointer = NULL, shouldn't."<<endl;
		if(!pKFi->isBad())
		{
			//2 3.1 save plain variables
			//2 -------------------------------
			// 0xeb,0x90, nNextId, mnId, mnFrameId, mTimeStamp, Rcw, tcw
            fkfVar.write(reinterpret_cast<char*>(saveHeader),2);
			fkfVar.write(reinterpret_cast<char*>(&pKFi->nNextId), sizeof(long unsigned int));
			fkfVar.write(reinterpret_cast<char*>(&pKFi->mnId), sizeof(long unsigned int));
			fkfVar.write(reinterpret_cast<char*>(&pKFi->mnFrameId), sizeof(long unsigned int));
			fkfVar.write(reinterpret_cast<char*>(&pKFi->mTimeStamp), sizeof(double));
			// protected
			cv::Mat Rcwi=pKFi->GetRotation();
			for(int ti=0;ti<3;ti++)
				for(int tj=0;tj<3;tj++)
					fkfVar.write(reinterpret_cast<char*>(&Rcwi.at<float>(ti,tj)), sizeof(float));
			cv::Mat tcwi = pKFi->GetTranslation();
			for(int ti=0;ti<3;ti++)
				fkfVar.write(reinterpret_cast<char*>(&tcwi.at<float>(ti)), sizeof(float));
			//2 -------------------------------

			//2 3.2 save KeyPoints and KeyPointsUn
			//2 -------------------------------
			// 0xeb,0x90, nKeys, nKeys*(ptx,pty,size,angle,responsse,  octave,classid)
			{
			vector<cv::KeyPoint> mvKeysi = pKFi->GetKeyPoints();
			size_t nKeys=mvKeysi.size();
            fkfKeys.write(reinterpret_cast<char*>(saveHeader),2);
			fkfKeys.write(reinterpret_cast<char*>(&nKeys), sizeof(size_t));
			for(vector<cv::KeyPoint>::iterator vitkeys=mvKeysi.begin(), vendkeys=mvKeysi.end(); vitkeys!=vendkeys; vitkeys++)
			{
				cv::KeyPoint kpi = *vitkeys;	float tmpf; int tmpi;
				tmpf=kpi.pt.x;		fkfKeys.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.pt.y;		fkfKeys.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.size;		fkfKeys.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.angle;		fkfKeys.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.response;	fkfKeys.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpi=kpi.octave;	fkfKeys.write(reinterpret_cast<char*>(&tmpi), sizeof(int));
				tmpi=kpi.class_id;	fkfKeys.write(reinterpret_cast<char*>(&tmpi), sizeof(int));
			}
			}
			
			{
			vector<cv::KeyPoint> mvKeysiUn = pKFi->GetKeyPointsUn();
            size_t nKeysUn=mvKeysiUn.size();
            fkfKeysUn.write(reinterpret_cast<char*>(saveHeader),2);
			fkfKeysUn.write(reinterpret_cast<char*>(&nKeysUn), sizeof(size_t));
			for(vector<cv::KeyPoint>::iterator vitkeysun=mvKeysiUn.begin(), vendkeysun=mvKeysiUn.end(); vitkeysun!=vendkeysun; vitkeysun++)
			{
				cv::KeyPoint kpi = *vitkeysun;	float tmpf; int tmpi;
				tmpf=kpi.pt.x;		fkfKeysUn.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.pt.y;		fkfKeysUn.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.size;		fkfKeysUn.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.angle;		fkfKeysUn.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpf=kpi.response;	fkfKeysUn.write(reinterpret_cast<char*>(&tmpf), sizeof(float));
				tmpi=kpi.octave;	fkfKeysUn.write(reinterpret_cast<char*>(&tmpi), sizeof(int));
				tmpi=kpi.class_id;	fkfKeysUn.write(reinterpret_cast<char*>(&tmpi), sizeof(int));
			}
			}
			//2 -------------------------------

			//2 3.3 save descriptors
			//2 -------------------------------
			// 0xeb,0x90, nDes, nDes*(32*uint8)
			{
			cv::Mat descriptorsi = pKFi->GetDescriptors();
			int desnumi = descriptorsi.rows;
            fkfDes.write(reinterpret_cast<char*>(saveHeader),2);
			fkfDes.write(reinterpret_cast<char*>(&desnumi), sizeof(int));
			for(int ti=0;ti<desnumi;ti++)
			{
				cv::Mat tdes = descriptorsi.row(ti);
				for(int tj=0;tj<32;tj++)
					fkfDes.write(reinterpret_cast<char*>(&(tdes.at<unsigned char>(tj))),sizeof(unsigned char));
			}
			}
			//2 -------------------------------


			//2 3.4 save mappoint id
			//2 -------------------------------
			// 0xeb,0x90, KF.mnId, nMPs, nMPs*(MPmnId, MPidx)
			{
			vector<MapPoint*> vpsi = pKFi->GetMapPointMatches();
            size_t nMPcnt=0;
            //compute valid mappoint number
			for(size_t mvpMPidx=0, iend=vpsi.size();mvpMPidx<iend;mvpMPidx++)
			{
				MapPoint* vit = vpsi[mvpMPidx];
                if(vit)
					if(!vit->isBad()) 
            			nMPcnt++;
			}
			if(nMPcnt==0)	cerr<<"nMPcnt=0, shouldn't"<<endl;
            fkfMPids.write(reinterpret_cast<char*>(saveHeader),2);
			fkfMPids.write(reinterpret_cast<char*>(&pKFi->mnId), sizeof(long unsigned int));
			fkfMPids.write(reinterpret_cast<char*>(&nMPcnt), sizeof(size_t));	//mnId & number of mappoints
		
            size_t scnt=0;
            for(size_t mvpMPidx=0, iend=vpsi.size();mvpMPidx<iend;mvpMPidx++)
            {
                MapPoint* vit = vpsi[mvpMPidx];
				if(vit)
					if(!vit->isBad())
					{
						fkfMPids.write(reinterpret_cast<char*>(&vit->mnId), sizeof(long unsigned int));
						fkfMPids.write(reinterpret_cast<char*>(&mvpMPidx), sizeof(size_t));	//mnId & number of mappoints
		                scnt++;
					}
            }
			if(scnt==0)	cerr<<"scnt==0, no good mappoint? shouldn't"<<endl;
			if(scnt!=pKFi->GetMapPoints().size() || scnt!=nMPcnt)
				cerr<<"scnt!=pKFi->GetMapPoints().size(), shouldn't"<<endl;
			}
			//2 -------------------------------

			
			//2 3.5 save loopedges
			//2 -------------------------------
			// 0xeb,0x90, KF.mnId, nLPEGs, nLPEGs*(lpKFmnId)
			{
			set<KeyFrame*> lpedges = pKFi->GetLoopEdges();
			size_t nlpegs = lpedges.size();
            fkfLPEGs.write(reinterpret_cast<char*>(saveHeader),2);
			fkfLPEGs.write(reinterpret_cast<char*>(&pKFi->mnId), sizeof(long unsigned int));
			fkfLPEGs.write(reinterpret_cast<char*>(&nlpegs), sizeof(size_t));
			if(nlpegs>0)
			{
                for(set<KeyFrame*>::iterator sit=lpedges.begin(), send=lpedges.end(); sit!=send; sit++)
				{
                    KeyFrame * tpKFs=*sit;
                    fkfLPEGs.write(reinterpret_cast<char*>(&tpKFs->mnId), sizeof(long unsigned int));
				}
			}
			}
			//2 -------------------------------

			//2 3.6 save mGrid
			//2 -------------------------------
			// 0xeb,0x90, FRAME_GRID_COLS*FRAME_GRID_ROWS*(gridN, gridN*mGrid_ijk)
			{
			vector< vector <vector<size_t> > > mGrid = pKFi->GetmGrid();
			fkfmGrid.write(reinterpret_cast<char*>(saveHeader),2);
			for(int i=0; i<FRAME_GRID_COLS;i++)
		    {
		        for(int j=0; j<FRAME_GRID_ROWS; j++)
		        {
		        	vector<size_t> gridij = mGrid[i][j];
					size_t gridN=gridij.size();
					fkfmGrid.write(reinterpret_cast<char*>(&gridN),sizeof(size_t));
					for(size_t k=0; k<gridN; k++)
					{
						fkfmGrid.write(reinterpret_cast<char*>(&gridij[k]),sizeof(size_t));
					}
				}
		    }
			}
			//2 -------------------------------
			
			
			kfSaveCnt++;
			
		}
		
	}
	
	fkfVar.close();
	fkfKeys.close();
	fkfKeysUn.close();
	fkfDes.close();
	fkfMPids.close();
	fkfLPEGs.close();
	fkfmGrid.close();

	cout<<"total "<<kfSaveCnt<<" KeyFrames saved."<<endl;
	}
	//1 ------------------------------------------------

	
	//1 4. save global parameters
	//1 ------------------------------------------------
	{
	fstream f;
	cout<<endl<<"Saving global params"<<endl;
	strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.bin";
	f.open(strFile.c_str(), ios::out|ios::binary);
	
	//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
	//3 ------------------------------------------------
	long unsigned int tMPnNextId,tFRnNextId,tKFnNextId;
	tMPnNextId=MapPoint::nNextId;
	tFRnNextId=Frame::nNextId;
	tKFnNextId=KeyFrame::nNextId;
    f.write(reinterpret_cast<char*>(&tMPnNextId),	sizeof(long unsigned int));
    f.write(reinterpret_cast<char*>(&mpSaveCnt),	sizeof(long unsigned int));
    f.write(reinterpret_cast<char*>(&kfSaveCnt),	sizeof(long unsigned int));
    f.write(reinterpret_cast<char*>(&tFRnNextId),	sizeof(long unsigned int));
    f.write(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
	//KeyFrame data
    KeyFrame* pKF0=static_cast<KeyFrame*>(NULL);
	vector<KeyFrame*> vpKFt = World.GetAllKeyFrames();
    for(vector<KeyFrame*>::iterator tvit=vpKFt.begin(), tvend=vpKFt.end(); tvit!=tvend; tvit++)
    {
        pKF0 = *tvit;
        if(pKF0->mnId==0)
            break;
    }
	if(pKF0->mnId != 0)
		cerr<<"pKF0->mnId != 0, shouldn't. Note!!\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	if(pKF0->isBad())
		cerr<<"pKF0->isBad(), shouldn't"<<endl;
	//3 ------------------------------------------------
	
	//3 Line2.  KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy,
	//3 (cont.) mnMinX,mnMinY,mnMaxX,mnMaxY
	//3 ------------------------------------------------
    f.write(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
    f.write(reinterpret_cast<char*>(&pKF0->mfGridElementWidthInv),	sizeof(float));
    f.write(reinterpret_cast<char*>(&pKF0->mfGridElementHeightInv),	sizeof(float));
    f.write(reinterpret_cast<char*>(&pKF0->fx),	sizeof(float));
    f.write(reinterpret_cast<char*>(&pKF0->fy),	sizeof(float));
    f.write(reinterpret_cast<char*>(&pKF0->cx),	sizeof(float));
    f.write(reinterpret_cast<char*>(&pKF0->cy),	sizeof(float));
	vector<int> tv=pKF0->GetMinMaxXY();
	for(int ti=0;ti<4;ti++)
        f.write(reinterpret_cast<char*>(&tv[ti]),	sizeof(int));
	//3 ------------------------------------------------
	
	//3 Line3. mnScaleLevels0(N), N*mvScaleFactors0, N*mvLevelSigma20 for the first 2 KFs. 
	//3 ------------------------------------------------
	int mscalelevels=pKF0->GetScaleLevels();
	f.write(reinterpret_cast<char*>(&mscalelevels),sizeof(int));
	vector<float> sfactors =  pKF0->GetScaleFactors();
    for(int i=0;i<mscalelevels;i++)
		f.write(reinterpret_cast<char*>(&sfactors[i]),sizeof(float));
	vector<float> lsigma2 = pKF0->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
    for(int i=0;i<mscalelevels;i++)
		f.write(reinterpret_cast<char*>(&lsigma2[i]),sizeof(float));
    if((size_t)mscalelevels!=sfactors.size() || (size_t)mscalelevels!=lsigma2.size())
		cerr<<"mscalelevels!=sfactors.size() || mscalelevels!=lsigma2.size(), shouldn't"<<endl;
	//3 ------------------------------------------------
	
	//3 Line4. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
	//3 ------------------------------------------------
    KeyFrame* pKFg=static_cast<KeyFrame*>(NULL);
	for(vector<KeyFrame*>::iterator kfit=vpKFt.begin(), kfend=vpKFt.end(); kfit!=kfend; kfit++)
	{
		pKFg=*kfit;
        if(pKFg)
            if(pKFg->mnId>2 && !pKFg->isBad())
                break;
	}
    if(!pKFg) cerr<<"pKFg=NULL, shouldn't"<<endl;
	
	int mslevelsg=pKFg->GetScaleLevels();
	f.write(reinterpret_cast<char*>(&mslevelsg),sizeof(int));
    vector<float> sfactorsg =  pKFg->GetScaleFactors();
    for(int i=0;i<mslevelsg;i++)
		f.write(reinterpret_cast<char*>(&sfactorsg[i]),sizeof(float));
    vector<float> lsigma2g = pKFg->GetVectorScaleSigma2();	//mvInvLevelSigma2 is 1/mvLevelSigma2
    for(int i=0;i<mslevelsg;i++)
		f.write(reinterpret_cast<char*>(&lsigma2g[i]),sizeof(float));
    if((size_t)mslevelsg!=sfactorsg.size() || (size_t)mslevelsg!=lsigma2g.size())
		cerr<<"mslevelsg!=sfactorsg.size() || mslevelsg!=lsigma2g.size()"<<endl;
	//3 ------------------------------------------------

	
	f.close();
	}
	//1 ------------------------------------------------


}

//void SaveLocalMapThread(LocalMapping* pLM)
//{

	
//	//1 5. save local mappint params
//	//1 ------------------------------------------------
	
//	fstream flm;
//	cout<<endl<<"Saving local mapping params"<<endl;
//    string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"LocalMapParams.bin";
//	flm.open(strFile.c_str(), ios::out|ios::binary);

//	list<MapPoint*> lMPinLM = pLM->GetmlpRecentAddedMapPoints();
//	size_t rampsize = lMPinLM.size();
//	flm.write(reinterpret_cast<char*>(&rampsize),sizeof(size_t));
	
//	for(list<MapPoint*>::iterator lit=lMPinLM.begin(), lend=lMPinLM.end(); lit!=lend; lit++)
//	{
//		MapPoint* pMP=*lit;
//		flm.write(reinterpret_cast<char*>(&pMP->mnId),sizeof(long unsigned int));
//	}

//	flm.close();
	

//}

//bool LoadLocalMapThread(LocalMapping* pLM, MapMPIndexPointer &mpIdxPtMap)
//{
	
//	fstream flm;
//    cout<<endl<<"Loading local mapping params"<<endl;
//    string strFile = ros::package::getPath("ORB_SLAM")+"/tmp/"+"LocalMapParams.bin";
//	flm.open(strFile.c_str(), ios::in|ios::binary);
//    if(!flm.is_open())
//    {
//        cerr<<"LocalMapParams.bin open error"<<endl;
//        return false;
//    }

//	size_t rampsize;
//	flm.read(reinterpret_cast<char*>(&rampsize),sizeof(size_t));

//	list<MapPoint*> lMPinLM;// = pLM->GetmlpRecentAddedMapPoints();
//	for(size_t i=0;i<rampsize;i++)
//	{
//		long unsigned int mpId;
//		flm.read(reinterpret_cast<char*>(&mpId),sizeof(long unsigned int));
//        if(mpIdxPtMap.count(mpId)>0)
//        {

//            //cerr<<mpId<<" mpIdxPtMap.cout(mpId)==0, shouldn't"<<endl;
//            MapPoint* pMP=mpIdxPtMap[mpId];
//            lMPinLM.push_back(pMP);
//        }
//		if(flm.fail())	cerr<<"flm.fail(), shouldn't"<<endl;
//	}
	
//	pLM->SetmlpRecentAddedMapPoints(lMPinLM);

//	flm.close();
//    return true;
//}

static bool myOpenFile(fstream &ifs, string strFile)
{
	ifs.open(strFile.c_str(), ios::in | ios::binary);
	if(!ifs.is_open())
	{
		cout<<strFile<<" open failed."<<endl;
		return false;
	}
	if(ifs.eof())
	{
		cout<<strFile<<" empty."<<endl;
		ifs.close();
		return false;
	}
	return true;
}


// load plain variables of mappoints from file
bool loadMPVariables(KeyFrameDatabase *db, Map *wd, MapMPIndexPointer *mpIdxPtMap, 
		VecUL &_VecMPmnId, VecUL &_vRefKFIdInMP)
{
	fstream ifs,ifGlobal;
	if(	!myOpenFile(ifs, string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpVariables.bin"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.bin"))   )
		return false;

	//read global params
	long unsigned int tMPnNextId,mpSaveCnt,kfSaveCnt,tFRnNextId,tKFnNextId;
	cout<<endl<<"Reading MapPoint params"<<endl;
	{
	//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
	//3 ------------------------------------------------
    ifGlobal.read(reinterpret_cast<char*>(&tMPnNextId),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&mpSaveCnt),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&kfSaveCnt),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&tFRnNextId),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
	//3 ------------------------------------------------
	
	MapPoint::nNextId = tMPnNextId;
	Frame::nNextId = tFRnNextId;
	KeyFrame::nNextId = tKFnNextId;
	}

	//record the reference KF's mnId, in the order of lines saved in file
	VecUL vRefKFIdInMP(mpSaveCnt);
	VecUL VecMPmnId(mpSaveCnt);

	Frame tmpFrame;
    tmpFrame.mTcw = Mat::eye(4,4,CV_32F);
    KeyFrame * tmpKF = new KeyFrame(tmpFrame, wd, db);


	unsigned long int linecnt=0;
    for(unsigned long int smpcnt=0;smpcnt<mpSaveCnt;smpcnt++)
    {	
        unsigned char hd[2];	//header1=0xeb,header2=0x90.
		
		//2 2.1 read plain variable
		//2 ---------------------------
		// 0xeb,0x90,nNextId,mnId,mnFirstKFid,WorldPos,mDescriptor,mpRefKF
		// plain variables in MapPoint
		unsigned long int nNextId;
		long unsigned int mnId, mpRefKFId;
		long int mnFirstKFid;
		Mat mWorldPos = Mat::zeros(3, 1, CV_32F);
		Mat mDescriptor = Mat::zeros(1, 32, CV_8UC1);
		{
        ifs.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error mpVariables, shouldn't"<<endl;
		ifs.read(reinterpret_cast<char*>(&nNextId),	sizeof(long unsigned int));
		ifs.read(reinterpret_cast<char*>(&mnId),	sizeof(long unsigned int));
		ifs.read(reinterpret_cast<char*>(&mnFirstKFid),	sizeof(long int));
		// protected
		for(int ti=0;ti<3;ti++)
		{
			float tmpf;
			ifs.read(reinterpret_cast<char*>(&tmpf),sizeof(float));
			mWorldPos.at<float>(ti)=tmpf;
		}
		for(int ti=0;ti<32;ti++)
		{
			unsigned char tmpuc;
            ifs.read(reinterpret_cast<char*>(&tmpuc),sizeof(unsigned char));
			mDescriptor.at<unsigned char>(ti)=tmpuc;
		}
		ifs.read(reinterpret_cast<char*>(&mpRefKFId),sizeof(long unsigned int));
        if(ifs.fail()) cerr<<"ifsfail in mpVariables, shouldn't."<<endl;
		if(mnFirstKFid<0) cerr<<"mnFirstKFid<0, shouldn't."<<endl;
		}
		//2 ---------------------------


		// new MapPoint memory space and pointer
        MapPoint* tmpMP = new MapPoint(mWorldPos, tmpKF, wd);
        MapPoint::nNextId = nNextId;

		//public
		tmpMP->mnId = mnId;
		tmpMP->mnFirstKFid = mnFirstKFid;
		
		tmpMP->mTrackProjX = 0;//mTrackProjX;
		tmpMP->mTrackProjY = 0;//mTrackProjY;
		tmpMP->mbTrackInView = 0;//mbTrackInView;
		tmpMP->mnTrackScaleLevel = 0;//mnTrackScaleLevel;
		tmpMP->mTrackViewCos = 0;//mTrackViewCos;
		tmpMP->mnTrackReferenceForFrame = 0;//mnTrackReferenceForFrame;
		tmpMP->mnLastFrameSeen = 0;//mnLastFrameSeen;
		tmpMP->mnBALocalForKF = 0;//mnBALocalForKF;
		tmpMP->mnFuseCandidateForKF = 0;//mnFuseCandidateForKF;
		tmpMP->mnLoopPointForKF = 0;//mnLoopPointForKF;
		tmpMP->mnCorrectedByKF = 0;//mnCorrectedByKF;
		tmpMP->mnCorrectedReference = 0;//mnCorrectedReference;
		
		//protected
		tmpMP->SetWorldPos(mWorldPos);
		tmpMP->SetDescriptor(mDescriptor);
//		tmpMP->SetNormalVec(mNormalVector);		//set by UpdateNormalAndDepth(). need mObservations/mpRefKF/mWorldPos and KF.Tcw
//		tmpMP->SetmnVisible(1);//(mnVisible);
//      tmpMP->SetmnFound(1);//(mnFound);
//		tmpMP->SetMinDistance(mfMaxDistance);
//		tmpMP->SetMaxDistance(mfMaxDistance);

		//record reference KF id of this mappoint
		vRefKFIdInMP[linecnt] = mpRefKFId;

		//record mnId
		VecMPmnId[linecnt] = mnId;

		// add to the mapping from index to pointer
		if(mpIdxPtMap->count(mnId)>0)
            cerr<<mnId<<" mappoint count "<< mpIdxPtMap->count(mnId)<<"exist? shouldn't!!"<<endl;
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

    //delete tmpKF;
	
	return true;
}


bool loadKFVariables(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc,
	MapKFIndexPointer *kfIdxPtMap, VecUL &_VecKFmnId)
{
	fstream ifkfVar,ifkfKeys,ifkfKeysUn,ifkfDes,ifkfmGrid,ifGlobal;
	if(	!myOpenFile(ifkfVar,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.bin")) 	||
		!myOpenFile(ifkfKeys,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.bin")) 	||
		!myOpenFile(ifkfKeysUn,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.bin")) 	||
		!myOpenFile(ifkfDes,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.bin"))	||
		!myOpenFile(ifkfmGrid,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfmGrid.bin"))		||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.bin"))   )
	{	
		return false;
	}

	//read global params
	cout<<endl<<"Reading KeyFrame params"<<endl;
	
	long unsigned int tMPnNextId,mpSaveCnt,kfSaveCnt,tFRnNextId,tKFnNextId;
	float mfGridElementWidthInv, mfGridElementHeightInv, fx, fy, cx, cy;
	int mnMinX,mnMinY,mnMaxX,mnMaxY;
	int mnScaleLevels0, mnScaleLevelsOther;
    std::vector<float> mvScaleFactors0,mvScaleFactorsOther;
    std::vector<float> mvLevelSigma20,mvLevelSigma2Other;
    std::vector<float> mvInvLevelSigma20,mvInvLevelSigma2Other;
	{
	//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
	//3 ------------------------------------------------
	ifGlobal.read(reinterpret_cast<char*>(&tMPnNextId),	sizeof(long unsigned int));
	ifGlobal.read(reinterpret_cast<char*>(&mpSaveCnt),	sizeof(long unsigned int));
	ifGlobal.read(reinterpret_cast<char*>(&kfSaveCnt),	sizeof(long unsigned int));
	ifGlobal.read(reinterpret_cast<char*>(&tFRnNextId),	sizeof(long unsigned int));
	ifGlobal.read(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
	//3 ------------------------------------------------
	
	MapPoint::nNextId = tMPnNextId;
	Frame::nNextId = tFRnNextId;
	KeyFrame::nNextId = tKFnNextId;
	}
	
	//record KeyFrame mnId
    VecUL VecKFmnId(kfSaveCnt);

	{
	//3 Line2.  KF.nNextID, mfGridElementWidthInv, mfGridElementHeightInv,fx/fy/cx/cy,
	//3 (cont.) mnMinX,mnMinY,mnMaxX,mnMaxY
	//3 ------------------------------------------------
	ifGlobal.read(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
	ifGlobal.read(reinterpret_cast<char*>(&mfGridElementWidthInv),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&mfGridElementHeightInv),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&fx),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&fy),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&cx),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&cy),	sizeof(float));
	ifGlobal.read(reinterpret_cast<char*>(&mnMinX),	sizeof(int));
	ifGlobal.read(reinterpret_cast<char*>(&mnMinY),	sizeof(int));
	ifGlobal.read(reinterpret_cast<char*>(&mnMaxX),	sizeof(int));
	ifGlobal.read(reinterpret_cast<char*>(&mnMaxY),	sizeof(int));
	//3 ------------------------------------------------
	}

	{
	//3 Line3. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for the first 2 KFs. 
    //3 ------------------------------------------------
	ifGlobal.read(reinterpret_cast<char*>(&mnScaleLevels0),	sizeof(int));
	mvScaleFactors0.resize(mnScaleLevels0);
	mvLevelSigma20.resize(mnScaleLevels0);
	mvInvLevelSigma20.resize(mnScaleLevels0);
    for(int i=0;i<mnScaleLevels0;i++)
		ifGlobal.read(reinterpret_cast<char*>(&mvScaleFactors0[i]),	sizeof(float));
    for(int i=0;i<mnScaleLevels0;i++)
	{
		ifGlobal.read(reinterpret_cast<char*>(&mvLevelSigma20[i]),	sizeof(float));
		mvInvLevelSigma20[i] = 1/mvLevelSigma20[i];
	}
	//3 ------------------------------------------------
	}

	{
	//3 Line4. mnScaleLevels(N), N*mvScaleFactors, N*mvLevelSigma2 for other KFs. 
    //3 ------------------------------------------------
	ifGlobal.read(reinterpret_cast<char*>(&mnScaleLevelsOther),	sizeof(int));
	mvScaleFactorsOther.resize(mnScaleLevelsOther);
	mvLevelSigma2Other.resize(mnScaleLevelsOther);
	mvInvLevelSigma2Other.resize(mnScaleLevelsOther);
    for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ifGlobal.read(reinterpret_cast<char*>(&mvScaleFactorsOther[i]),	sizeof(float));
		cout<<mvScaleFactorsOther[i]<<" ";
	}
    for(int i=0;i<mnScaleLevelsOther;i++)
	{
		ifGlobal.read(reinterpret_cast<char*>(&mvLevelSigma2Other[i]),	sizeof(float));
		mvInvLevelSigma2Other[i] = 1/mvLevelSigma2Other[i];
		cout<<mvLevelSigma2Other[i]<<" ";
	}
	//3 ------------------------------------------------
	}


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
    tmpFrame.mTcw = Mat::eye(4,4,CV_32F);

	// KeyFrame 0&1 is different. ORBextractor settings are fixed as 2000/1.2/8
	Frame tmpFrame0(tmpFrame);
	tmpFrame0.mnScaleLevels = mnScaleLevels0;
	tmpFrame0.mvScaleFactors = mvScaleFactors0;
	tmpFrame0.mvLevelSigma2 = mvLevelSigma20;
	tmpFrame0.mvInvLevelSigma2 = mvInvLevelSigma20;

	// read each row of the files
	unsigned long int linecnt=0;
    for(unsigned long int skfcnt=0;skfcnt<kfSaveCnt;skfcnt++)
	{	
        unsigned char hd[2];	//header1=0xeb,header2=0x90.
		//new keyframe from tmp frame
		KeyFrame* tmpKF;
		long unsigned int nNextId,mnId,mnFrameId;

		//1 kfVariables.txt
		//2 3.1 save plain variables - read
		{
		//2 -------------------------------
		// 0xeb,0x90, nNextId, mnId, mnFrameId, mTimeStamp, Rcw, tcw
		//public
		double mTimeStamp;
		//protected
		cv::Mat Rcwi = Mat::eye(3, 3, CV_32F);
		cv::Mat tcwi = Mat::zeros(3, 1, CV_32F);
		
        ifkfVar.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error kfVariables, shouldn't"<<endl;

		ifkfVar.read(reinterpret_cast<char*>(&nNextId), sizeof(long unsigned int));
		ifkfVar.read(reinterpret_cast<char*>(&mnId), sizeof(long unsigned int));
		ifkfVar.read(reinterpret_cast<char*>(&mnFrameId), sizeof(long unsigned int));
		ifkfVar.read(reinterpret_cast<char*>(&mTimeStamp), sizeof(double));
		// protected
		for(int ti=0;ti<3;ti++)
			for(int tj=0;tj<3;tj++)
				ifkfVar.read(reinterpret_cast<char*>(&Rcwi.at<float>(ti,tj)), sizeof(float));
		for(int ti=0;ti<3;ti++)
			ifkfVar.read(reinterpret_cast<char*>(&tcwi.at<float>(ti)), sizeof(float));
		//2 -------------------------------
        if(mnId<=1) //mnId>=0 &&  always >=0
			tmpKF = new KeyFrame(tmpFrame0, wd, db);	
		else
			tmpKF = new KeyFrame(tmpFrame, wd, db);
		
        KeyFrame::nNextId = nNextId;
		//evaluate 
		tmpKF->mnId = mnId;
		tmpKF->mnFrameId = mnFrameId;
		tmpKF->mTimeStamp = mTimeStamp;
		tmpKF->SetPose(Rcwi,tcwi);
        tmpKF->mnTrackReferenceForFrame = 0;//mnTrackReferenceForFrame;
		tmpKF->mnFuseTargetForKF = 0;//mnFuseTargetForKF;
		tmpKF->mnBALocalForKF = 0;//mnBALocalForKF;
		tmpKF->mnBAFixedForKF = 0;//mnBAFixedForKF;
		tmpKF->mnLoopQuery = 0;//mnLoopQuery;
		tmpKF->mnLoopWords = 0;//mnLoopWords;
		tmpKF->mLoopScore = 0;//mLoopScore;
		tmpKF->mnRelocQuery = 0;//mnRelocQuery;
		tmpKF->mnRelocWords = 0;//mnRelocWords;
		tmpKF->mRelocScore = 0;//mRelocScore;	
		//2 -------------------------------
		}


		
		//2 kfKeyPoints.txt
		//2 3.2 save KeyPoints and KeyPointsUn - read
		{
		//2 -------------------------------
		// 0xeb,0x90, nKeys, nKeys*(ptx,pty,size,angle,responsse,  octave,classid)
		{
		vector<cv::KeyPoint> tmvKeys;
		size_t kpn;
        ifkfKeys.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error kfKeyPoints, shouldn't"<<endl;
		
		ifkfKeys.read(reinterpret_cast<char*>(&kpn), sizeof(size_t));
		tmvKeys.resize(kpn);
		//for(size_t vit=0;vit<kpn;vit++)
		for(vector<cv::KeyPoint>::iterator vitkeys=tmvKeys.begin(), vendkeys=tmvKeys.end(); vitkeys!=vendkeys; vitkeys++)
		{				
			float ptx,pty,size,angle,response;
			int octave,classid;
			ifkfKeys.read(reinterpret_cast<char*>(&ptx), sizeof(float));
			ifkfKeys.read(reinterpret_cast<char*>(&pty), sizeof(float));
			ifkfKeys.read(reinterpret_cast<char*>(&size), sizeof(float));
			ifkfKeys.read(reinterpret_cast<char*>(&angle), sizeof(float));
			ifkfKeys.read(reinterpret_cast<char*>(&response), sizeof(float));
			ifkfKeys.read(reinterpret_cast<char*>(&octave), sizeof(int));
			ifkfKeys.read(reinterpret_cast<char*>(&classid), sizeof(int));
			//tmvKeys[vit] = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			*vitkeys = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			if(ifkfKeys.fail())  cerr<<"loopcnt: "<<skfcnt<<" mnId "<<mnId<<" ifkfKeys fail. shouldn't"<<endl;
		}
		tmpKF->SetKeyPoints(tmvKeys);
		}
		//2 -------------------------------
//		std::vector<cv::KeyPoint> mvKeys;

		//2 kfKeyPointsUn.txt
		{
		vector<cv::KeyPoint> tmvKeysUn;
		size_t kpun;
        ifkfKeysUn.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error kfKeyPointsUn, shouldn't"<<endl;

        ifkfKeysUn.read(reinterpret_cast<char*>(&kpun), sizeof(size_t));
		tmvKeysUn.resize(kpun);
		//for(size_t vit=0;vit<kpun;vit++)
		for(vector<cv::KeyPoint>::iterator vitkeysun=tmvKeysUn.begin(), vendkeysun=tmvKeysUn.end(); vitkeysun!=vendkeysun; vitkeysun++)
		{				
			float ptx,pty,size,angle,response;
			int octave,classid;
			ifkfKeysUn.read(reinterpret_cast<char*>(&ptx), sizeof(float));
			ifkfKeysUn.read(reinterpret_cast<char*>(&pty), sizeof(float));
			ifkfKeysUn.read(reinterpret_cast<char*>(&size), sizeof(float));
			ifkfKeysUn.read(reinterpret_cast<char*>(&angle), sizeof(float));
			ifkfKeysUn.read(reinterpret_cast<char*>(&response), sizeof(float));
			ifkfKeysUn.read(reinterpret_cast<char*>(&octave), sizeof(int));
			ifkfKeysUn.read(reinterpret_cast<char*>(&classid), sizeof(int));
			//tmvKeysUn[vit] = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			*vitkeysun = cv::KeyPoint(ptx,pty,size,angle,response,octave,classid);
			if(ifkfKeysUn.fail())  cerr<<"loopcnt: "<<skfcnt<<" mnId "<<mnId<<" ifkfKeysUn fail. shouldn't"<<endl;
		}
		tmpKF->SetKeyPointsUn(tmvKeysUn);
//		std::vector<cv::KeyPoint> mvKeysUn;
		}
		//2 -------------------------------
		}

		
		//2 3.3 save descriptors - read
		//3 kfDescriptors.txt
		//2 -------------------------------
		// 0xeb,0x90, nDes, nDes*(32*uint8)
        {
        int desnumi;
        ifkfDes.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
            cerr<<"header error kfDescriptors, shouldn't"<<endl;
		
		ifkfDes.read(reinterpret_cast<char*>(&desnumi), sizeof(int));
        cv::Mat matDes = cv::Mat::zeros(desnumi,32,CV_8U);

		for(int ti=0;ti<desnumi;ti++)
		{
			cv::Mat tdes = cv::Mat::zeros(1,32,CV_8U);;
			for(int tj=0;tj<32;tj++)
				ifkfDes.read(reinterpret_cast<char*>(&(tdes.at<unsigned char>(tj))),sizeof(unsigned char));
            tdes.copyTo(matDes.row(ti));
            if(ifkfDes.fail())  cerr<<"loopcnt: "<<skfcnt<<" mnId "<<mnId<<" ifkfDes fail. shouldn't"<<endl;
		}
		tmpKF->SetDescriptors(matDes);
		tmpKF->ComputeBoW();	//get mBowVec and mFeatVec
//		cv::Mat mDescriptors;
//		DBoW2::BowVector mBowVec;
//		DBoW2::FeatureVector mFeatVec;
		}
		//2 -------------------------------

		//2 3.6 save mGrid
		//2 -------------------------------
		// 0xeb,0x90, FRAME_GRID_COLS*FRAME_GRID_ROWS*(gridN, gridN*mGrid_ijk)
		{
		vector< vector <vector<size_t> > > mGrid;
        ifkfmGrid.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
            cerr<<"header error kfmGrid, shouldn't"<<endl;
		mGrid.resize(FRAME_GRID_COLS);
		for(int i=0; i<FRAME_GRID_COLS;i++)
		{
			mGrid[i].resize(FRAME_GRID_ROWS);
			for(int j=0; j<FRAME_GRID_ROWS; j++)
			{
				size_t gridN;
				ifkfmGrid.read(reinterpret_cast<char*>(&gridN),sizeof(size_t));
				vector<size_t> gridij(gridN);
				for(size_t k=0; k<gridN; k++)
				{
					ifkfmGrid.read(reinterpret_cast<char*>(&gridij[k]),sizeof(size_t));
				}
				mGrid[i][j] = gridij;
			}
		}
		tmpKF->SetmGrid(mGrid);
		if(ifkfmGrid.fail())	cerr<<"ifkfmGrid.fail(), shouldn't"<<endl;
		//std::vector< std::vector <std::vector<size_t> > > mGrid
		}
		//2 -------------------------------


		//record the mnId order
		VecKFmnId[linecnt] = mnId;
		
		// add to the mapping from index to pointer
        if(kfIdxPtMap->count(mnId)>0)
            cerr<<mnId<<" KF count "<<kfIdxPtMap->count(mnId)<<" exist? shouldn't!!"<<endl;
		(*kfIdxPtMap)[mnId] = tmpKF;

		linecnt++;
	}

	//evaluate mnId vector
	_VecKFmnId = VecKFmnId;

	cout<<"total "<<linecnt<<" KeyFrames loaded."<<endl;
    if(_VecKFmnId.size()!=kfSaveCnt)
        cout<<"_VecKFmnId.size()!=kfSaveCnt"<<endl;
	if(linecnt!=kfSaveCnt)
		cerr<<"linecnt != kfSaveCnt, shouldn't"<<endl;

	//close file
	ifkfVar.close();
	ifkfKeys.close();
	ifkfKeysUn.close();
	ifkfDes.close();
	ifGlobal.close();
	ifkfmGrid.close();

	return true;
}



bool loadMPKFPointers(MapMPIndexPointer &mpIdxPtMap, MapKFIndexPointer &kfIdxPtMap,
		const VecUL& VecKFmnId, const VecUL& VecMPmnId, const VecUL &vRefKFIdInMP)
{
	fstream ifkfMPids,ifkfLPEGs,ifGlobal,ifmpObs;
	if(	!myOpenFile(ifkfMPids,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.bin"))	||
		!myOpenFile(ifkfLPEGs,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfLoopEdges.bin"))   	||
		!myOpenFile(ifmpObs,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"mpObservations.bin"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.bin"))   	)
	{	
		return false;
	}
	
	//read global params
	long unsigned int tMPnNextId,mpSaveCnt,kfSaveCnt,tFRnNextId,tKFnNextId;
	cout<<endl<<"Setting MapPoint and KeyFrame pointers"<<endl;
	{
	//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
    ifGlobal.read(reinterpret_cast<char*>(&tMPnNextId),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&mpSaveCnt),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&kfSaveCnt),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&tFRnNextId),	sizeof(long unsigned int));
    ifGlobal.read(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
	MapPoint::nNextId = tMPnNextId;
	Frame::nNextId = tFRnNextId;
	KeyFrame::nNextId = tKFnNextId;
	}
	//check
    if(VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || mpIdxPtMap.size()!=mpSaveCnt)
		cerr<<"VecMPmnId.size()!=mpSaveCnt || vRefKFIdInMP.size()!=mpSaveCnt || MapMPIndexPointer.size()!=mpSaveCnt, shouldn't"<<endl;

	//------------------------------
	//1 MapPoint pointers
	// MapPoint mnId , line order in file
	for(size_t i=0;i<mpSaveCnt;i++)
	{
		long unsigned int tMPmnid = VecMPmnId[i];	
		MapPoint* pMP = mpIdxPtMap[tMPmnid];	//MapPoint pointer of mnId
        if(kfIdxPtMap.count(vRefKFIdInMP[i])==0)
            cerr<<vRefKFIdInMP[i]<<" no KF id of vRefKFIdInMP[i], shouldn't"<<endl;
        KeyFrame* pkFref = kfIdxPtMap[vRefKFIdInMP[i]];    //Keyframe pointer to ReferenceKF
        pMP->SetRefKFPointer(pkFref);	//set mpRefKF
		//	tmpMP.mpRefKF;
		
        unsigned char hd[2];

		//2 2.2 save observations - read
		//2 ---------------------------
		// observation data
		// 0xeb,0x90,Nobs,Nobs*(ob.kfmnId,ob.mpIdx)
        ifmpObs.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error mpObservations, shouldn't"<<endl;
		size_t nObs;
		ifmpObs.read(reinterpret_cast<char*>(&nObs),sizeof(size_t));
		if(nObs==0)	cerr<<"read nObs==0, shouldn't"<<endl;
		for(size_t j=0;j<nObs;j++)
		{
			long unsigned int kfIdj; size_t obIdj;	//mnId of KF see this MP, and id of observation
			ifmpObs.read(reinterpret_cast<char*>(&kfIdj), sizeof(long unsigned int));
			ifmpObs.read(reinterpret_cast<char*>(&obIdj), sizeof(size_t));	//KF id and MP index in KF
            if(kfIdxPtMap.count(kfIdj)==0)
            {
                cerr<<kfIdj<<" no KF id of this observation, shouldn't"<<endl;
                cerr<<endl;
            }
			else	//ignore wrong ones
			{
				KeyFrame* pKF = kfIdxPtMap[kfIdj];	//KF
	            pMP->AddObservation(pKF,obIdj);		//add observation
			}
			if(ifmpObs.fail())	cerr<<"ifmpObs.fail(), shouldn't"<<endl;
		}
		//2 ---------------------------
	}


	//------------------------------
	//2 KeyFrame pointers
	for(long unsigned int linecnt=0;linecnt<kfSaveCnt;linecnt++)
	{
        unsigned char hd[2];
		//mnId corresponding to this line in file
		long unsigned int kfmnId=VecKFmnId[linecnt];
		//corresponding KeyFrame
		KeyFrame* pKF = kfIdxPtMap[kfmnId];
		
		//after all mappoints loaded
		//4. kfMapPointsID, for pKF->mvpMapPoints
		//2 3.4 save mappoint id - read
		//2 -------------------------------
		// 0xeb,0x90, KF.mnId, nMPs, nMPs*(MPmnId, MPidx)
		{
		long unsigned int kfmnIdread;
		size_t nMPid;
        ifkfMPids.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error kfMapPointsID, shouldn't"<<endl;
		ifkfMPids.read(reinterpret_cast<char*>(&kfmnIdread), sizeof(long unsigned int));
		ifkfMPids.read(reinterpret_cast<char*>(&nMPid), sizeof(size_t));	//mnId & number of mappoints
		if(kfmnId!=kfmnIdread)	cerr<<"mpid: kfmnId!=VecKFmnId[linecnt], shouldn't"<<endl;
		if(nMPid==0) 	cerr<<"line " << linecnt<<" nMPid=0. shouldn't"<<endl;

		//KeyPoint number, size of mvpMapPoints
        size_t kpN = pKF->GetKeyPoints().size();
		if(kpN==0)	cerr<<"kpN==0 when read mappoint id, shouldn't"<<endl;
		vector<MapPoint*> mvpMPs = vector<MapPoint*>(kpN,static_cast<MapPoint*>(NULL));
		pKF->SetmvpMapPoints(mvpMPs);	//init mvpMapPoint as NULL (size = KeyPoints.size())

		//for each KeyFrame, set 
		for(size_t i=0;i<nMPid;i++)
		{
			long unsigned int tmpid;
			size_t tvpMPidx;
			ifkfMPids.read(reinterpret_cast<char*>(&tmpid), sizeof(long unsigned int));
			ifkfMPids.read(reinterpret_cast<char*>(&tvpMPidx), sizeof(size_t)); //mnId & number of mappoints
			//pointer to the mappoint
			MapPoint* pMP = mpIdxPtMap[tmpid];
			pKF->AddMapPoint(pMP,tvpMPidx);
			//check
            if((int)tvpMPidx!=pMP->GetIndexInKeyFrame(pKF))
				cerr<<tvpMPidx<<" "<<pMP->GetIndexInKeyFrame(pKF)<<"\ntvpMPidx!=pMP->GetIndexInKeyFrame(pKF), shouldn't"<<endl;
            if(ifkfMPids.fail()) cerr<<"kfmnId:"<<kfmnId<<" linecnt: "<<linecnt<<"ssMPids fail. shouldn't"<<endl;
		}
		//pKF->mvpMapPoints
		//		std::vector<MapPoint*> mvpMapPoints;
		//2 -------------------------------
		}
		

        //after all keyframes loaded
        //5. kfLoopEdges, load kf id
    	//2 3.5 save loopedges
		//2 -------------------------------
		// 0xeb,0x90, KF.mnId, nLPEGs, nLPEGs*(lpKFmnId)
		{
        long unsigned int kfmnIdread;
        size_t nLPEGid;
        ifkfLPEGs.read(reinterpret_cast<char*>(hd),2);
		if(hd[0]!=0xeb || hd[1]!=0x90)
			cerr<<"header error kfLoopEdges, shouldn't"<<endl;
		ifkfLPEGs.read(reinterpret_cast<char*>(&kfmnIdread), sizeof(long unsigned int));
		ifkfLPEGs.read(reinterpret_cast<char*>(&nLPEGid), sizeof(size_t));
        if(kfmnId!=kfmnIdread) cerr<<"lpeg: kfmnId!=VecKFmnId[linecnt], shouldn't"<<endl;
		//set each loop edge kf pointer
		if(nLPEGid>0)
	        for(size_t i=0;i<nLPEGid;i++)
	        {
	            long unsigned int tkfmnId;
				ifkfLPEGs.read(reinterpret_cast<char*>(&tkfmnId), sizeof(long unsigned int));
	            pKF->AddLoopEdge(kfIdxPtMap[tkfmnId]);
	        }
		if(ifkfLPEGs.fail())	cerr<<"ifkfLPEGs.fail(), shouldn't"<<endl;
        //pKF->mspLoopEdges
        //		std::set<KeyFrame*> mspLoopEdges;
		//2 -------------------------------
        }

	}

	///////////////////////////////////////////
	//after all keyframes and mappoints loaded
	///////////////////////////////////////////
	
	//build spanning tree. in increasing order of KeyFrame mnId
	{
	unsigned long int preidx=0;
	for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
	{
		KeyFrame* pKFm=mit->second;
		pKFm->UpdateConnections();
		if(preidx>pKFm->mnId)
			cerr<<"KeyFrame pre Id > cur Id, shouldn't."<<endl;
		preidx=pKFm->mnId;
	}
	//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
	//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
	//		std::vector<int> mvOrderedWeights;
	//		KeyFrame* mpParent;
	//		std::set<KeyFrame*> mspChildrens;
	}

	//UpdateNormalAndDepth, need the mObservations/mpRefKF/mWorldPos
	{
	unsigned long int preidxmp=0;
	for(MapMPIndexPointer::iterator mit=mpIdxPtMap.begin(), mend=mpIdxPtMap.end(); mit!=mend; mit++)
	{
		MapPoint* pMPm=mit->second;
		pMPm->UpdateNormalAndDepth();
		if(preidxmp>pMPm->mnId)
			cerr<<"MapPoint pre Id > cur Id, shouldn't."<<endl;
		preidxmp=pMPm->mnId;
	}
	//		float mfMinDistance;
    //		float mfMaxDistance;
	//		cv::Mat mNormalVector;
	}
	
	ifkfMPids.close();
	ifkfLPEGs.close();
	ifmpObs.close();
	ifGlobal.close();

    return true;
}

bool LoadWroldFromFile(KeyFrameDatabase *db, Map *wd, ORBVocabulary* mpvoc, KeyFrame *pLastKF)//, MapMPIndexPointer &_mpIdxPtMap)
{
	MapMPIndexPointer mpIdxPtMap;
	MapKFIndexPointer kfIdxPtMap;
	VecUL vRefKFIdInMP;
	VecUL vKFmnId,vMPmnId;
    bool ret1,ret2,ret3,ret4;

    long unsigned int maxKFid=0;

	//1 step 1. load and create all mappoints
    cout<<"loading step 1.."<<endl;
	ret1=loadMPVariables(db,wd,&mpIdxPtMap,vMPmnId,vRefKFIdInMP);

	//1 step 2. load and craete all keyframes
    cout<<"loading step 2.."<<endl;
	ret2=loadKFVariables(db,wd,mpvoc,&kfIdxPtMap,vKFmnId);

	//1 step 3. associate pointers in MPs and KFs
    cout<<"loading step 3.."<<endl;
    ret3=loadMPKFPointers(mpIdxPtMap, kfIdxPtMap, vKFmnId, vMPmnId, vRefKFIdInMP );
	if(mpIdxPtMap.size()!=vMPmnId.size())
		cerr<<"mpIdxPtMap.size()!=vMPmnId.size()"<<endl;
	if(kfIdxPtMap.size()!=vKFmnId.size())
		cerr<<"kfIdxPtMap.size()!=vKFmnId.size()"<<endl;

	//1 step 4. associate pointers in invertfile of vocabulary
    cout<<"loading step 4.."<<endl;
	for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
	{
		KeyFrame* pKF = mit->second;
		db->add(pKF);
        if(maxKFid<pKF->mnId)
            maxKFid = pKF->mnId;
	}

    //1 step5. evaluate nNextId for Frame/MapPoint/KeyFrame
    ifstream ifGlobal;
    ret4 = myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.bin"));
    if(ret4)
    {
		long unsigned int tMPnNextId,mpSaveCnt,kfSaveCnt,tFRnNextId,tKFnNextId;
		//3 Line1. MP.nNextID, mpSaveCnt, kfSaveCnt, Frame::nNextId, KeyFrame::nNextId
        ifGlobal.read(reinterpret_cast<char*>(&tMPnNextId),	sizeof(long unsigned int));
        ifGlobal.read(reinterpret_cast<char*>(&mpSaveCnt),	sizeof(long unsigned int));
        ifGlobal.read(reinterpret_cast<char*>(&kfSaveCnt),	sizeof(long unsigned int));
        ifGlobal.read(reinterpret_cast<char*>(&tFRnNextId),	sizeof(long unsigned int));
        ifGlobal.read(reinterpret_cast<char*>(&tKFnNextId),	sizeof(long unsigned int));
		MapPoint::nNextId = tMPnNextId;
		Frame::nNextId = tFRnNextId;
		KeyFrame::nNextId = tKFnNextId;
    	ifGlobal.close();
    }

    //1 step 6. world
	if(ret1&&ret2&&ret3&&ret4)
	{
        cout<<"loading step 6.."<<endl;
		for(MapKFIndexPointer::iterator mit=kfIdxPtMap.begin(), mend=kfIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddKeyFrame(mit->second);
		}
		for(MapMPIndexPointer::iterator mit=mpIdxPtMap.begin(), mend=mpIdxPtMap.end(); mit!=mend; mit++)
		{
			wd->AddMapPoint(mit->second);
		}
	}	

    pLastKF = kfIdxPtMap[maxKFid];
    //_mpIdxPtMap = mpIdxPtMap;
	
	return (ret1&&ret2&&ret3&&ret4);
}


#endif

