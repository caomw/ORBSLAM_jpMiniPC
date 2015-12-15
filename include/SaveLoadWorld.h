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
	ifstream ifkfVar,ifkfKeys,ifkfKeysUn,ifkfDes,ifkfMPids,ifGlobal,ifkfLPEGs;
	if(	!myOpenFile(ifkfVar,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfVariables.txt")) 	||
		!myOpenFile(ifkfKeys,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPoints.txt")) 	||
		!myOpenFile(ifkfKeysUn,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfKeyPointsUn.txt")) 	||
		!myOpenFile(ifkfDes,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfDescriptors.txt"))	||
		!myOpenFile(ifkfMPids,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfMapPointsID.txt"))	||
		!myOpenFile(ifGlobal,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"GlobalParams.txt"))   ||
		!myOpenFile(ifkfLPEGs,	string(ros::package::getPath("ORB_SLAM")+"/tmp/"+"kfLoopEdges.txt"))    )
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
	tmpFrame.mpORBvocabulary = db->mpVoc;
	tmpFrame.mnScaleLevels = mnScaleLevelsOther;
	tmpFrame.mvScaleFactors = mvScaleFactorsOther;
	tmpFrame.mvLevelSigma2 = mvLevelSigma2Other;
	tmpFrame.mvInvLevelSigma2 = mvInvLevelSigma2Other;
	
	Frame tmpFrame0(tmpFrame);
	tmpFrame0.mnScaleLevels = mnScaleLevels0;
	tmpFrame0.mvScaleFactors = mvScaleFactors0;
	tmpFrame0.mvLevelSigma2 = mvLevelSigma20;
	tmpFrame0.mvInvLevelSigma2 = mvInvLevelSigma20;
	
	unsigned long int linecnt=0;
	while(!ifkfVar.eof())
	{	
		string slVar,slKeys,slKeysUn,slDes,slMPids,slLPEGs;
		stringstream ssVar,ssKeys,ssKeysUn,ssDes,ssMPids,ssLPEGs;

		//1. kfVariables.txt
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


		//new keyframe from tmp frame
		if(mnId>=0 && mnId<=1)
			KeyFrame* tmpKF = new KeyFrame(tmpFrame0, wd, db);	
		else
			KeyFrame* tmpKF = new KeyFrame(tmpFrame, wd, db);

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

		//2.1 kfKeyPoints.txt
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
		}
		tmpKF->SetKeyPoints(tmvKeys);
//		std::vector<cv::KeyPoint> mvKeys;
		}

		//2.2 kfKeyPointsUn.txt
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
		}
		tmpKF->SetKeyPointsUn(tmvKeysUn);
//		std::vector<cv::KeyPoint> mvKeysUn;
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

		//3. kfDescriptors.txt
		getline(ifkfDes,slDes);
		ssDes << slDes;
//		cv::Mat mDescriptors;
//		DBoW2::BowVector mBowVec;
//		DBoW2::FeatureVector mFeatVec;

		//after all keyframes loaded
		getline(ifkfMPids,slMPids);
		ssMPids << slMPids;
//		std::vector<MapPoint*> mvpMapPoints;

		//after all keyframes loaded
		getline(ifkfLPEGs,slLPEGs);
		ssLPEGs << slLPEGs;
//		std::set<KeyFrame*> mspLoopEdges;

		//after all keyframes and mappoints loaded
//		std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
//		std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
//		std::vector<int> mvOrderedWeights;
//		KeyFrame* mpParent;
//		std::set<KeyFrame*> mspChildrens;

		//ignored. neither saved or loaded
//		cv::Mat im;

	}


	cout<<"total "<<linecnt<<" KeyFrames loaded."<<endl;


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


void SaveWorldToFile(const Map& World)
{
	string strFile;
	
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
	f<<MapPoint::nNextId<<endl;
	
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
		}
	}

	fmpVar.close();
	fmpObs.close();
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
			set<MapPoint*> mpsi = pKFi->GetMapPoints(); 	//not GetMapPointMatches()!
			fkfMPids << mpsi.size() <<" ";	//number of mappoints
			for(set<MapPoint*>::iterator sit=mpsi.begin(), send=mpsi.end(); sit!=send; sit++)
			{
				fkfMPids << sit->mnId <<" ";
			}
			fkfMPids <<endl;

			// 5. save loopedges
			set<KeyFrame*> lpedges = pKFi->GetLoopEdges();
			size_t nlpegs = lpedges.size();
			fkfLPEGs << nlpegs<<" ";
			if(nlpegs>0)
			{
				for(set<KeyFrame*> sit=lpedges.begin(), send=lpedges.end(); sit!=send; sit++)
				{
					fkfLPEGs << sit->mnId <<" ";
				}
			}
			fkfLPEGs <<endl;
			
			// xxx. for test
			if(printflag && tmpIdx==4)
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
			
		}
		
	}
	
	fkfVar.close();
	fkfKeys.close();
	fkfKeysUn.close();
	fkfDes.close();
	fkfMPids.close();
	fkfLPEGs.close();
	
	}
		

}

#endif

