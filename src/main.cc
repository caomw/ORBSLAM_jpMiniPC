/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<ros/package.h>
#include<boost/thread.hpp>

#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "Converter.h"


using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
        ros::shutdown();
        return 1;
    }

    // Load Settings and Check
    string strSettingsFile = ros::package::getPath("ORB_SLAM")+"/"+argv[2];

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
   /* Old version to load vocabulary using cv::FileStorage
    string strVocFile = ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
    if(!fsVoc.isOpened())
    {
        cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        ros::shutdown();
        return 1;
    }
    ORB_SLAM::ORBVocabulary Vocabulary;
    Vocabulary.load(fsVoc);
    */
    
    // New version to load vocabulary from text file "Data/ORBvoc.txt". 
    // If you have an own .yml vocabulary, use the function
    // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
    string strVocFile = ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    
    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        ros::shutdown();
        return 1;
    }

    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

	//Added by wangjing
	//get euler angle from IMU via Serial Port and set to tracker
	Tracker.SetIMUEulerAngle(30.0/180.0*3.1415926,20.0/180.0*3.1415926,50.0/180.0*3.1415926);


    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    ros::Rate r(fps);

    while (ros::ok())
    {
        FramePub.Refresh();
        MapPub.Refresh();
        Tracker.CheckResetByPublishers();
        r.sleep();
    }

    // Save keyframe poses at the end of the execution
    ofstream f;

    vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

    cout << endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << endl;
    string strFile = ros::package::getPath("ORB_SLAM")+"/"+"KeyFrameTrajectory.txt";
    f.open(strFile.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
    f.close();

	//Added by wangjing
	using namespace ORB_SLAM;

    //for test
    cout<<"max KF id: "<<World.GetMaxKFid()<<endl;
    cout<<"min KF id: "<<vpKFs[0]->mnId<<endl;
    cout<<"KF number: "<<World.KeyFramesInMap()<<endl;
    if(vpKFs[0]->isBad())
        cout<<"first KF is bad"<<endl;
    else
        cout<<"first KF is good"<<endl;
    for (int i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        std::vector<float> tmpScaleFactors = pKFi->GetScaleFactors();
        std::vector<float> tmpScaleLevelSigma2 = pKFi->GetVectorScaleSigma2();
        std::cout<<pKFi->mnId<<"\tscale factor: ";
        for(int j=0;j<tmpScaleFactors.size();j++)
        {
            std::cout<<tmpScaleFactors[j]<<" ";
        }
        std::cout<<endl;


        std::cout<<pKFi->mnId<<"\tlevel sigma2: ";
        for(int j=0;j<tmpScaleLevelSigma2.size();j++)
        {
            std::cout<<tmpScaleLevelSigma2[j]<<" ";
        }
        std::cout<<endl;

        std::cout<<pKFi->mnId<<"\tnNextId: "<<pKFi->nNextId<<endl;
        std::cout<<pKFi->mnId<<"\tmfGridElementWidthInv: "<<pKFi->mfGridElementWidthInv<<endl;
        std::cout<<pKFi->mnId<<"\tmfGridElementHeightInv: "<<pKFi->mfGridElementHeightInv<<endl;
        std::cout<<pKFi->mnId<<"\tfx/fy/cx/cy: "<<pKFi->fx<<" "<<pKFi->fy<<" "<<pKFi->cx<<" "<<pKFi->cy<<endl;
    }

	{
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
			f << tmpIdx << " " << listsize << " ";	//save wordID,  and number of KFs see this word
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
//    bool printflag=true;
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
			fmpVar << setprecision(7);
			cv::Mat twp = pMPi->GetWorldPos();
			fmpVar << twp.at<float>(0) <<" "<< twp.at<float>(1) <<" "<< twp.at<float>(2) <<" ";
			cv::Mat tnv = pMPi->GetNormal();
			fmpVar << tnv.at<float>(0) <<" "<< tnv.at<float>(1) <<" "<< tnv.at<float>(2) <<" ";
			cv::Mat tdes = pMPi->GetDescriptor();	//256b, 8*uint32_t
			const uint32_t *tpdes = tdes.ptr<uint32_t>();
            for(int ti=0; ti<8; ti++)
			{
				fmpVar << tpdes[ti] <<" ";
			}
//            if(printflag)
//            {
//                const unsigned char *tp = tdes.ptr();
//                for(int ti=0;ti<32;ti++)
//                    cout<<(int)tp[ti]<<" ";
//                cout<<endl;
//                cout<<"cout worldpos: "<<twp<<endl;
//                cout<<"normal vector: "<<tnv<<endl;
//                cout<<"cout descriptor: "<<tdes<<endl;
//                printflag = false;
//            }
			fmpVar << pMPi->mnVisible <<" ";
			fmpVar << pMPi->mnFound <<" ";
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
				fmpObs << pKFm->mnId << " " << idMPinKF << " ";	//KF id and MP index in KF
			}

			fmpObs << std::endl;
		}
	}

	fmpVar.close();
	fmpObs.close();
	}


	{
	ofstream fkfVar,fkfKeys,fkfKeysUn,fkfDes,fkfMPids;
	cout << endl << "Saving MapPoint" << endl;
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
	
	fkfVar << setprecision(7);
	fkfKeys << setprecision(7);
	fkfKeysUn << setprecision(7);
	fkfDes << setprecision(7);
	fkfMPids<< setprecision(7);

	int tmpIdx=0;
    vector<KeyFrame*> vKeyFrames = World.GetAllKeyFrames();
    bool printflag=true;
    for(std::vector<KeyFrame*>::iterator vit=vKeyFrames.begin(), vend=vKeyFrames.end(); vit!=vend; vit++, tmpIdx++)
	{
		KeyFrame* pKFi = *vit;
		if(!pKFi->isBad())
		{
			//1. save plain variables
			// public
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
			fkfVar << setprecision(7);
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
			for(vector<cv::KeyPoint>::iterator vitkeysun=fkfKeysUn.begin(), vendkeysun=mvKeysi.end(); vitkeysun!=vendkeysun; vitkeysun++)
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
			set<MapPoint*> mpsi = pKFi->GetMapPoints();		//not GetMapPointMatches()!
			fkfMPids << mpsi.size() <<" ";	//number of mappoints
			for(set<MapPoint*>::iterator sit=mpsi.begin(), send=mpsi.end(); sit!=send; sit++)
			{
				MapPoint* tmp = *sit;
				fkfMPids << tmp.mnId <<" ";
			}
			fkfMPids <<endl;
			
			if(printflag)
			{
				for(int ti=0;ti<3;ti++)
				{
					for(int tj=0;tj<3;tj++)					
						cout << Rcwi.at<float>(ti,tj) <<" ";
					cout<<endl;
				}
				for(int ti=0;ti<3;ti++)
					cout<<tcwi.at<float><<" ";
				cout<<endl;
				cout<<"Pose Tcw: "<<pKFi->GetPose();

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

	}
	

    ros::shutdown();

	return 0;
}
