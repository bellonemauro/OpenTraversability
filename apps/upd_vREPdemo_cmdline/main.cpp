// eric@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

// modified by Eric Rohmer and Mauro Bellone


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
//# define M_PI 3.141592653589793238462643383279502884L /* pi */

extern "C" {
#include "v_repConst.h"
#include "extApiPlatform.h"
#include "extApi.h"   
	/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

//include UPD library
#include "upd.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::MatrixXf;


//constructurs and allocation stuff 
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud;// (new pcl::PointCloud<pcl::PointXYZRGBA>);   // define a my cloud
pcl::PointCloud<pcl::PointSurfel>::Ptr output_cloud;// (new pcl::PointCloud<pcl::PointSurfel>);
 int user_data;


 
 void
 viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
 {
     viewer.setBackgroundColor (1.0, 1.0, 1.0);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb1(input_cloud);
     viewer.addPointCloud<pcl::PointXYZRGBA> (input_cloud, rgb1, "cloud");
     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
     cout << "i only run once" << endl;
 }

 void
 viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
 {
     static unsigned count = 0;
     stringstream ss;
     ss << "Once per viewer loop: " << count++;
     viewer.removeShape ("text", 0);
     viewer.addText (ss.str(), 200, 300, "text", 0);

     //FIXME: possible race condition here:
     user_data++;
 }


int main(int argc,char* argv[])
{
	int portNb=19999;

    // parsing arguments

	bool _activate_viewer = true;
	bool _verbose = false;
	double unevenness_treshold = 0.99;
	double max_climbing_angle = 13;
	double search_radius = 0.3;   //this is something we can discuss and maybe change from V-Rep


	if (pcl::console::find_argument (argc, argv, "-verbose") >= 0)   // -nv --> no viewer
   {
	   _verbose = true;

   }
	if (pcl::console::find_argument (argc, argv, "-nv") >= 0)   // -nv --> no viewer
   {
	   _activate_viewer = false;

   }
	if (pcl::console::parse (argc, argv, "-ut", unevenness_treshold) >= 0)   // -ut set the unevenness treshold
	{
		cout << "set unevenness_treshold to " << unevenness_treshold << " " << endl; 
	}
	else{
		cout << "no unevenness_treshold set, using default value " << unevenness_treshold << " " << endl; 
	}

	if (pcl::console::parse (argc, argv, "-ca", max_climbing_angle) >= 0)   // -ca set the climbing angle
	{
		cout << "set max_climbing_angle to " << max_climbing_angle << " deg" << endl; 
	}
	else {
		cout << "sno et max_climbing_angle set, using default value" << max_climbing_angle << " deg" << endl; 
	}
	if (pcl::console::parse (argc, argv, "-sr", search_radius) >= 0)   // -sr set the search radius
	{
		cout << "set search_radius to " << search_radius << " m" << endl; 
	}
	else {
		cout << "no search_radius set, using default value " << search_radius << " m" << endl; 
	}


	if (pcl::console::find_argument (argc, argv, "-h") >= 0)
   {
     std::cout<<"\n Help ... please refer to this help for application usage\n\n"<<std::endl;
     std::cout<<"-nv --> no viewer \n"<<std::endl;
     std::cout<<"-ca set the climbing angle in [deg]  \n"<<std::endl;
     std::cout<<"-sr set the search radius in [m] \n"<<std::endl;
     std::cout<<"-ut set the unevenness treshold \n"<<std::endl;
    return 0;
   }



//initialize the cloud and the viewer
input_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);   // define a my cloud
output_cloud.reset( new pcl::PointCloud<pcl::PointSurfel>);
pcl::visualization::CloudViewer viewer("Cloud Viewer");

if (_activate_viewer == true)
{
	viewer.runOnVisualizationThreadOnce (viewerOneOff);
	viewer.runOnVisualizationThread (viewerPsycho);
}

	int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if (clientID!=-1)
	{

		simxInt err =-1;
		simxInt err2=-1;
		simxInt err3=-1;
		simxInt err4=-1;
		simxInt err5=-1;
		simxInt errStreamTarget;
		simxInt kinHandle,  camHandle, chairHandle, laserFrameHandle, kinectFrameHandle;
		simxInt res[2];
		simxFloat *img;
		simxUChar *imgRGB;
		simxInt resRGB[2];
		simxFloat kinPos[3],kinOri[3]; 
		simxUChar* data;
		simxInt dataSize;
		simxFloat xc,yc,thc,xl,yl,isTarget; //data streamed in "fromVREP" signal (xc,yc,thc) current chair's position and orientation in inertial frame, (xl,yl) coordinates of the laser dot on the ground in inertial frame, not in use: isTarget =1 is the spot is selected else =0 
		simxFloat xt,yt,tht; //data streamed in "target" signal, (xt,yt,tht) latest coordinates of the dot on ground that has been defined as a target for the chair and target's orientation in radians
		simxFloat posk[3],orik[3]; //ground truth of the kinect position and orientation in intertial frame
		simxFloat H_laserFrame_in_wheelchairFrame[3][4]; //tranformation matrix (warning:3x4 and not 4x4) of the laserFrame expressed in the wheelchairFrame
		simxFloat H_kinectFrame_in_wheelchairFrame[3][4]; //tranformation matrix (warning:3x4 and not 4x4) of the kinectFrame expressed in the wheelchairFrame


		simxFloat laserFramePosition[3];
        simxGetObjectPosition(clientID,laserFrameHandle,-1,laserFramePosition,simx_opmode_oneshot_wait);
        std::cout<<"####################################### laser height "<<laserFramePosition[2]<<std::endl;

        simxFloat kinectFramePosition[3];
        simxGetObjectPosition(clientID,kinectFrameHandle,-1,kinectFramePosition,simx_opmode_oneshot_wait);
        std::cout<<"####################################### kinect height "<<kinectFramePosition[2]<<std::endl;

		simxFloat (*pointcloud)[3];

		bool b_targetStreamInititated=false;
		simxInt dataTargetSize;

		printf("Connected to remote API server\n");

		//make sure we get the handle of the cam, the depth sensor and the wheelchair frame
		while (err !=simx_return_ok &&  err2 !=simx_return_ok &&  err3 !=simx_return_ok && err4 !=simx_return_ok && err5 !=simx_return_ok){
			err=simxGetObjectHandle(clientID,"kinect_visionSensorChair",&kinHandle,simx_opmode_oneshot_wait);
			err2=simxGetObjectHandle(clientID,"kinect_camSensorChair",&camHandle,simx_opmode_oneshot_wait);
			err3=simxGetObjectHandle(clientID,"wheelchairFrame",&chairHandle,simx_opmode_oneshot_wait);
			err4=simxGetObjectHandle(clientID,"laserFrame",&laserFrameHandle,simx_opmode_oneshot_wait);			
			err5=simxGetObjectHandle(clientID,"kinectFrame",&kinectFrameHandle,simx_opmode_oneshot_wait);
		}

		//Here get the resolution of the kinect and start its streaming
		err=simxGetVisionSensorDepthBuffer(clientID,kinHandle,res,&img,simx_opmode_streaming);
		while  (err !=simx_return_ok){
			err=simxGetVisionSensorDepthBuffer(clientID,kinHandle,res,&img,simx_opmode_buffer);
			extApi_sleepMs(100);
		}

		simxInt camXResolution=res[0];
		simxInt camYResolution=res[1];
		simxFloat camXAngleInDegrees=57.;
		simxFloat camXHalfAngle=camXAngleInDegrees*0.5* M_PI /180;
		simxFloat camYHalfAngle=(camXAngleInDegrees*0.5*M_PI/180)*camYResolution/camXResolution;
		simxFloat nearClippingPlane=0.2;
		simxFloat depthAmplitude=3.3;
		simxFloat camXResolutionHalf=camXResolution/2;
		simxFloat camYResolutionHalf=camYResolution/2;	
		//printf("--> err %i res %i %i camXHalfAngle %f camYHalfAngle %f \n",err,res[0],res[1],camXHalfAngle,camYHalfAngle);
		//for (int i=0;i<res[0]*res[1];i++) printf("%f \n",img[i]);

		//start the streaming of the RGB image of the laser (optional, just for displaying purposes)
		err=simxGetVisionSensorImage(clientID,camHandle,resRGB,&imgRGB,0,simx_opmode_streaming);


		//get the transformation matrix of the kinect in the wheelchair frame
		simxGetStringSignal(clientID,"TransfLaserFrameInWheelchairFrame",&data,&dataSize,simx_opmode_oneshot_wait);
		////simxClearStringSignal(clientID,"TransfLaserFrameInWheelchairFrame",simx_opmode_oneshot);
		//printf("TransfLaserFrameInWheelchairFrame\n");
		for(int j=0;j<3;j++){
			for(int i=0;i<4;i++){
				H_laserFrame_in_wheelchairFrame[j][i]=((simxFloat*)data)[j*4+i];
				//printf("%1.2f\t",H_laserFrame_in_wheelchairFrame[j][i]);
				//if ((j*4+i)==3 ) printf("\n");
				//if ((j*4+i)==7 ) printf("\n");
				//if ((j*4+i)==11 ) printf("\n0\t0\t0\t1\n");
			}
		}
		
		//get the transformation matrix of the kinect in the wheelchair frame
		simxGetStringSignal(clientID,"TransfkinectFrameInWheelchairFrame",&data,&dataSize,simx_opmode_oneshot_wait);
		////simxClearStringSignal(clientID,"TransfkinectFrameInWheelchairFrame",simx_opmode_oneshot);
		//printf("TransfKinectFrameInWheelchairFrame\n");
		for(int j=0;j<3;j++){
			for(int i=0;i<4;i++){
				H_kinectFrame_in_wheelchairFrame[j][i]=((simxFloat*)data)[j*4+i];
				//printf("%1.2f\t",H_kinectFrame_in_wheelchairFrame[j][i]);
				//if ((j*4+i)==3 ) printf("\n");
				//if ((j*4+i)==7 ) printf("\n");
				//if ((j*4+i)==11 ) printf("\n0\t0\t0\t1\n");
			}
		}


		//kinPos is the position of the kinect sensor in the inertial frame
		//kinOri is the orientation of the kinect sensor in the inertial frame
		//i.e. euler angles as defined in V-REP, by default the chair frame and the kinect frame are oriented in the same way
		// see http://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm

		err=simxGetObjectPosition(clientID,kinectFrameHandle,-1,kinPos,simx_opmode_streaming);
		err=simxGetObjectOrientation(clientID,kinectFrameHandle,-1,kinOri,simx_opmode_streaming);

		//init streaming of current position and xl,yl
		//  Here we get the initial pose of the chair in world frame xc,yc,thc
		//  as well as xl and yl the laser beam position in world frame, and isTarget (NOT IN USE) defines if the point targeted by xl,yl is a target (1) or not (0)
		//  through the signal "fromVREP" sent from laserFrame script

		err=simxReadStringStream(clientID,"fromVREP",&data,&dataSize,simx_opmode_streaming);
		while (err !=simx_return_ok){
			err=simxReadStringStream(clientID,"fromVREP",&data,&dataSize,simx_opmode_buffer);
			extApi_sleepMs(100);
		}
		// (xc,yc,thc) current position of the chair in world frame
		// (xl,yl) current laser beam's position on the ground in world frame
		// isTarget=1 if (xl,yl) is a chair target position if not isTarget=0 (not in use)
		xc=((simxFloat*)data)[0];
		yc=((simxFloat*)data)[1];
		thc=((simxFloat*)data)[2];
		xl=((simxFloat*)data)[3];
		yl=((simxFloat*)data)[4];
		isTarget=((simxFloat*)data)[5];
		//printf("xc= %f yc=%f thc=%f xl=%f yl=%f isTarget=%i\n",xc,yc,thc*180./M_PI,xl,yl,(int)isTarget);


		//// USE HERE THE (xc, yc, thc) TO GET THE INITIAL MAP SCAN
		//
		//
		// 
		//          HERE is your code?

		//constructurs and allocation stuff 
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);   // define a my cloud
		pcl::PointCloud<pcl::PointSurfel>::Ptr output_cloud (new pcl::PointCloud<pcl::PointSurfel>);

		upd *my_upd= new upd ();      // define an object from my library which is the UPD
		//my_upd->setFlip(true);        //set a flipping function to true, just a configuration 
		//my_upd->setViewPoint(Eigen::Vector3d(0, 0, 0));

		my_upd->setSearchRadius(search_radius);  // send the search radius to the UPD object
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		pass.setInputCloud (input_cloud);


		//
		//
		//
		////

		//Init the streaming of "target" signal that will inform what is the last target that has been defined (xt,yt,tht)
		// by the user in world frame, send by laserFrame script through "target" signal

		errStreamTarget=simxGetStringSignal(clientID,"target",&data,&dataSize,simx_opmode_streaming);

		// Initialize the point cloud 2D array
		pointcloud= new simxFloat[camXResolution*camYResolution][3];


		printf("Getting in the main loop\n");
		while (simxGetConnectionId(clientID)!=-1)
		{

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			// I receive xc,yc,thc current position of the chair and xl
			// yl are the coordinates of the laser beam on the ground in the workframe
			// isTarget is not used right now

			if (simxReadStringStream(clientID,"fromVREP",&data,&dataSize,simx_opmode_buffer)==simx_return_ok){
				xc=((simxFloat*)data)[0];
				yc=((simxFloat*)data)[1];
				thc=((simxFloat*)data)[2];
				xl=((simxFloat*)data)[3];
				yl=((simxFloat*)data)[4];
				isTarget=((simxFloat*)data)[5];

				//printf("xc= %f yc=%f thc=%f xl=%f yl=%f isTarget=%i\n",xc,yc,thc*180./M_PI,xl,yl,(int)isTarget);
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			// if required, the position and orientation of the kinect frame in inertial world is available after these 7 lines: 
			// orik[i] is in radian
			if(simxGetObjectPosition(clientID,kinectFrameHandle,-1,kinPos,simx_opmode_buffer)==simx_return_ok){
				//printf("kinPos [%f , %f , %f]\n",((simxFloat*)kinPos)[0],((simxFloat*)kinPos)[1],((simxFloat*)kinPos)[2]); 
				for (int i=1;i>3;i++) posk[i]=((simxFloat*)kinPos)[i];
			}
			if(simxGetObjectOrientation(clientID,kinectFrameHandle,-1,kinOri,simx_opmode_buffer)==simx_return_ok){
				//printf("kinOri [%f , %f , %f]\n",((simxFloat*)kinOri)[0]*180/M_PI,((simxFloat*)kinOri)[1]*180/M_PI,((simxFloat*)kinOri)[2]*180/M_PI); 
				for (int i=1;i>3;i++) orik[i]=((simxFloat*)kinOri)[i];
			}


			////////////////////////////////////////////////////////////////////////////////////////////////////////
			//xt and yt which are the latest coordinates that have been clicked and tht the orientation of the target, so it is our target in the inertial frame        
			if (simxGetStringSignal(clientID,"target",&data,&dataSize,simx_opmode_buffer)==simx_return_ok && dataSize>0){
				//simxClearStringSignal(clientID,"target",simx_opmode_oneshot); 
				xt= ((simxFloat*)data)[0];
				yt= ((simxFloat*)data)[1];
				tht=((simxFloat*)data)[2];
				printf("xt= %1.2f yt=%1.2f tht=%1.2f dataSize=%i\n",xt,yt,tht*180/M_PI,dataSize);
			}
			 

			// here i read the image streamed from the kinect cam (optional)
			err=simxGetVisionSensorImage(clientID,camHandle,resRGB,&imgRGB,0,simx_opmode_buffer);    			    

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Getting an update' if any, of the depthmap from kinect

			//simxFloat *img;
			if (simxGetVisionSensorDepthBuffer(clientID,kinHandle,res,&img,simx_opmode_buffer)==simx_error_noerror){
      
				// drawing point cloud to V-REP: variable init
				simxFloat* drawdata;
				drawdata= new simxFloat	[3*camXResolution*camYResolution];
				int cpt=0;

				input_cloud->clear();

				//transform img (i.e. the depthmap) into a point cloud
				simxFloat xAngle,yAngle, depthValue, xCoord, yCoord, zCoord; 
				for (int i=0;i<camYResolution;i++){
					yAngle=((camYResolutionHalf-i-0.5)/camYResolutionHalf)*camYHalfAngle;

					for (int j=0;j<camXResolution;j++){
						xAngle=((j-camXResolutionHalf+0.5)/camXResolutionHalf)*camXHalfAngle;
						depthValue=img[j+i*camXResolution];
						zCoord=nearClippingPlane+depthAmplitude*depthValue;
						xCoord=tan(xAngle)*zCoord;
						yCoord=tan(yAngle)*zCoord; 

						pointcloud[0][j*camXResolution+i]=xCoord;
						pointcloud[1][j*camXResolution+i]=yCoord;
						pointcloud[2][j*camXResolution+i]=zCoord;

						pcl::PointXYZRGBA _Point;
					   _Point.x = xCoord;
					   _Point.y = -yCoord;
					   _Point.z = -zCoord;

						input_cloud->push_back(_Point);


						drawdata[cpt]=xCoord;
						cpt++;
						drawdata[cpt]=zCoord;	
						cpt++;
						drawdata[cpt]=yCoord;		
						cpt++;		

					}
				}
				
				pass.setFilterFieldName ("y");
				pass.setFilterLimits (-0.6, 0.1);
				pass.filter (*input_cloud);

				pass.setFilterFieldName ("z");
				pass.setFilterLimits (-3.2, 0.0);
				pass.filter (*input_cloud);
				

				my_upd->setInputCloud(input_cloud);	    // set the cloud as input


				std::clock_t start;    //just define a timer
				double duration;
				start = std::clock();

				my_upd->runUPD_radius();				// run the upd
				
				duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				cout << "UPD processed in " << duration << endl;

				output_cloud = my_upd->getUPD( );          // get ther result



				my_upd->getAsColorMap(   input_cloud, unevenness_treshold, max_climbing_angle);


				/*int cpt=0;
				for (int i=0;i<input_cloud->size();i++){
					
						drawdata[cpt]=input_cloud->points[i].x;
						cpt++;
						drawdata[cpt]=input_cloud->points[i].z;	
						cpt++;
						drawdata[cpt]=input_cloud->points[i].y;		
						cpt++;		
				}*/

				simxFloat accessible=0.;

                // SIMPLE TEST FOR COLOR : TO REMOVE


				simxFloat H_laserFrame_in_kinectFrame[3][4];

				//get the transformation matrix of the kinect in the wheelchair frame
				simxGetStringSignal(clientID,"TransfLaserFrameInKinectFrame",&data,&dataSize,simx_opmode_oneshot_wait);
				////simxClearStringSignal(clientID,"TransfkinectFrameInWheelchairFrame",simx_opmode_oneshot);
				if ( _verbose == true) printf("H_laserFrame_in_kinectFrame\n");

				for(int j=0;j<3;j++){
					for(int i=0;i<4;i++){
						H_laserFrame_in_kinectFrame[j][i]=((simxFloat*)data)[j*4+i];
						if (_verbose == true) printf("%1.2f\t",H_laserFrame_in_kinectFrame[j][i]);
						if ((j*4+i)==3 && _verbose == true) printf("\n");
						if ((j*4+i)==7 && _verbose == true) printf("\n");
						if ((j*4+i)==11 && _verbose == true) printf("\n0\t0\t0\t1\n");
					}
				}


				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// I receive xc,yc,thc current position of the chair and xl
				// yl are the coordinates of the laser beam on the ground in the worldframe
				// isTarget is not used right now
				// xl_kinect and yl_kinect are the coordinates of the laser beam on the ground in kinect's frame

				simxFloat xl_kinect, yl_kinect;

				if (simxReadStringStream(clientID,"fromVREP",&data,&dataSize,simx_opmode_buffer)==simx_return_ok){
					xc=((simxFloat*)data)[0];
					yc=((simxFloat*)data)[1];
					thc=((simxFloat*)data)[2];
					xl=((simxFloat*)data)[3];
					yl=((simxFloat*)data)[4];
					isTarget=((simxFloat*)data)[5];
					xl_kinect=((simxFloat*)data)[6];
					yl_kinect=((simxFloat*)data)[7];

				   // printf("xc= %1.3f yc=%1.3f thc=%1.3f xl=%1.3f yl=%1.3f isTarget=%i xl_kinect=%1.3f yl_kinect=%1.3f\n",xc,yc,thc*180./M_PI,xl,yl,(int)isTarget,xl_kinect,yl_kinect);

				}



				   int m_k_neighbors=1;   //i need only the closest point
				   vector<int> pointIdxNKNSearch(m_k_neighbors);
				   vector<float> pointNKNSquaredDistance(m_k_neighbors);
				   pcl::PointXYZRGBA searchPoint;
				   float _angle = thc - M_PI/2;
				   searchPoint.x = -xl_kinect;
				   searchPoint.y = 0;
				   searchPoint.z = yl_kinect;

   				   if ( _verbose == true)
					{
					cout << "laser absolute coordinates xl = " << xl << " yl = "<< yl << endl;
				   cout << "chair coordinates xc = " << xc << " yc = "<< yc << " theta c " << thc << endl;
				   cout << "Laser point xl = " << searchPoint.x << " yl = "<< searchPoint.z << endl;
				   }

				   pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
				   kdtree.setInputCloud (input_cloud);
				   if ( kdtree.nearestKSearch (searchPoint, m_k_neighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) cout << "Index found : " << pointIdxNKNSearch[0] << " pointNKNSquaredDistance " << pointNKNSquaredDistance[0] << "\n" << endl; //m_k_neighbors=1; //TODO fix here

						pcl::PointXYZRGBA _Point;
						_Point.x = input_cloud->points[pointIdxNKNSearch[0]].x;
						_Point.y = input_cloud->points[pointIdxNKNSearch[0]].y;
						_Point.z = input_cloud->points[pointIdxNKNSearch[0]].z;
						_Point.rgba =  input_cloud->points[pointIdxNKNSearch[0]].rgba;

						input_cloud->points[pointIdxNKNSearch[0]].rgba = 0x0000FF;

						if ( _Point.g > 250 && pointNKNSquaredDistance[0] < 0.2 )
						{
						accessible=1;
						if ( _verbose == true)
						{
						cout << "analysed accessible point x = " << _Point.x << " y = " << _Point.y << " z = " << _Point.z << " rgba = " << std::hex << _Point.rgba << " . " << endl;
						cout << "ACCESSIBLE POINT" << endl;
						}
						}
						else {
							if ( _verbose == true)
							{
							cout << "analysed NOT accessible point x = " << _Point.x << " y = " << _Point.y << " z = " << _Point.z << " rgba = " << std::hex << _Point.rgba << " . " << endl;
							cout << "NOT ACCESSIBLE POINT" << endl;
							}
						}

				if (_activate_viewer == true)
				{
				viewer.showCloud(input_cloud, "cloud");
				}
				//if (sqrt((xl-xc)*(xl-xc)+(yl-yc)*(yl-yc))< 3) accessible=1.;
                //END SIMPLE TEST

                // accessible=0 or accessible =1, depending on your traversability map.

                simxFloat lsignal[3]={xl,yl,accessible};
                simxUChar* packedString1=(simxUChar*) lsignal; 
                int packedStringSize1=3*sizeof(float);
                   simxSetStringSignal(clientID,"laser",packedString1,packedStringSize1,simx_opmode_oneshot);


				//signal to send for drawing of the point cloud in V-REP
				simxUChar* packedString=(simxUChar*)drawdata; 
				int packedStringSize=3*camXResolution*camYResolution*sizeof(float);
				simxSetStringSignal(clientID,"draw3D",packedString,packedStringSize,simx_opmode_oneshot);
				delete[] drawdata,packedString;
				//end point cloud to draw in V-REP


				////////////////////////////////////////////////////////////////////////////////////////////////////////

			}


			extApi_sleepMs(5);
		}
		printf("finished!\n");
		//delete[] pointcloud[3];
		//delete[] img,imgRGB,data;
		simxFinish(clientID);
	} 
	else 
	{
		cout << " no connection found, closing ...." << endl;
	}


	return(0);
}

