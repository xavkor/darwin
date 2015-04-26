/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include <FaceTracker/Tracker.h>
#include <opencv2/opencv.hpp>

#define INI_FILE_PATH       "config.ini"

#define U2D_DEV_NAME        "/dev/ttyUSB0"

void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &con,cv::Mat &tri,cv::Mat &visi)
{
  int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;

  //draw triangulation
  c = CV_RGB(0,0,0);
  for(i = 0; i < tri.rows; i++){
    if(visi.at<int>(tri.at<int>(i,0),0) == 0 ||
       visi.at<int>(tri.at<int>(i,1),0) == 0 ||
       visi.at<int>(tri.at<int>(i,2),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
  }
  //draw connections
  c = CV_RGB(0,0,255);
  for(i = 0; i < con.cols; i++){
    if(visi.at<int>(con.at<int>(0,i),0) == 0 ||
       visi.at<int>(con.at<int>(1,i),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(con.at<int>(0,i),0),
		   shape.at<double>(con.at<int>(0,i)+n,0));
    p2 = cv::Point(shape.at<double>(con.at<int>(1,i),0),
		   shape.at<double>(con.at<int>(1,i)+n,0));
    cv::line(image,p1,p2,c,1);
  }
  //draw points
  for(i = 0; i < n; i++){    
    if(visi.at<int>(i,0) == 0)continue;
    p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));
    c = CV_RGB(255,0,0); cv::circle(image,p1,2,c);
  }return;
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int parse_cmd(int argc, const char** argv,
	      char* ftFile,char* conFile,char* triFile,
	      bool &fcheck,double &scale,int &fpd)
{
  int i; fcheck = false; scale = 1; fpd = -1;
  for(i = 1; i < argc; i++){
    if((std::strcmp(argv[i],"-?") == 0) ||
       (std::strcmp(argv[i],"--help") == 0)){
      std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
	   << "Performs automatic face tracking" << std::endl << std::endl
	   << "#" << std::endl 
	   << "# usage: ./face_tracker [options]" << std::endl
	   << "#" << std::endl << std::endl
	   << "Arguments:" << std::endl
	   << "-m <string> -> Tracker model (default: ../model/face2.tracker)"
	   << std::endl
	   << "-c <string> -> Connectivity (default: ../model/face.con)"
	   << std::endl
	   << "-t <string> -> Triangulation (default: ../model/face.tri)"
	   << std::endl
	   << "-s <double> -> Image scaling (default: 1)" << std::endl
	   << "-d <int>    -> Frames/detections (default: -1)" << std::endl
	   << "--check     -> Check for failure" << std::endl;
      return -1;
    }
  }
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"--check") == 0){fcheck = true; break;}
  }
  if(i >= argc)fcheck = false;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-s") == 0){
      if(argc > i+1)scale = std::atof(argv[i+1]); else scale = 1;
      break;
    }
  }
  if(i >= argc)scale = 1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-d") == 0){
      if(argc > i+1)fpd = std::atoi(argv[i+1]); else fpd = -1;
      break;
    }
  }
  if(i >= argc)fpd = -1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-m") == 0){
      if(argc > i+1)std::strcpy(ftFile,argv[i+1]);
      else strcpy(ftFile,"./model/face.tracker");
      break;
    }
  }
  if(i >= argc)std::strcpy(ftFile,"./model/face.tracker");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-c") == 0){
      if(argc > i+1)std::strcpy(conFile,argv[i+1]);
      else strcpy(conFile,"../model/face.con");
      break;
    }
  }
  if(i >= argc)std::strcpy(conFile,"./model/face.con");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-t") == 0){
      if(argc > i+1)std::strcpy(triFile,argv[i+1]);
      else strcpy(triFile,"../model/face.tri");
      break;
    }
  }
  if(i >= argc)std::strcpy(triFile,"./model/face.tri");
  return 0;
}

int main(int argc, const char** argv)
{
    printf( "\n===== Face tracking Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    BallTracker tracker = BallTracker();

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);		
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
//		return 0;
	}
   MotionManager::GetInstance()->LoadINISettings(ini);		
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());		
   LinuxMotionTimer *motion_timer = new LinuxMotionTimer();		
   motion_timer->Start();

	MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
	MotionManager::GetInstance()->SetEnable(true);
	/////////////////////////////////////////////////////////////////////

	//Head::GetInstance()->LoadINISettings(ini);
	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

	Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
	Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

   printf("Press the ENTER key to begin!\n");
	getchar();

     //parse command line arguments
     char ftFile[256],conFile[256],triFile[256];
     bool fcheck = false; double scale = 1; int fpd = -1; bool show = true;
     if(parse_cmd(argc,argv,ftFile,conFile,triFile,fcheck,scale,fpd)<0)return 0;

     //set other tracking parameters
     std::vector<int> wSize1(1); wSize1[0] = 7;
     std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
     int nIter = 5; double clamp=3,fTol=0.01; 
     FACETRACKER::Tracker model(ftFile);
     cv::Mat tri=FACETRACKER::IO::LoadTri(triFile);
     cv::Mat con=FACETRACKER::IO::LoadCon(conFile);

     //initialize camera and display window
     cv::Mat frame,gray,im; double fps=0; char sss[256]; std::string text; 
     int64 t1,t0 = cvGetTickCount(); int fnum=0;
     bool failed = true;

    while(1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();

         //Create the image header which will contain the converted BYTE image
	      int channel = 3; //1 for black and white pictures, 3 for colored pictures
	      int step = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_WidthStep;//depends from the channel

          //grab image, resize and flip
          IplImage* I = cvCreateImageHeader(cv::Size(LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_Width,LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_Height), IPL_DEPTH_8U, channel);
	      //Set the BYTE pointer as attribut of the IplImage --> "converts" BYTE *pData in IplImage *frame
	      cvSetData(I, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, step); 

          if(!I)continue; frame = I;
          if(scale == 1)im = frame; 
          else cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
          cv::flip(im,im,1); 
          cv::cvtColor(im,gray,CV_RGB2GRAY);

          //track this image
          std::vector<int> wSize; if(failed)wSize = wSize2; else wSize = wSize1; 
          if(model.Track(gray,wSize,fpd,nIter,clamp,fTol,fcheck) == 0){
            int idx = model._clm.GetViewIdx(); failed = false;
            Draw(im,model._shape,con,tri,model._clm._visi[idx]); 
            pos.X = model._rect.x+ model._rect.width/2;
            pos.Y = model._rect.y+ model._rect.height/2;
          }else{
            if(show){cv::Mat R(im,cvRect(0,0,150,50)); R = cv::Scalar(0,0,255);}
            model.FrameReset(); failed = true;
            pos.X = -1;
            pos.Y = -1;
          } 
    
          //draw framerate on display image 
          if(fnum >= 9){      
            t1 = cvGetTickCount();
            fps = 10.0/((double(t1-t0)/cvGetTickFrequency())/1e+6); 
            t0 = t1; fnum = 0;
          }else fnum += 1;
          if(show){
            sprintf(sss,"%d frames/sec",(int)round(fps)); text = sss;
            cv::putText(im,text,cv::Point(10,20),
		        CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
          }
          //show image and check for user input
          imshow("Face Tracker",im); 
          int c = cv::waitKey(10);
          if(c == 27)break; else if(char(c) == 'd')model.Save("/home/koreki/Documents/perso/darwin/Linux/project/tutorial/face_tracker/koreki.tracker");//model.FrameReset();
//model.FrameReset();
//        pos = facefinder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_RGBFrame);
        tracker.Process(pos);

        fprintf(stderr, "posx: %f, posy: %f \r", pos.X, pos.Y);

		  rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
        rgb_ball->m_ImageData = im.data;

        streamer->send_image(rgb_ball);
    }
    return 0;
}
