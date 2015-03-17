/*
 * FaceFinder.cpp
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#include <stdlib.h>

#include "FaceFinder.h"
#include "ImgProcess.h"

using namespace Robot;

CascadeClassifier face_cascade;

FaceFinder::FaceFinder() :
        m_center_point(Point2D()),
        face_cascade_name("haarcascade_frontalface_alt.xml"),
        count(0)
{  
   /** Global variables */
   //-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
//   String face_cascade_name = "haarcascade_frontalface_alt.xml";
  
}

FaceFinder::~FaceFinder()
{
    // TODO Auto-generated destructor stub
}

void FaceFinder::LoadINISettings()
{
   if( !face_cascade.load( face_cascade_name ) )
   { 
      printf("--(!)Error loading\n"); 
      exit(-1); 
   };
}

Point2D& FaceFinder::GetPosition(Image* rgb_img)
{
   //Create the image header which will contain the converted BYTE image
	int channel = 3; //1 for black and white pictures, 3 for colored pictures
	int step = rgb_img->m_WidthStep;//depends from the channel
	IplImage *tmp = cvCreateImageHeader(Size(rgb_img->m_Width,rgb_img->m_Height), IPL_DEPTH_8U, channel);		

	//Set the BYTE pointer as attribut of the IplImage --> "converts" BYTE *pData in IplImage *frame
	cvSetData(tmp, rgb_img->m_ImageData, step); 

   Mat frame(tmp);
   std::vector<Rect> faces;
   Mat frame_gray,frame_tmp;

   cvtColor( frame, frame_gray, CV_RGB2GRAY );
   equalizeHist( frame_gray, frame_gray );

   //-- Detect faces
   face_cascade.detectMultiScale( frame_gray, faces, 1.1, 3, CV_HAAR_SCALE_IMAGE, Size(60, 60) );

   if(faces.size()==0)
      count++;
   else
      count=0;

   if(count>50)
   {
      m_center_point.X=-1.0;
      m_center_point.Y=-1.0;
   }

   for( size_t i = 0; i < faces.size(); i++ )
   {
      if(i==0)
      {
         Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
         ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );
         m_center_point.X=center.x;
         m_center_point.Y=center.y;
      }
   }
   return m_center_point;
}
