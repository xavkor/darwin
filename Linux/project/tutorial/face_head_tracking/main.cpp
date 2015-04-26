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
#include "FaceFinder.h"

#define INI_FILE_PATH       "config.ini"

#define U2D_DEV_NAME        "/dev/ttyUSB0"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Face tracking Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    FaceFinder* facefinder = new FaceFinder();
    facefinder->LoadINISettings();

    BallTracker tracker = BallTracker();

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);		
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
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

        //printf("Press the ENTER key to begin!\n");
	//getchar();

    while(1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();

        pos = facefinder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_RGBFrame);
        tracker.Process(pos);

        fprintf(stderr, "posx: %f, posy: %f \r", pos.X, pos.Y);

		  rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        streamer->send_image(rgb_ball);
    }
    return 0;
}
