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
#include "Joystick.h"
#include "MX28.h"
#include "AX12.h"
#include "minIni.h"
#include "LinuxCamera.h"

#include "LinuxDARwIn.h"

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
   printf( "\n===== Test for Hybrid =====\n\n");

   change_current_dir();

   minIni* ini = new minIni(INI_FILE_PATH);

   LinuxCamera::GetInstance()->Initialize(0);
   LinuxCamera::GetInstance()->LoadINISettings(ini);

   mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);	
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
   MotionManager::GetInstance()->LoadINISettings(ini);
   Walking::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
   LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
   motion_timer->Start();
	/////////////////////////////////////////////////////////////////////
	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
      //printf("id %d goal %d\n",id,wGoalPosition);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

   Walking::GetInstance()->useJoystick(); 
   Head::GetInstance()->useJoystick();

	printf("Press the ENTER key to begin!\n");
	getchar();
	printf("Process START\n");
	
   Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
   Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

   while(Walking::GetInstance()->endProg())
   {
      LinuxCamera::GetInstance()->CaptureFrame();
      streamer->send_image(LinuxCamera::GetInstance()->fbuffer->m_YUVFrame);
   };

   Walking::GetInstance()->Stop();
   Walking::GetInstance()->closeJoystick(); 
   Head::GetInstance()->closeJoystick();

   n=0;
   int param2[JointData::NUMBER_OF_JOINTS * 2];
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		param2[n++] = id;
		param2[n++] = 0;
   }
	cm730.SyncWrite(MX28::P_TORQUE_ENABLE, 2, JointData::NUMBER_OF_JOINTS - 1, param2);

   return 0;
}
