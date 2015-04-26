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
#define MOTION_FILE_PATH    "motion_hybrid.bin"

using namespace Robot;

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
   Joystick* joy = new Joystick();
   if(!joy->init())
   {
      printf("Error: Joystick not found\n");
//	   return 0;
   }

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
   Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);

   MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

   LinuxMotionTimer *motion_timer = new LinuxMotionTimer();
   motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

   Walking::GetInstance()->useJoystick(); 
   Head::GetInstance()->useJoystick();

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	MotionManager::GetInstance()->SetEnable(true);

   Action::GetInstance()->Start(15);    /* SitDown pose */
   while(Action::GetInstance()->IsRunning()) usleep(8*1000);
   Action::GetInstance()->Start(9);    /* WalkReady pose */
   while(Action::GetInstance()->IsRunning()) usleep(8*1000);

	printf("Press the ENTER key to begin!\n");
	getchar();
	printf("START process ...\n");

   while(!joy->button[7])
   {
      if(joy->button[2] || joy->button[3] || joy->button[4] || joy->button[6])
      {
         Walking::GetInstance()->Stop();
	      Action::GetInstance()->m_Joint.SetEnableBody(true, true);
         if(joy->button[2])
         {
            Action::GetInstance()->Start(15);    /* SitDown pose */
            while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
         }
         if(joy->button[3])
         {
            Action::GetInstance()->Start(9);    /* WalkReady pose */
            while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
         }
         if(joy->button[6])
         {
            Action::GetInstance()->Start(10);    /* forward up pose */
            while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
         }
//         if(joy.button[4])
//         {
//            Action::GetInstance()->Start(11);    /* backward up pose */
//            while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//         }
      }

      if(joy->button[0])
      {
         Action::GetInstance()->Start(9);    /* WalkReady pose */
         while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
         Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
         Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);   
      }
      LinuxCamera::GetInstance()->CaptureFrame();
      streamer->send_image(LinuxCamera::GetInstance()->fbuffer->m_YUVFrame);
      joy->update();
   }

	printf("STOP process\n");

   Walking::GetInstance()->Stop();

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
   Action::GetInstance()->Start(15);    /* SitDown pose */
   while(Action::GetInstance()->IsRunning()) usleep(8*1000);

   Action::GetInstance()->Stop();

   Walking::GetInstance()->closeJoystick(); 
   Head::GetInstance()->closeJoystick();
   delete joy;

   int n = 0;
   int param[JointData::NUMBER_OF_JOINTS * 2];
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		param[n++] = id;
		param[n++] = 0;
   }
	cm730.SyncWrite(MX28::P_TORQUE_ENABLE, 2, JointData::NUMBER_OF_JOINTS - 1, param);

   return 0;
}
