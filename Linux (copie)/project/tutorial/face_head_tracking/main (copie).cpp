/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "FaceFinder.h"

#define INI_FILE_PATH       "config.ini"

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

    while(1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        pos = facefinder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_RGBFrame);

        fprintf(stderr, "posx: %f, posy: %f \r", pos.X, pos.Y);

		  rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        streamer->send_image(rgb_ball);
    }
    return 0;
}
