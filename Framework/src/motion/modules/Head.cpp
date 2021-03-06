/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "MX28.h"
#include "AX12.h"
#include "Kinematics.h"
#include "MotionStatus.h"
#include "Head.h"

//joystick add
#include <stdlib.h>
//joystick add

using namespace Robot;


Head* Head::m_UniqueInstance = new Head();

Head::Head()
{
	m_Pan_p_gain = 0.1;
	m_Pan_d_gain = 0.05;

   m_Tilt_p_gain = 0.1;
	m_Tilt_d_gain = 0.05;

	m_LeftLimit = 70;
	m_RightLimit = -70;
	m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE;
	m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65;

// joystick add
   m_stick = NULL;
// joystick add

	m_Pan_Home = 0.0;
	m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0;
//	m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE;

	m_Joint.SetEnableHeadOnly(true);
}

Head::~Head()
{
}

void Head::CheckLimit()
{
	if(m_PanAngle > m_LeftLimit)
		m_PanAngle = m_LeftLimit;
	else if(m_PanAngle < m_RightLimit)
		m_PanAngle = m_RightLimit;

	if(m_TiltAngle > m_TopLimit)
		m_TiltAngle = m_TopLimit;
	else if(m_TiltAngle < m_BottomLimit)
		m_TiltAngle = m_BottomLimit;	
}

void Head::Initialize()
{
	m_PanAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	m_TiltAngle = -MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	CheckLimit();

	InitTracking();
	MoveToHome();
}

void Head::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, HEAD_SECTION);
}

void Head::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "pan_p_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_p_gain = value;
    if((value = ini->getd(section, "pan_d_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_d_gain = value;
    if((value = ini->getd(section, "tilt_p_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_p_gain = value;
    if((value = ini->getd(section, "tilt_d_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_d_gain = value;
    if((value = ini->getd(section, "left_limit", INVALID_VALUE)) != INVALID_VALUE)  m_LeftLimit = value;
    if((value = ini->getd(section, "right_limit", INVALID_VALUE)) != INVALID_VALUE) m_RightLimit = value;
    if((value = ini->getd(section, "top_limit", INVALID_VALUE)) != INVALID_VALUE)   m_TopLimit = value;
    if((value = ini->getd(section, "bottom_limit", INVALID_VALUE)) != INVALID_VALUE)m_BottomLimit = value;
    if((value = ini->getd(section, "pan_home", INVALID_VALUE)) != INVALID_VALUE)    m_Pan_Home = value;
    if((value = ini->getd(section, "tilt_home", INVALID_VALUE)) != INVALID_VALUE)   m_Tilt_Home = value;
}

void Head::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, HEAD_SECTION);
}

void Head::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "pan_p_gain",   m_Pan_p_gain);
    ini->put(section,   "pan_d_gain",   m_Pan_d_gain);
    ini->put(section,   "tilt_p_gain",  m_Tilt_p_gain);
    ini->put(section,   "tilt_d_gain",  m_Tilt_d_gain);
    ini->put(section,   "left_limit",   m_LeftLimit);
    ini->put(section,   "right_limit",  m_RightLimit);
    ini->put(section,   "top_limit",    m_TopLimit);
    ini->put(section,   "bottom_limit", m_BottomLimit);
    ini->put(section,   "pan_home",     m_Pan_Home);
    ini->put(section,   "tilt_home",    m_Tilt_Home);
}

void Head::MoveToHome()
{
	MoveByAngle(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveByAngle(double pan, double tilt)
{
	m_PanAngle = pan;
	m_TiltAngle = tilt;

	CheckLimit();
}

void Head::MoveByAngleOffset(double pan, double tilt)
{	
	MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}

void Head::InitTracking()
{
	m_Pan_err = 0;
	m_Pan_err_diff = 0;
	m_Tilt_err = 0;
	m_Tilt_err_diff = 0;
}

void Head::MoveTracking(Point2D err)
{	
	m_Pan_err_diff = err.X - m_Pan_err;
	m_Pan_err = err.X;

	m_Tilt_err_diff = err.Y - m_Tilt_err;
	m_Tilt_err = err.Y;

	MoveTracking();
}

void Head::MoveTracking()
{
	double pOffset, dOffset;

	// Do nothing if we do not have control. This prevents run-away
	// on pan/tilt values.
	if(m_Joint.GetEnable(JointData::ID_HEAD_PAN))
	{
		pOffset = m_Pan_err * m_Pan_p_gain;
		pOffset *= pOffset;
		if(m_Pan_err < 0)
			pOffset = -pOffset;
		dOffset = m_Pan_err_diff * m_Pan_d_gain;
		dOffset *= dOffset;
		if(m_Pan_err_diff < 0)
			dOffset = -dOffset;
		m_PanAngle += (pOffset + dOffset);
	}

	if(m_Joint.GetEnable(JointData::ID_HEAD_TILT))
	{
		pOffset = m_Tilt_err * m_Tilt_p_gain;
		pOffset *= pOffset;
		if(m_Tilt_err < 0)
			pOffset = -pOffset;
		dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
		dOffset *= dOffset;
		if(m_Tilt_err_diff < 0)
			dOffset = -dOffset;
		m_TiltAngle += (pOffset + dOffset);
	}
	CheckLimit();
}

void Head::Process()
{
// joystick add
   // Use joystick values if available & wanted
   if(m_stick) 
   {
      m_stick->update();
      if(m_stick->axis[4]==1)
         m_PanAngle -= 0.75;
      else if(m_stick->axis[4]==-1)
         m_PanAngle += 0.75;

      if(m_stick->axis[5]==1)
         m_TiltAngle -= 0.75;
      else if(m_stick->axis[5]==-1)
         m_TiltAngle += 0.75;

      CheckLimit();

      if(m_stick->button[1])
         MoveToHome();

   }
// joystick add

	if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);
	else
		m_PanAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);

	if(m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);
	else
		m_TiltAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
}

// joystick add
void Head::useJoystick() 
{
   m_stick = new Joystick();
   if(!m_stick->init())
   {
      delete m_stick;
      m_stick = 0;
      fprintf(stderr,"Error: Joystick not found\n");
      exit(-1);
   }
}

void Head::closeJoystick() 
{
   delete m_stick;
}
// joystick add
