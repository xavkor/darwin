/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Kinematics.h"

using namespace Robot;

/* Darwin specifications 
*/
/*
const double Kinematics::CAMERA_DISTANCE = 33.2; //mm
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree
const double Kinematics::LEG_SIDE_OFFSET = 37.0; //mm
const double Kinematics::THIGH_LENGTH = 93.0; //mm
const double Kinematics::CALF_LENGTH = 93.0; //mm
const double Kinematics::ANKLE_LENGTH = 33.5; //mm
const double Kinematics::LEG_LENGTH = 219.5; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
*/
/* Bioloid specifications 
*/
/*
const double Kinematics::CAMERA_DISTANCE = 40; //mm
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree
const double Kinematics::LEG_SIDE_OFFSET = 38.0; //mm
const double Kinematics::THIGH_LENGTH = 75.0; //mm
const double Kinematics::CALF_LENGTH = 75.0; //mm
const double Kinematics::ANKLE_LENGTH = 33.0; //mm
const double Kinematics::LEG_LENGTH = 188.0; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
*/
/* My specifications 
*/
const double Kinematics::CAMERA_DISTANCE = 40.0; //mm
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0; //degree
const double Kinematics::LEG_SIDE_OFFSET = 40.0; //mm
const double Kinematics::THIGH_LENGTH = 93.0; //mm
const double Kinematics::CALF_LENGTH = 93.0; //mm
const double Kinematics::ANKLE_LENGTH = 33.0; //mm
const double Kinematics::LEG_LENGTH = 219.0; //mm

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}
