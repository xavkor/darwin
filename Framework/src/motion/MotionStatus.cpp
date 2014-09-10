/*
 *   MotionStatus.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MotionStatus.h"

using namespace Robot;

JointData MotionStatus::m_CurrentJoints;

// Start angle estimator add
//int MotionStatus::FB_GYRO(0);
//int MotionStatus::RL_GYRO(0);
double MotionStatus::FB_GYRO(0);
double MotionStatus::RL_GYRO(0);
// End angle estimator add

int MotionStatus::FB_ACCEL(0);
int MotionStatus::RL_ACCEL(0);

int MotionStatus::BUTTON(0);
int MotionStatus::FALLEN(0);

// Start angle estimator add
double MotionStatus::ANGLE_PITCH(0);
double MotionStatus::ANGLE_ROLL(0);
// End angle estimator add
