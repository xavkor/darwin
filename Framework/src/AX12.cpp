/*
 *   AX12.cpp
 *
 *  
 *
 */
#include "AX12.h"

using namespace Robot;

const int AX12::MIN_VALUE = 0;
const int AX12::CENTER_VALUE = 512;
const int AX12::MAX_VALUE = 1023;
const double AX12::MIN_ANGLE = -150.0; // degree
const double AX12::MAX_ANGLE = 150.0; // degree
const double AX12::RATIO_VALUE2ANGLE = 0.293; // 300 / 1024
const double AX12::RATIO_ANGLE2VALUE = 3.413; // 1024 / 300

const int AX12::PARAM_BYTES = 5;
