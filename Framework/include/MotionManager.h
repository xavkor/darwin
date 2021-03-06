/*
 *   MotionManager.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_MANGER_H_
#define _MOTION_MANGER_H_

#include <list>
#include <fstream>
#include <iostream>
#include "MotionStatus.h"
#include "MotionModule.h"
#include "CM730.h"
#include "minIni.h"
// Start angle estimator add
#include "AngleEstimator.h"
// End angle estimator add

// Start RAZOR add
#include "RazorAHRS.h"
// End RAZOR add

#define OFFSET_SECTION "Offset"
#define INVALID_VALUE   -1024.0

namespace Robot
{
	class MotionManager
	{
	private:
		static MotionManager* m_UniqueInstance;
		std::list<MotionModule*> m_Modules;
		CM730 *m_CM730;
		bool m_ProcessEnable;
		bool m_Enabled;
		int m_FBGyroCenter;
		int m_RLGyroCenter;
		int m_CalibrationStatus;

		bool m_IsRunning;
		bool m_IsThreadRunning;
		bool m_IsLogging;

		std::ofstream m_LogFileStream;

// Start angle estimator add
      AngleEstimator m_angleEstimator;
// End angle estimator add

      MotionManager();

	protected:

	public:
		bool DEBUG_PRINT;
      int m_Offset[JointData::NUMBER_OF_JOINTS];

		~MotionManager();

		static MotionManager* GetInstance() { return m_UniqueInstance; }

		bool Initialize(CM730 *cm730);
		bool Reinitialize();
      void Process();
		void SetEnable(bool enable);
		bool GetEnable()				{ return m_Enabled; }
		void AddModule(MotionModule *module);
		void RemoveModule(MotionModule *module);

		void ResetGyroCalibration() { m_CalibrationStatus = 0; m_FBGyroCenter = 512; m_RLGyroCenter = 512; }
		int GetCalibrationStatus() { return m_CalibrationStatus; }
		void SetJointDisable(int index);

		void StartLogging();
		void StopLogging();

      void LoadINISettings(minIni* ini);
      void LoadINISettings(minIni* ini, const std::string &section);
      void SaveINISettings(minIni* ini);
      void SaveINISettings(minIni* ini, const std::string &section);

// Start RAZOR add
      std::string serial_port_name;
      RazorAHRS *razor;
// End RAZOR add

// Start angle estimator add
		inline AngleEstimator* angleEstimator()
		{ return &m_angleEstimator; }
// End angle estimator add
	};
}

#endif
