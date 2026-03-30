#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

#include "Task/z_main.h"

namespace control_config {

inline constexpr float kControlDtSec = static_cast<float>(TIM2_PERIOD_CLOCKS) / 1000000.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 8.0f;
inline constexpr float kAccThresholdY = 8.0f;
inline constexpr float kAccThresholdYaw = 40.0f;

inline constexpr float kVelErrGainX = 1.0f;
inline constexpr float kVelErrGainY = 1.0f;
inline constexpr float kVelErrGainYaw = 2.0f;

inline constexpr float kJerkLimitX = 200.0f;
inline constexpr float kJerkLimitY = 200.0f;
inline constexpr float kJerkLimitYaw = 800.0f;

inline constexpr float kRobotMassKg = 2.19692f;
inline constexpr float kRobotInertiaKgM2 = 2.29587357e-6f;
inline constexpr float kWheelRadiusM = 0.0285f;
inline constexpr float kWheelCenterDistanceM = 0.073f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 2.0f;
inline constexpr float kLesoYawObserverBandwidth = 2.0f;

inline constexpr float kVelFeedbackGainX = 0.02f;
inline constexpr float kVelFeedbackGainY = 0.02f;
inline constexpr float kVelFeedbackGainYaw = 0.02f;

inline constexpr float kWheelTorqueFfLimitNm = 0.00f;

inline constexpr float kWheelSpeedPidKp = 0.0f;
inline constexpr float kWheelSpeedPidKi = 0.0f;
inline constexpr float kWheelSpeedPidKd = 0.0f;
inline constexpr float kWheelSpeedPidBackCalcGain = 1.0f;
inline constexpr float kWheelSpeedPidOutputLimitNm = 0.20f;
inline constexpr float kWheelSpeedPidIntegLimitNm = 0.20f;
inline constexpr float kWheelSpeedFilterCutoffHz = 40.0f;

inline constexpr float kMITRunKp = 0.0f;
inline constexpr float kMITRunKd = 0.075f;
inline constexpr float kMITRunTorqueFf = 0.0f;
inline constexpr float kMITKdRampTimeSec = 1.0f;

inline constexpr float kMITSafeKp = 0.0f;
inline constexpr float kMITSafeKd = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP