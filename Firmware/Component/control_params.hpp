#ifndef __CONTROL_PARAMS_HPP
#define __CONTROL_PARAMS_HPP

namespace control_config {

inline constexpr float kControlDtSec = 1.0f / 500.0f;
inline constexpr float kPi = 3.1415926535f;

inline constexpr float kAccThresholdX = 2.0f;
inline constexpr float kAccThresholdY = 2.0f;
inline constexpr float kAccThresholdYaw = 10.0f;

inline constexpr float kVelErrGainX = 5.0f;
inline constexpr float kVelErrGainY = 5.0f;
inline constexpr float kVelErrGainYaw = 5.0f;

inline constexpr float kJerkLimitX = 200.0f;
inline constexpr float kJerkLimitY = 200.0f;
inline constexpr float kJerkLimitYaw = 800.0f;

// inline constexpr float kRobotMassKg = 2.19692f;
inline constexpr float kRobotMassKg = 0.05f;
inline constexpr float kRobotInertiaKgM2 = 2.29587357e-6f;
inline constexpr float kWheelRadiusM = 0.0285f;
inline constexpr float kWheelCenterDistanceM = 0.073f;
inline constexpr float kCenterToComDistanceM = 0.0f;
inline constexpr float kWheelAlphaRad = 30.0f / 180.0f * kPi;
inline constexpr float kWheelBetaRad = 45.0f / 180.0f * kPi;

inline constexpr float kLesoVelObserverBandwidth = 10.0f;
inline constexpr float kLesoYawObserverBandwidth = 10.0f;

inline constexpr float kVelFeedbackGainX = 5.0f;
inline constexpr float kVelFeedbackGainY = 5.0f;
inline constexpr float kVelFeedbackGainYaw = 5.0f;

inline constexpr float kWheelTorqueFfLimitNm = 0.05f;

inline constexpr float kMITRunKp = 0.0f;
inline constexpr float kMITRunKd = 0.001f;
inline constexpr float kMITRunTorqueFf = 0.0f;

inline constexpr float kMITSafeKp = 0.0f;
inline constexpr float kMITSafeKd = 0.0f;
inline constexpr float kMITSafeTorqueFf = 0.0f;

} // namespace control_config

#endif // __CONTROL_PARAMS_HPP