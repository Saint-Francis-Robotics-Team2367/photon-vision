#pragma once

#define PI 3.14159265359
#define PI_2 1.570796326

#define driveEncConvFactor 6.86 * 2 * M_PI
#define steerEncConvFactor 12.8 * 2 * M_PI

// Controller Settings
#define ctrDeadzone 0.09
#define ctrSlewRate 0.3

// Module Constraints
#define moduleMaxFPS 17.111 // feet per sec
#define moduleMaxRPM 6000 // RPM
#define moduleMaxRot 2.0 // 9.678 / 2, Radians/sec

// Drivebase Measurements
#define trackWidthNumber 2.375          // feet
#define wheelBase 2.375           // feet
#define moduleDriveRatio 6.12     // L3
#define wheelRadiusInches 2       // inches
#define wheelCircumFeet 1.0471976 // feet

// Util
#define kEpsilon 1e-12
#define loopTime 0.02

constexpr float velocityP = 6e-5;
constexpr float velocityI = 1e-6;
constexpr float velocityD = 0.0;
constexpr float velocityFF = 0.000015;

constexpr float positionP = 0.06;
constexpr float positionI = 0.0;
constexpr float positionD = 0.0;
constexpr float positionFF = 0.0;

// Motor/CAN IDs
#define FLsteerID 5
#define FLdriveID 8
#define FL_CAN_ID 3 // updated

#define FRsteerID 10
#define FRdriveID 4
#define FR_CAN_ID 1 // updated

#define BLsteerID 1
#define BLdriveID 6
#define BL_CAN_ID 2 // updated;

#define BRsteerID 3
#define BRdriveID 2
#define BR_CAN_ID 0 // updated

// Steer PID values(custom, untuned)
constexpr float steerP = 0.3;
constexpr float steerI = 0.0;
constexpr float steerD = 0.0;
constexpr float steerIZone = 0.0;

class Constants
{
public:
};