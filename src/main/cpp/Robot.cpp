// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>

void Robot::RobotInit()
{
  mDrive.initModules();
  mGyro.init();

}
void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mGyro.init();
  mDrive.enableModules();
 
  // if (frc::DriverStation::IsDSAttached()) {
  //   mTraj.isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
  // }
  // if (mLimelight.targetDetected()) {
  //   mTraj.startPose = mLimelight.getRobotPoseFieldSpace();
  //   mTraj.receivedPose = true;
  // } else {
  //   mTraj.receivedPose = false;
  // }
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{

  mDrive.state = DriveState::Teleop;

  mDrive.enableModules();
  mGyro.init();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  /*if (ctr.GetCircleButtonPressed()) {
    mDrive.autoMove(PI/2, 100);
  }
  */
  if (ctr.GetCircleButtonPressed()) {
    mDrive.mFrontLeft.steerMotor.Set(0.5);
  }

  auto startTime = frc::Timer::GetFPGATimestamp();
  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = xStickLimiter.calculate(leftX); 
  leftY = yStickLimiter.calculate(leftY);

  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  //int dPad = ctrOperator.GetPOV();
  bool rumbleController = false;

  // Driver Information

  // Teleop States
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX * moduleMaxRot * 2;

  // Decide drive modes
  //if (snapRobotToGoal.update(dPad >= 0 && !driveTurning, 5.0, driveTurning)) // SNAP mode
  //{
  //  mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
  //  mHeadingController.setSetpointPOV(dPad);
  //}
  // else if (preScoringSpeaker && !driveTurning) // ALIGN(scoring) mode
  // {
  //   // if (mLimelight.isSpeakerTagDetected())
  //   // {
  //   //   Pose3d target = mLimelight.getTargetPoseRobotSpace();
  //   //   double angleOffset = Rotation2d::polarToCompass(atan2(target.y, target.x)) * 180 / PI;
  //   //   double zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
  //   //   mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
  //   //   mHeadingController.setSetpoint(zeroSetpoint);
  //   // }
  // }
  //else // Normal driving mode
  //{
  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  //}

  // Output heading controller if used
  rot = mHeadingController.getHeadingControllerState() == SwerveHeadingController::OFF
            ? rot
            : mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());

  // Gyro Resets
  if (ctr.GetCrossButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, -rot),
      mGyro.getBoundedAngleCCW(),
      mGyro.gyro.IsConnected(),
      cleanDriveAccum);
  mDrive.updateOdometry();
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.stopModules();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif