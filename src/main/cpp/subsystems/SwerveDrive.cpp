/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "subsystems/SwerveDrive.h"  // Include the header file for the SwerveDrive class

#include <iostream>  // Include the iostream library for input/output operations
#include <string>  // Include the string library for string operations

#include <frc/TimedRobot.h>  // Include the TimedRobot class from FRC
#include <frc/smartdashboard/SmartDashboard.h>  // Include the SmartDashboard library from FRC

// Constructor for the SwerveDrive class
SwerveDrive::SwerveDrive()
    : poseEstimator(kinematics, GetGyroYaw(), GetModulePositions(),
                    frc::Pose2d{}, {0.1, 0.1, 0.1}, {1.0, 1.0, 1.0}),  // Initialize the pose estimator
      gyroSim(gyro),  // Initialize the gyro simulation
      swerveDriveSim(constants::Swerve::kDriveFF, frc::DCMotor::Falcon500(1),
                     constants::Swerve::kDriveGearRatio,
                     constants::Swerve::kWheelDiameter / 2,
                     constants::Swerve::kSteerFF, frc::DCMotor::Falcon500(1),
                     constants::Swerve::kSteerGearRatio, kinematics) {}  // Initialize the swerve drive simulation

// Periodic function called periodically during runtime
void SwerveDrive::Periodic() {
  for (auto& currentModule : swerveMods) {  // Iterate over swerve modules
    currentModule.Periodic();  // Call the periodic function of the current module
  }

  poseEstimator.Update(GetGyroYaw(), GetModulePositions());  // Update the pose estimator
}

// Drive the robot with specified velocities
void SwerveDrive::Drive(units::meters_per_second_t vx,
                        units::meters_per_second_t vy,
                        units::radians_per_second_t omega) {
  frc::ChassisSpeeds newChassisSpeeds =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetHeading());  // Calculate chassis speeds
  SetChassisSpeeds(newChassisSpeeds, true, false);  // Set the chassis speeds
}

// Set the chassis speeds
void SwerveDrive::SetChassisSpeeds(const frc::ChassisSpeeds& newChassisSpeeds,
                                   bool openLoop, bool steerInPlace) {
  SetModuleStates(kinematics.ToSwerveModuleStates(newChassisSpeeds), true,
                  steerInPlace);  // Set the module states
  this->targetChassisSpeeds = newChassisSpeeds;  // Store the target chassis speeds
}

// Set the module states
void SwerveDrive::SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop,
    bool steerInPlace) {
  std::array<frc::SwerveModuleState, 4> desaturatedStates = desiredStates;  // Copy the desired states
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      static_cast<wpi::array<frc::SwerveModuleState, 4>*>(&desaturatedStates),
      constants::Swerve::kMaxLinearSpeed);  // Desaturate the wheel speeds
  for (int i = 0; i < swerveMods.size(); i++) {  // Iterate over swerve modules
    swerveMods[i].SetDesiredState(desaturatedStates[i], openLoop, steerInPlace);  // Set the desired state of the current module
  }
}

// Stop the robot
void SwerveDrive::Stop() { Drive(0_mps, 0_mps, 0_rad_per_s); }

// Reset the pose of the robot
void SwerveDrive::ResetPose(const frc::Pose2d& pose, bool resetSimPose) {
  if (resetSimPose) {  // Check if the simulation pose should be reset
    swerveDriveSim.Reset(pose, false);  // Reset the swerve drive simulation
    for (int i = 0; i < swerveMods.size(); i++) {  // Iterate over swerve modules
      swerveMods[i].SimulationUpdate(0_m, 0_mps, 0_A, 0_rad, 0_rad_per_s, 0_A);  // Reset the simulation values of the current module
    }
    gyroSim.SetAngle(-pose.Rotation().Degrees());  // Set the gyro angle
    gyroSim.SetRate(0_rad_per_s);  // Set the gyro rate
  }

  poseEstimator.ResetPosition(GetGyroYaw(), GetModulePositions(), pose);  // Reset the pose estimator
}

// Get the pose of the robot
frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();  // Return the estimated position
}

// Get the heading of the robot
frc::Rotation2d SwerveDrive::GetHeading() const { return GetPose().Rotation(); }

// Get the yaw of the gyro
frc::Rotation2d SwerveDrive::GetGyroYaw() const { return gyro.GetRotation2d(); }

// Get the chassis speeds
frc::ChassisSpeeds SwerveDrive::GetChassisSpeeds() const {
  return kinematics.ToChassisSpeeds(GetModuleStates());  // Return the chassis speeds
}

// Get the module states
std::array<frc::SwerveModuleState, 4> SwerveDrive::GetModuleStates() const {
  std::array<frc::SwerveModuleState, 4> moduleStates;  // Array to store module states
  moduleStates[0] = swerveMods[0].GetState();  // Get the state of module 0
  moduleStates[1] = swerveMods[1].GetState();  // Get the state of module 1
  moduleStates[2] = swerveMods[2].GetState();  // Get the state of module 2
  moduleStates[3] = swerveMods[3].GetState();  // Get the state of module 3
  return moduleStates;  // Return the module states
}

// Get the module positions
std::array<frc::SwerveModulePosition, 4> SwerveDrive::GetModulePositions()
    const {
  std::array<frc::SwerveModulePosition, 4> modulePositions;  // Array to store module positions
  modulePositions[0] = swerveMods[0].GetPosition();  // Get the position of module 0
  modulePositions[1] = swerveMods[1].GetPosition();  // Get the position of module 1
  modulePositions[2] = swerveMods[2].GetPosition();  // Get the position of module 2
  modulePositions[3] = swerveMods[3].GetPosition();  // Get the position of module 3
  return modulePositions;  // Return the module positions
}

// Get the module poses
std::array<frc::Pose2d, 4> SwerveDrive::GetModulePoses() const {
  std::array<frc::Pose2d, 4> modulePoses;  // Array to store module poses
  for (int i = 0; i < swerveMods.size(); i++) {  // Iterate over swerve modules
    const SwerveModule& module = swerveMods[i];  // Get the current module
    modulePoses[i] = GetPose().TransformBy(frc::Transform2d{
        module.GetModuleConstants().centerOffset, module.GetAbsoluteHeading()});  // Calculate the pose of the current module
  }
  return modulePoses;  // Return the module poses
}

// Log the current state of the swerve drive to the SmartDashboard
void SwerveDrive::Log() {
  std::string table = "Drive/";  // Create a string for the SmartDashboard table name
  frc::Pose2d pose = GetPose();  // Get the pose of the robot
  frc::SmartDashboard::PutNumber(table + "X", pose.X().to<double>());  // Log the X position
  frc::SmartDashboard::PutNumber(table + "Y", pose.Y().to<double>());  // Log the Y position
  frc::SmartDashboard::PutNumber(table + "Heading",
                                 pose.Rotation().Degrees().to<double>());  // Log the heading
  frc::ChassisSpeeds chassisSpeeds = GetChassisSpeeds();  // Get the chassis speeds
  frc::SmartDashboard::PutNumber(table + "VX", chassisSpeeds.vx.to<double>());  // Log the X velocity
  frc::SmartDashboard::PutNumber(table + "VY", chassisSpeeds.vy.to<double>());  // Log the Y velocity
  frc::SmartDashboard::PutNumber(
      table + "Omega Degrees",
      chassisSpeeds.omega.convert<units::degrees_per_second>().to<double>());  // Log the angular velocity
  frc::SmartDashboard::PutNumber(table + "Target VX",
                                 targetChassisSpeeds.vx.to<double>());  // Log the target X velocity
  frc::SmartDashboard::PutNumber(table + "Target VY",
                                 targetChassisSpeeds.vy.to<double>());  // Log the target Y velocity
  frc::SmartDashboard::PutNumber(
      table + "Target Omega Degrees",
      targetChassisSpeeds.omega.convert<units::degrees_per_second>()
          .to<double>());  // Log the target angular velocity

  for (auto& module : swerveMods) {  // Iterate over swerve modules
    module.Log();  // Log the current state of the current module
  }
}

// Periodic function called periodically during simulation
void SwerveDrive::SimulationPeriodic() {
  std::array<units::volt_t, 4> driveInputs;  // Array to store drive inputs
  std::array<units::volt_t, 4> steerInputs;  // Array to store steer inputs
  for (int i = 0; i < swerveMods.size(); i++) {  // Iterate over swerve modules
    driveInputs[i] = swerveMods[i].GetDriveVoltage();  // Get the drive voltage of the current module
    steerInputs[i] = swerveMods[i].GetSteerVoltage();  // Get the steer voltage of the current module
  }
  swerveDriveSim.SetDriveInputs(driveInputs);  // Set the drive inputs for the simulation
  swerveDriveSim.SetSteerInputs(steerInputs);  // Set the steer inputs for the simulation

  swerveDriveSim.Update(frc::TimedRobot::kDefaultPeriod);  // Update the simulation

  auto driveStates = swerveDriveSim.GetDriveStates();  // Get the drive states from the simulation
  auto steerStates = swerveDriveSim.GetSteerStates();  // Get the steer states from the simulation
  totalCurrentDraw = 0_A;  // Initialize the total current draw
  std::array<units::ampere_t, 4> driveCurrents =
      swerveDriveSim.GetDriveCurrentDraw();  // Get the drive currents from the simulation
  for (const auto& current : driveCurrents) {  // Iterate over drive currents
    totalCurrentDraw += current;  // Add the current to the total current draw
  }
  std::array<units::ampere_t, 4> steerCurrents =
      swerveDriveSim.GetSteerCurrentDraw();  // Get the steer currents from the simulation
  for (const auto& current : steerCurrents) {  // Iterate over steer currents
    totalCurrentDraw += current;  // Add the current to the total current draw
  }
  for (int i = 0; i < swerveMods.size(); i++) {  // Iterate over swerve modules
    units::meter_t drivePos{driveStates[i](0, 0)};  // Get the drive position of the current module
    units::meters_per_second_t driveRate{driveStates[i](1, 0)};  // Get the drive rate of the current module
    units::radian_t steerPos{steerStates[i](0, 0)};  // Get the steer position of the current module
    units::radians_per_second_t steerRate{steerStates[i](1, 0)};  // Get the steer rate of the current module
    swerveMods[i].SimulationUpdate(drivePos, driveRate, driveCurrents[i],
                                   steerPos, steerRate, steerCurrents[i]);  // Update the simulation values of the current module
  }
  gyroSim.SetRate(-swerveDriveSim.GetOmega());  // Set the gyro rate
  gyroSim.SetAngle(-swerveDriveSim.GetPose().Rotation().Degrees());  // Set the gyro angle
}

// Get the simulation pose of the robot
frc::Pose2d SwerveDrive::GetSimPose() const { return swerveDriveSim.GetPose(); }

// Get the current draw of the robot
units::ampere_t SwerveDrive::GetCurrentDraw() const { return totalCurrentDraw; }