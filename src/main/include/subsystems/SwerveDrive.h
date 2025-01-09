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

#pragma once  // Ensure the file is included only once during compilation

#include <frc/ADXRS450_Gyro.h>  // Include the ADXRS450 Gyro class from FRC
#include <frc/SPI.h>  // Include the SPI class from FRC
#include <frc/estimator/SwerveDrivePoseEstimator.h>  // Include the SwerveDrivePoseEstimator class from FRC
#include <frc/kinematics/SwerveDriveKinematics.h>  // Include the SwerveDriveKinematics class from FRC
#include <frc/simulation/ADXRS450_GyroSim.h>  // Include the ADXRS450 Gyro simulation class from FRC

#include "SwerveDriveSim.h"  // Include the SwerveDriveSim header file
#include "SwerveModule.h"  // Include the SwerveModule header file

// Define the SwerveDrive class
class SwerveDrive {
 public:
  // Constructor for the SwerveDrive class
  SwerveDrive();
  // Periodic function called periodically during runtime
  void Periodic();
  // Drive the robot with specified velocities
  void Drive(units::meters_per_second_t vx, units::meters_per_second_t vy,
             units::radians_per_second_t omega);
  // Set the chassis speeds
  void SetChassisSpeeds(const frc::ChassisSpeeds& targetChassisSpeeds,
                        bool openLoop, bool steerInPlace);
  // Set the module states
  void SetModuleStates(
      const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop,
      bool steerInPlace);
  // Stop the robot
  void Stop();
  // Reset the pose of the robot
  void ResetPose(const frc::Pose2d& pose, bool resetSimPose);
  // Get the pose of the robot
  frc::Pose2d GetPose() const;
  // Get the heading of the robot
  frc::Rotation2d GetHeading() const;
  // Get the yaw of the gyro
  frc::Rotation2d GetGyroYaw() const;
  // Get the chassis speeds
  frc::ChassisSpeeds GetChassisSpeeds() const;
  // Get the module states
  std::array<frc::SwerveModuleState, 4> GetModuleStates() const;
  // Get the module positions
  std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
  // Get the module poses
  std::array<frc::Pose2d, 4> GetModulePoses() const;
  // Log the current state of the swerve drive to the SmartDashboard
  void Log();
  // Periodic function called periodically during simulation
  void SimulationPeriodic();
  // Get the simulation pose of the robot
  frc::Pose2d GetSimPose() const;
  // Get the current draw of the robot
  units::ampere_t GetCurrentDraw() const;

 private:
  std::array<SwerveModule, 4> swerveMods{  // Array to store swerve modules
      SwerveModule{constants::Swerve::FL_CONSTANTS},  // Front left module
      SwerveModule{constants::Swerve::FR_CONSTANTS},  // Front right module
      SwerveModule{constants::Swerve::BL_CONSTANTS},  // Back left module
      SwerveModule{constants::Swerve::BR_CONSTANTS}};  // Back right module
  frc::SwerveDriveKinematics<4> kinematics{  // Swerve drive kinematics
      swerveMods[0].GetModuleConstants().centerOffset,
      swerveMods[1].GetModuleConstants().centerOffset,
      swerveMods[2].GetModuleConstants().centerOffset,
      swerveMods[3].GetModuleConstants().centerOffset};
  frc::ADXRS450_Gyro gyro{frc::SPI::Port::kOnboardCS0};  // Gyro sensor
  frc::SwerveDrivePoseEstimator<4> poseEstimator;  // Pose estimator
  frc::ChassisSpeeds targetChassisSpeeds{};  // Target chassis speeds

  frc::sim::ADXRS450_GyroSim gyroSim;  // Gyro simulation
  SwerveDriveSim swerveDriveSim;  // Swerve drive simulation
  units::ampere_t totalCurrentDraw{0};  // Total current draw
};