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

#include "subsystems/SwerveModule.h" // Include the header file for the SwerveModule class

#include <iostream> // Include the iostream library for input/output operations
#include <string> // Include the string library for string operations

#include <frc/MathUtil.h> // Include the MathUtil library from FRC
#include <frc/RobotController.h> // Include the RobotController library from FRC
#include <frc/smartdashboard/SmartDashboard.h> // Include the SmartDashboard library from FRC

// Constructor for the SwerveModule class
SwerveModule::SwerveModule(const constants::Swerve::ModuleConstants& consts)
    : moduleConstants(consts), // Store the module constants
      driveMotor(frc::PWMSparkMax{moduleConstants.driveMotorId}), // Initialize drive motor
      driveEncoder(frc::Encoder{moduleConstants.driveEncoderA, moduleConstants.driveEncoderB}), // Initialize drive encoder
      steerMotor(frc::PWMSparkMax{moduleConstants.steerMotorId}), // Initialize steer motor
      steerEncoder(frc::Encoder{moduleConstants.steerEncoderA, moduleConstants.steerEncoderB}), // Initialize steer encoder
      driveEncoderSim(driveEncoder), // Initialize drive encoder simulation
      steerEncoderSim(steerEncoder) { // Initialize steer encoder simulation

  // Set the distance per pulse for the drive encoder
  driveEncoder.SetDistancePerPulse(constants::Swerve::kDriveDistPerPulse.to<double>());
  
  // Set the distance per pulse for the steer encoder
  steerEncoder.SetDistancePerPulse(constants::Swerve::kSteerRadPerPulse.to<double>());
  
  // Enable continuous input for the steer PID controller, allowing it to wrap around from -π to π
  steerPIDController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
}

// Periodic function called periodically during runtime
void SwerveModule::Periodic() {
  // Calculate the steering PID output voltage
  units::volt_t steerPID = units::volt_t{
      steerPIDController.Calculate(GetAbsoluteHeading().Radians().to<double>(),
                                   desiredState.angle.Radians().to<double>())};
  // Set the steering motor voltage based on the PID output
  steerMotor.SetVoltage(steerPID);

  // Calculate the feedforward voltage for the drive motor
  units::volt_t driveFF =
      constants::Swerve::kDriveFF.Calculate(desiredState.speed);
  units::volt_t drivePID{0};
  // If not in open loop mode, calculate the drive PID output voltage
  if (!openLoop) {
    drivePID = units::volt_t{drivePIDController.Calculate(
        driveEncoder.GetRate(), desiredState.speed.to<double>())};
  }
  // Set the drive motor voltage based on the sum of feedforward and PID output
  driveMotor.SetVoltage(driveFF + drivePID);
}

// Set the desired state of the swerve module
void SwerveModule::SetDesiredState(frc::SwerveModuleState newState,
                                   bool shouldBeOpenLoop, bool steerInPlace) {
  // Get the current rotation of the module
  frc::Rotation2d currentRotation = GetAbsoluteHeading();
  // Optimize the new state based on the current rotation
  newState.Optimize(currentRotation);
  // Set the desired state to the new optimized state
  desiredState = newState;
}

// Get the absolute heading of the module
frc::Rotation2d SwerveModule::GetAbsoluteHeading() const {
  // Return the absolute heading of the module based on the steer encoder distance
  return frc::Rotation2d{units::radian_t{steerEncoder.GetDistance()}};
}

// Get the current state of the swerve module
frc::SwerveModuleState SwerveModule::GetState() const {
  // Return the current state of the module, including speed and heading
  return frc::SwerveModuleState{driveEncoder.GetRate() * 1_mps,
                                GetAbsoluteHeading()};
}

// Get the current position of the swerve module
frc::SwerveModulePosition SwerveModule::GetPosition() const {
  // Return the current position of the module, including distance and heading
  return frc::SwerveModulePosition{driveEncoder.GetDistance() * 1_m,
                                   GetAbsoluteHeading()};
}

// Get the current voltage applied to the drive motor
units::volt_t SwerveModule::GetDriveVoltage() const {
  // Return the current voltage applied to the drive motor
  return driveMotor.Get() * frc::RobotController::GetBatteryVoltage();
}

// Get the current voltage applied to the steer motor
units::volt_t SwerveModule::GetSteerVoltage() const {
  // Return the current voltage applied to the steer motor
  return steerMotor.Get() * frc::RobotController::GetBatteryVoltage();
}

// Get the simulated current for the drive motor
units::ampere_t SwerveModule::GetDriveCurrentSim() const {
  // Return the simulated current for the drive motor
  return driveCurrentSim;
}

// Get the simulated current for the steer motor
units::ampere_t SwerveModule::GetSteerCurrentSim() const {
  // Return the simulated current for the steer motor
  return steerCurrentSim;
}

// Get the module constants
constants::Swerve::ModuleConstants SwerveModule::GetModuleConstants() const {
  // Return the module constants
  return moduleConstants;
}

// Log the current state of the swerve module to the SmartDashboard
void SwerveModule::Log() {
  // Get the current state of the swerve module, including speed and heading
  frc::SwerveModuleState state = GetState();

  // Create a string for the SmartDashboard table name, including the module number
  std::string table = "Module " + std::to_string(moduleConstants.moduleNum) + "/";

  // Log the current steering angle in degrees to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Steer Degrees",
      frc::AngleModulus(state.angle.Radians()) // Normalize the angle to be within -π to π
          .convert<units::degrees>() // Convert the angle from radians to degrees
          .to<double>()); // Convert the angle to a double for logging

  // Log the target steering angle in degrees to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Steer Target Degrees",
      units::radian_t{steerPIDController.GetSetpoint()} // Get the target angle from the PID controller
          .convert<units::degrees>() // Convert the angle from radians to degrees
          .to<double>()); // Convert the angle to a double for logging

  // Log the current drive velocity in feet per second to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Drive Velocity Feet",
      state.speed.convert<units::feet_per_second>() // Convert the speed to feet per second
          .to<double>()); // Convert the speed to a double for logging

  // Log the target drive velocity in feet per second to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Drive Velocity Target Feet",
      desiredState.speed.convert<units::feet_per_second>() // Convert the target speed to feet per second
          .to<double>()); // Convert the target speed to a double for logging

  // Log the current simulated drive motor current to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Drive Current",
      driveCurrentSim.to<double>()); // Convert the current to a double for logging

  // Log the current simulated steer motor current to the SmartDashboard
  frc::SmartDashboard::PutNumber(
      table + "Steer Current",
      steerCurrentSim.to<double>()); // Convert the current to a double for logging
}

// Update the simulation values for the swerve module
void SwerveModule::SimulationUpdate(
    units::meter_t driveEncoderDist,
    units::meters_per_second_t driveEncoderRate, units::ampere_t driveCurrent,
    units::radian_t steerEncoderDist,
    units::radians_per_second_t steerEncoderRate,
    units::ampere_t steerCurrent) {
  // Set the simulated distance for the drive encoder
  driveEncoderSim.SetDistance(driveEncoderDist.to<double>());
  // Set the simulated rate for the drive encoder
  driveEncoderSim.SetRate(driveEncoderRate.to<double>());
  // Set the simulated current for the drive motor
  driveCurrentSim = driveCurrent;
  // Set the simulated distance for the steer encoder
  steerEncoderSim.SetDistance(steerEncoderDist.to<double>());
  // Set the simulated rate for the steer encoder
  steerEncoderSim.SetRate(steerEncoderRate.to<double>());
  // Set the simulated current for the steer motor
  steerCurrentSim = steerCurrent;
}