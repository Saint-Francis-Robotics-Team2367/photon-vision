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

#include "frc/Encoder.h" // Include the Encoder class from FRC
#include "frc/controller/PIDController.h" // Include the PIDController class from FRC
#include "frc/kinematics/SwerveModulePosition.h" // Include the SwerveModulePosition class from FRC
#include "frc/kinematics/SwerveModuleState.h" // Include the SwerveModuleState class from FRC
#include "frc/motorcontrol/PWMSparkMax.h" // Include the PWMSparkMax class from FRC
#include "frc/simulation/EncoderSim.h" // Include the EncoderSim class from FRC
#include "units/current.h" // Include the units library for current
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "subsystems/TalonFXMotor.h"  // Include the TalonFXMotor class
#include "subsystems/CAN_Coder.h"  // Include the CAN_Coder class

#include "Constants.h"  // Include the constants header file
#include "Konstants.h"

// Define the SwerveModule class
class SwerveModule {
 public:
  // Constructor for the SwerveModule class
  explicit SwerveModule(const constants::Swerve::ModuleConstants& consts);
  SwerveModule(int steerMotorID, int driveMotorID, int cancoderID); // New constructor

  // Periodic function called periodically during runtime
  void Periodic();
  // Set the desired state of the swerve module
  void SetDesiredState(frc::SwerveModuleState newState, bool shouldBeOpenLoop,
                       bool steerInPlace);
  // Get the absolute heading of the module
  frc::Rotation2d GetAbsoluteHeading() const;
  // Get the current state of the swerve module
  frc::SwerveModuleState GetState() const;
  // Get the current position of the swerve module
  frc::SwerveModulePosition GetPosition() const;
  // Get the current voltage applied to the drive motor
  units::volt_t GetDriveVoltage() const;
  // Get the current voltage applied to the steer motor
  units::volt_t GetSteerVoltage() const;
  // Get the simulated current for the drive motor
  units::ampere_t GetDriveCurrentSim() const;
  // Get the simulated current for the steer motor
  units::ampere_t GetSteerCurrentSim() const;
  // Get the module constants
  constants::Swerve::ModuleConstants GetModuleConstants() const;
  // Log the current state of the swerve module to the SmartDashboard
  void Log();
  // Update the simulation values for the swerve module
  void SimulationUpdate(units::meter_t driveEncoderDist,
                        units::meters_per_second_t driveEncoderRate,
                        units::ampere_t driveCurrent,
                        units::radian_t steerEncoderDist,
                        units::radians_per_second_t steerEncoderRate,
                        units::ampere_t steerCurrent);

 private:
  const constants::Swerve::ModuleConstants moduleConstants;  // Store the module constants

  // Old motor and encoder definitions (commented out to avoid redundancy)
  // frc::PWMSparkMax driveMotor;  // Drive motor
  // frc::Encoder driveEncoder;  // Drive encoder
  // frc::PWMSparkMax steerMotor;  // Steer motor
  // frc::Encoder steerEncoder;  // Steer encoder

  // New motor and encoder definitions
  std::unique_ptr<rev::spark::SparkMax> steerMotor;  // Steer motor
  TalonFXMotor driveMotor;  // Drive motor
  CAN_Coder steerEnc;  // Steer encoder
  frc::PIDController steerCTR;  // Steer PID controller

  frc::SwerveModuleState desiredState{};  // Desired state of the module
  bool openLoop{false};  // Flag for open loop control

  frc::PIDController drivePIDController{constants::Swerve::kDriveKP,
                                        constants::Swerve::kDriveKI,
                                        constants::Swerve::kDriveKD};  // PID controller for drive motor
  frc::PIDController steerPIDController{constants::Swerve::kSteerKP,
                                        constants::Swerve::kSteerKI,
                                        constants::Swerve::kSteerKD};  // PID controller for steer motor

  frc::sim::EncoderSim driveEncoderSim;  // Simulation for drive encoder
  units::ampere_t driveCurrentSim{0};  // Simulated current for drive motor
  frc::sim::EncoderSim steerEncoderSim;  // Simulation for steer encoder
  units::ampere_t steerCurrentSim{0};  // Simulated current for steer motor
};