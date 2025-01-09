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

#include "subsystems/SwerveDriveSim.h"  // Include the header file

#include <iostream>  // Include iostream for debugging

#include <frc/RobotController.h>  // Include FRC robot controller
#include <frc/system/Discretization.h>  // Include FRC system discretization

// Template function to get the sign of a value
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

// Constructor with motor feedforward and kinematics
SwerveDriveSim::SwerveDriveSim(
    const frc::SimpleMotorFeedforward<units::meters>& driveFF,
    const frc::DCMotor& driveMotor, double driveGearing,
    units::meter_t driveWheelRadius,
    const frc::SimpleMotorFeedforward<units::radians>& steerFF,
    const frc::DCMotor& steerMotor, double steerGearing,
    const frc::SwerveDriveKinematics<numModules>& kinematics)
    : SwerveDriveSim(
          frc::LinearSystem<2, 1, 2>{
              (Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
               -driveFF.GetKv().to<double>() / driveFF.GetKa().to<double>())
                  .finished(),
              Eigen::Matrix<double, 2, 1>{0.0,
                                          1.0 / driveFF.GetKa().to<double>()},
              (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
              Eigen::Matrix<double, 2, 1>{0.0, 0.0}},
          driveFF.GetKs(), driveMotor, driveGearing, driveWheelRadius,
          frc::LinearSystem<2, 1, 2>{
              (Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
               -steerFF.GetKv().to<double>() / steerFF.GetKa().to<double>())
                  .finished(),
              Eigen::Matrix<double, 2, 1>{0.0,
                                          1.0 / steerFF.GetKa().to<double>()},
              (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
              Eigen::Matrix<double, 2, 1>{0.0, 0.0}},
          steerFF.GetKs(), steerMotor, steerGearing, kinematics) {}

// Constructor with linear system and kinematics
SwerveDriveSim::SwerveDriveSim(
    const frc::LinearSystem<2, 1, 2>& drivePlant, units::volt_t driveKs,
    const frc::DCMotor& driveMotor, double driveGearing,
    units::meter_t driveWheelRadius,
    const frc::LinearSystem<2, 1, 2>& steerPlant, units::volt_t steerKs,
    const frc::DCMotor& steerMotor, double steerGearing,
    const frc::SwerveDriveKinematics<numModules>& kinematics)
    : drivePlant(drivePlant),  // Initialize drive plant
      driveKs(driveKs),  // Initialize drive system constant
      driveMotor(driveMotor),  // Initialize drive motor model
      driveGearing(driveGearing),  // Initialize drive gearing ratio
      driveWheelRadius(driveWheelRadius),  // Initialize drive wheel radius
      steerPlant(steerPlant),  // Initialize steer plant
      steerKs(steerKs),  // Initialize steer system constant
      steerMotor(steerMotor),  // Initialize steer motor model
      steerGearing(steerGearing),  // Initialize steer gearing ratio
      kinematics(kinematics) {}  // Initialize swerve drive kinematics

// Set drive inputs
void SwerveDriveSim::SetDriveInputs(
    const std::array<units::volt_t, numModules>& inputs) {
  units::volt_t battVoltage = frc::RobotController::GetBatteryVoltage();  // Get battery voltage
  for (int i = 0; i < driveInputs.size(); i++) {  // Iterate over drive inputs
    units::volt_t input = inputs[i];  // Get input voltage
    driveInputs[i] = std::clamp(input, -battVoltage, battVoltage);  // Clamp input to battery voltage
  }
}

// Set steer inputs
void SwerveDriveSim::SetSteerInputs(
    const std::array<units::volt_t, numModules>& inputs) {
  units::volt_t battVoltage = frc::RobotController::GetBatteryVoltage();  // Get battery voltage
  for (int i = 0; i < steerInputs.size(); i++) {  // Iterate over steer inputs
    units::volt_t input = inputs[i];  // Get input voltage
    steerInputs[i] = std::clamp(input, -battVoltage, battVoltage);  // Clamp input to battery voltage
  }
}

// Calculate the next state
Eigen::Matrix<double, 2, 1> SwerveDriveSim::CalculateX(
    const Eigen::Matrix<double, 2, 2>& discA,
    const Eigen::Matrix<double, 2, 1>& discB,
    const Eigen::Matrix<double, 2, 1>& x, units::volt_t input,
    units::volt_t kS) {
  auto Ax = discA * x;  // Calculate A*x
  double nextStateVel = Ax(1, 0);  // Get next state velocity
  double inputToStop = nextStateVel / -discB(1, 0);  // Calculate input to stop
  double ksSystemEffect =
      std::clamp(inputToStop, -kS.to<double>(), kS.to<double>());  // Clamp system effect

  nextStateVel += discB(1, 0) * ksSystemEffect;  // Update next state velocity
  inputToStop = nextStateVel / -discB(1, 0);  // Recalculate input to stop
  double signToStop = sgn(inputToStop);  // Get sign to stop
  double inputSign = sgn(input.to<double>());  // Get input sign
  double ksInputEffect = 0;  // Initialize input effect

  if (std::abs(ksSystemEffect) < kS.to<double>()) {  // Check if system effect is less than kS
    double absInput = std::abs(input.to<double>());  // Get absolute input
    ksInputEffect =
        -std::clamp(kS.to<double>() * inputSign, -absInput, absInput);  // Clamp input effect
  } else if ((input.to<double>() * signToStop) > (inputToStop * signToStop)) {  // Check if input is greater than input to stop
    double absInput = std::abs(input.to<double>() - inputToStop);  // Get absolute input difference
    ksInputEffect =
        -std::clamp(kS.to<double>() * inputSign, -absInput, absInput);  // Clamp input effect
  }

  auto sF = Eigen::Matrix<double, 1, 1>{input.to<double>() + ksSystemEffect +
                                        ksInputEffect};  // Calculate sF
  auto Bu = discB * sF;  // Calculate B*u
  auto retVal = Ax + Bu;  // Calculate next state
  return retVal;  // Return next state
}

// Update the simulation
void SwerveDriveSim::Update(units::second_t dt) {
  Eigen::Matrix<double, 2, 2> driveDiscA;  // Discretized A matrix for drive
  Eigen::Matrix<double, 2, 1> driveDiscB;  // Discretized B matrix for drive
  frc::DiscretizeAB<2, 1>(drivePlant.A(), drivePlant.B(), dt, &driveDiscA,
                          &driveDiscB);  // Discretize drive system

  Eigen::Matrix<double, 2, 2> steerDiscA;  // Discretized A matrix for steer
  Eigen::Matrix<double, 2, 1> steerDiscB;  // Discretized B matrix for steer
  frc::DiscretizeAB<2, 1>(steerPlant.A(), steerPlant.B(), dt, &steerDiscA,
                          &steerDiscB);  // Discretize steer system

  std::array<frc::SwerveModulePosition, 4> moduleDeltas;  // Array to store module deltas

  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    double prevDriveStatePos = driveStates[i](0, 0);  // Get previous drive state position
    driveStates[i] = CalculateX(driveDiscA, driveDiscB, driveStates[i],
                                driveInputs[i], driveKs);  // Calculate next drive state
    double currentDriveStatePos = driveStates[i](0, 0);  // Get current drive state position
    steerStates[i] = CalculateX(steerDiscA, steerDiscB, steerStates[i],
                                steerInputs[i], steerKs);  // Calculate next steer state
    double currentSteerStatePos = steerStates[i](0, 0);  // Get current steer state position
    moduleDeltas[i] = frc::SwerveModulePosition{
        units::meter_t{currentDriveStatePos - prevDriveStatePos},
        frc::Rotation2d{units::radian_t{currentSteerStatePos}}};  // Calculate module delta
  }

  frc::Twist2d twist = kinematics.ToTwist2d(moduleDeltas);  // Calculate twist
  pose = pose.Exp(twist);  // Update pose
  omega = twist.dtheta / dt;  // Update angular velocity
}

// Reset the simulation
void SwerveDriveSim::Reset(const frc::Pose2d& pose, bool preserveMotion) {
  this->pose = pose;  // Set pose
  if (!preserveMotion) {  // Check if motion should be preserved
    for (int i = 0; i < numModules; i++) {  // Iterate over modules
      driveStates[i] = Eigen::Matrix<double, 2, 1>{0, 0};  // Reset drive state
      steerStates[i] = Eigen::Matrix<double, 2, 1>{0, 0};  // Reset steer state
    }
    omega = 0_rad_per_s;  // Reset angular velocity
  }
}

// Reset the simulation with module states
void SwerveDriveSim::Reset(const frc::Pose2d& pose,
                           const std::array<Eigen::Matrix<double, 2, 1>,
                                            numModules>& moduleDriveStates,
                           const std::array<Eigen::Matrix<double, 2, 1>,
                                            numModules>& moduleSteerStates) {
  this->pose = pose;  // Set pose
  driveStates = moduleDriveStates;  // Set drive states
  steerStates = moduleSteerStates;  // Set steer states
  omega = kinematics.ToChassisSpeeds(GetModuleStates()).omega;  // Update angular velocity
}

// Get the robot pose
frc::Pose2d SwerveDriveSim::GetPose() const { return pose; }

// Get the module positions
std::array<frc::SwerveModulePosition, numModules>
SwerveDriveSim::GetModulePositions() const {
  std::array<frc::SwerveModulePosition, numModules> positions;  // Array to store positions
  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    positions[i] = frc::SwerveModulePosition{
        units::meter_t{driveStates[i](0, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};  // Get module position
  }
  return positions;
}

// Get noisy module positions
std::array<frc::SwerveModulePosition, numModules>
SwerveDriveSim::GetNoisyModulePositions(units::meter_t driveStdDev,
                                        units::radian_t steerStdDev) {
  std::array<frc::SwerveModulePosition, numModules> positions;  // Array to store positions
  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    positions[i] = frc::SwerveModulePosition{
        units::meter_t{driveStates[i](0, 0)} +
            randDist(generator) * driveStdDev,  // Add noise to drive position
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)} +
                        randDist(generator) * steerStdDev}};  // Add noise to steer position
  }
  return positions;
}

// Get the module states
std::array<frc::SwerveModuleState, numModules>
SwerveDriveSim::GetModuleStates() {
  std::array<frc::SwerveModuleState, numModules> states;  // Array to store states
  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    states[i] = frc::SwerveModuleState{
        units::meters_per_second_t{driveStates[i](1, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};  // Get module state
  }
  return states;
}

// Get the drive states
std::array<Eigen::Matrix<double, 2, 1>, numModules>
SwerveDriveSim::GetDriveStates() const {
  return driveStates;  // Return drive states
}

// Get the steer states
std::array<Eigen::Matrix<double, 2, 1>, numModules>
SwerveDriveSim::GetSteerStates() const {
  return steerStates;  // Return steer states
}

// Get the angular velocity
units::radians_per_second_t SwerveDriveSim::GetOmega() const { return omega; }

// Get the current draw
units::ampere_t SwerveDriveSim::GetCurrentDraw(
    const frc::DCMotor& motor, units::radians_per_second_t velocity,
    units::volt_t inputVolts, units::volt_t batteryVolts) const {
  units::volt_t effVolts = inputVolts - velocity / motor.Kv;  // Calculate effective voltage
  if (inputVolts >= 0_V) {  // Check if input voltage is positive
    effVolts = std::clamp(effVolts, 0_V, inputVolts);  // Clamp effective voltage
  } else {
    effVolts = std::clamp(effVolts, inputVolts, 0_V);  // Clamp effective voltage
  }
  auto retVal = (inputVolts / batteryVolts) * (effVolts / motor.R);  // Calculate current draw
  return retVal;  // Return current draw
}

// Get the drive current draw
std::array<units::ampere_t, numModules> SwerveDriveSim::GetDriveCurrentDraw()
    const {
  std::array<units::ampere_t, numModules> currents;  // Array to store currents
  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    units::radians_per_second_t speed =
        units::radians_per_second_t{driveStates[i](1, 0)} * driveGearing /
        driveWheelRadius.to<double>();  // Calculate speed
    currents[i] = GetCurrentDraw(driveMotor, speed, driveInputs[i],
                                 frc::RobotController::GetBatteryVoltage());  // Get current draw
  }
  return currents;  // Return currents
}

// Get the steer current draw
std::array<units::ampere_t, numModules> SwerveDriveSim::GetSteerCurrentDraw()
    const {
  std::array<units::ampere_t, numModules> currents;  // Array to store currents
  for (int i = 0; i < numModules; i++) {  // Iterate over modules
    units::radians_per_second_t speed =
        units::radians_per_second_t{steerStates[i](1, 0) * steerGearing};  // Calculate speed
    // TODO: If uncommented we get huge current values.. Not sure how to fix
    // atm. :(
    currents[i] = 20_A;  // Set current to 20A
    // currents[i] = GetCurrentDraw(steerMotor, speed, steerInputs[i],
    // frc::RobotController::GetBatteryVoltage());
  }
  return currents;  // Return currents
}

// Get the total current draw
units::ampere_t SwerveDriveSim::GetTotalCurrentDraw() const {
  units::ampere_t total{0};  // Initialize total current
  for (const auto& val : GetDriveCurrentDraw()) {  // Iterate over drive currents
    total += val;  // Add drive current to total
  }
  for (const auto& val : GetSteerCurrentDraw()) {  // Iterate over steer currents
    total += val;  // Add steer current to total
  }
  return total;  // Return total current
}