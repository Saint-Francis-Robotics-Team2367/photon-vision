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

#include <random>  // Include the random library for generating random numbers

#include <frc/controller/SimpleMotorFeedforward.h>  // Include FRC motor feedforward controller
#include <frc/kinematics/SwerveDriveKinematics.h>  // Include FRC swerve drive kinematics
#include <frc/system/LinearSystem.h>  // Include FRC linear system
#include <frc/system/plant/DCMotor.h>  // Include FRC DC motor model
#include <units/voltage.h>  // Include units library for voltage

static constexpr int numModules{4};  // Define the number of swerve modules

class SwerveDriveSim {
 public:
  // Constructor with motor feedforward and kinematics
  SwerveDriveSim(const frc::SimpleMotorFeedforward<units::meters>& driveFF,
                 const frc::DCMotor& driveMotor, double driveGearing,
                 units::meter_t driveWheelRadius,
                 const frc::SimpleMotorFeedforward<units::radians>& steerFF,
                 const frc::DCMotor& steerMotor, double steerGearing,
                 const frc::SwerveDriveKinematics<numModules>& kinematics);
  // Constructor with linear system and kinematics
  SwerveDriveSim(const frc::LinearSystem<2, 1, 2>& drivePlant,
                 units::volt_t driveKs, const frc::DCMotor& driveMotor,
                 double driveGearing, units::meter_t driveWheelRadius,
                 const frc::LinearSystem<2, 1, 2>& steerPlant,
                 units::volt_t steerKs, const frc::DCMotor& steerMotor,
                 double steerGearing,
                 const frc::SwerveDriveKinematics<numModules>& kinematics);
  // Set drive inputs
  void SetDriveInputs(const std::array<units::volt_t, numModules>& inputs);
  // Set steer inputs
  void SetSteerInputs(const std::array<units::volt_t, numModules>& inputs);
  // Calculate the next state
  static Eigen::Matrix<double, 2, 1> CalculateX(
      const Eigen::Matrix<double, 2, 2>& discA,
      const Eigen::Matrix<double, 2, 1>& discB,
      const Eigen::Matrix<double, 2, 1>& x, units::volt_t input,
      units::volt_t kS);
  // Update the simulation
  void Update(units::second_t dt);
  // Reset the simulation
  void Reset(const frc::Pose2d& pose, bool preserveMotion);
  // Reset the simulation with module states
  void Reset(const frc::Pose2d& pose,
             const std::array<Eigen::Matrix<double, 2, 1>, numModules>&
                 moduleDriveStates,
             const std::array<Eigen::Matrix<double, 2, 1>, numModules>&
                 moduleSteerStates);
  // Get the robot pose
  frc::Pose2d GetPose() const;
  // Get the module positions
  std::array<frc::SwerveModulePosition, numModules> GetModulePositions() const;
  // Get noisy module positions
  std::array<frc::SwerveModulePosition, numModules> GetNoisyModulePositions(
      units::meter_t driveStdDev, units::radian_t steerStdDev);
  // Get the module states
  std::array<frc::SwerveModuleState, numModules> GetModuleStates();
  // Get the drive states
  std::array<Eigen::Matrix<double, 2, 1>, numModules> GetDriveStates() const;
  // Get the steer states
  std::array<Eigen::Matrix<double, 2, 1>, numModules> GetSteerStates() const;
  // Get the angular velocity
  units::radians_per_second_t GetOmega() const;
  // Get the current draw
  units::ampere_t GetCurrentDraw(const frc::DCMotor& motor,
                                 units::radians_per_second_t velocity,
                                 units::volt_t inputVolts,
                                 units::volt_t batteryVolts) const;
  // Get the drive current draw
  std::array<units::ampere_t, numModules> GetDriveCurrentDraw() const;
  // Get the steer current draw
  std::array<units::ampere_t, numModules> GetSteerCurrentDraw() const;
  // Get the total current draw
  units::ampere_t GetTotalCurrentDraw() const;

 private:
  std::random_device rd{};  // Random device for seeding the generator
  std::mt19937 generator{rd()};  // Mersenne Twister random number generator
  std::normal_distribution<double> randDist{0.0, 1.0};  // Normal distribution for noise
  const frc::LinearSystem<2, 1, 2> drivePlant;  // Linear system for drive
  const units::volt_t driveKs;  // Drive system constant
  const frc::DCMotor driveMotor;  // Drive motor model
  const double driveGearing;  // Drive gearing ratio
  const units::meter_t driveWheelRadius;  // Drive wheel radius
  const frc::LinearSystem<2, 1, 2> steerPlant;  // Linear system for steer
  const units::volt_t steerKs;  // Steer system constant
  const frc::DCMotor steerMotor;  // Steer motor model
  const double steerGearing;  // Steer gearing ratio
  const frc::SwerveDriveKinematics<numModules> kinematics;  // Swerve drive kinematics
  std::array<units::volt_t, numModules> driveInputs{};  // Drive inputs
  std::array<Eigen::Matrix<double, 2, 1>, numModules> driveStates{};  // Drive states
  std::array<units::volt_t, numModules> steerInputs{};  // Steer inputs
  std::array<Eigen::Matrix<double, 2, 1>, numModules> steerStates{};  // Steer states
  frc::Pose2d pose{frc::Pose2d{}};  // Robot pose
  units::radians_per_second_t omega{0};  // Angular velocity
};