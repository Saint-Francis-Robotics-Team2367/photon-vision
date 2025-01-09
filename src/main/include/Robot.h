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

#include <photon/PhotonCamera.h>  // Include the PhotonCamera library

#include <frc/TimedRobot.h>  // Include the TimedRobot class from FRC
#include <frc/XboxController.h>  // Include the XboxController class from FRC

#include "Constants.h"  // Include the constants header file
#include "VisionSim.h"  // Include the VisionSim header file
#include "subsystems/SwerveDrive.h"  // Include the SwerveDrive header file

// Define the Robot class, inheriting from frc::TimedRobot
class Robot : public frc::TimedRobot {
 public:
  // Override the RobotInit function
  void RobotInit() override;
  // Override the RobotPeriodic function
  void RobotPeriodic() override;
  // Override the DisabledInit function
  void DisabledInit() override;
  // Override the DisabledPeriodic function
  void DisabledPeriodic() override;
  // Override the DisabledExit function
  void DisabledExit() override;
  // Override the AutonomousInit function
  void AutonomousInit() override;
  // Override the AutonomousPeriodic function
  void AutonomousPeriodic() override;
  // Override the AutonomousExit function
  void AutonomousExit() override;
  // Override the TeleopInit function
  void TeleopInit() override;
  // Override the TeleopPeriodic function
  void TeleopPeriodic() override;
  // Override the TeleopExit function
  void TeleopExit() override;
  // Override the TestInit function
  void TestInit() override;
  // Override the TestPeriodic function
  void TestPeriodic() override;
  // Override the TestExit function
  void TestExit() override;
  // Override the SimulationPeriodic function
  void SimulationPeriodic() override;

 private:
  photon::PhotonCamera camera{constants::Vision::kCameraName};  // Initialize the PhotonCamera
  SwerveDrive drivetrain{};  // Initialize the SwerveDrive
  VisionSim vision{&camera};  // Initialize the VisionSim with the camera
  frc::XboxController controller{0};  // Initialize the XboxController on port 0
  static constexpr double VISION_TURN_kP = 0.01;  // Define the vision turn proportional constant
};