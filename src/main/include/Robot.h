// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once
#include <tunables.h>
#include <string>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/Relay.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{0};
  frc::Joystick rightDriveStick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  frc::VictorSP lDrive0{0};
  frc::VictorSP lDrive1{1};
  frc::VictorSP rDrive0{2};
  frc::VictorSP rDrive1{3};
  frc::SpeedControllerGroup lDrive{lDrive0, lDrive1};
  frc::SpeedControllerGroup rDrive{rDrive0, rDrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};
  //Effectors
  frc::VictorSP ballIntake{4};
  frc::VictorSP liftMotor{5};
  frc::VictorSP climber{6};
  frc::Servo climbLocker{7};
  //Sensors
	frc::Encoder leftDriveEncoder{0,1,false,frc::Encoder::k4X};
	frc::Encoder rightDriveEncoder{2,3,false,frc::Encoder::k4X};
  //Global Vars
  frc::Timer AutoTimer;
  bool sdfr = false;
  bool autoactive = true;
  

  enum autoModeTypes {AutoDriveForward, AutoShootClose, AutoNothing, AutoPickup, Auto2Ball} autoMode;
  int AutoStage = 0;
  int RC;
  enum robotSideTypes {leftSide, rightSide};


 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void StraightDrive(float);
  void HoldTheLine();
  void Abort();
  int DistanceDrive(float,float,bool);
  int DistanceTurn(enum robotSideTypes,float,bool);
  int DistanceRotate (float targetDistance, bool brake);
  float  m_autoDistance;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoDriveForward = "Drive Forward";
  const std::string kAutoShootFromClose = "Shoot One From Target";
  const std::string kAutoPickup = "Drive Then Pickup";
  const std::string kAuto2Ball = "2 Ball Auto";
  const std::string kAutoDoNothing = "Do Nothing";
  std::string m_autoSelected;
};
