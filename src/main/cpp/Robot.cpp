// FRC Team 716 Who Me Robot code 2022
// not reccomended for general use
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoDriveForward, kAutoDriveForward);
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //compressor.Start();
}

void Robot::RobotPeriodic() {}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  if (m_autoSelected==kAutoDriveForward) {autoMode=AutoDriveForward;}
  else if (m_autoSelected==kAutoShootFromClose) {autoMode=AutoShootClose;}
  else autoMode=AutoNothing;

   leftDriveEncoder.Reset();
  rightDriveEncoder.Reset();
}

void Robot::AutonomousPeriodic() {
  switch(autoMode)
  {
    case AutoDriveForward:
     if (autoactive) {
        if (DistanceDrive(1,AUTODIST, true) == DONE) autoactive = false;
     }
     else drive.TankDrive(0,0,false);
     break;
    case AutoShootClose:
     // need a timer
     // reset timer in autonomous INIT
    break;
    default:
      drive.TankDrive(0,0,false); 
  }
}

void Robot::TeleopInit() {
  leftDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
}

void Robot::TeleopPeriodic() {
  if (rightDriveStick.GetTrigger()) HoldTheLine();
  else if (leftDriveStick.GetTrigger()) StraightDrive();
  else {
    drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY() ));
    sdfr = false;}
  if (gamepad.GetBackButtonPressed()) Abort();
  //Analog Controls
  // check pickup wheel control
  if (gamepad.GetRightTriggerAxis() > 0.7){ // check deadzone
    ballIntake.Set(-1);}
  else if (gamepad.GetLeftTriggerAxis() > 0.7){
    ballIntake.Set(1);}
  else {ballIntake.Set(0);}
  
  if (fabs(gamepad.GetLeftY()) > 0.1 ){ // check deadzone
    liftMotor.Set(gamepad.GetLeftY());}
  else {liftMotor.Set(0);} 
  if (fabs(gamepad.GetRightY()) > 0.1 ){ // check deadzone
    climber.Set(gamepad.GetRightY());}
  else {climber.Set(0);} 
  if (gamepad.GetPOV() != -1) Lock();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(){
  if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();

    sdfr = true;
  }

  double throttle = (leftDriveStick.GetY());
  double difference = (rightDriveEncoder.GetDistance()) + (leftDriveEncoder.GetDistance());
  drive.TankDrive(-1*(throttle - (difference * 0.1)), (throttle + (difference * 0.1)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
  std::cout << "Love isn't always on time" << std::endl; // No I am not ashamed of this TOTO reference
    if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
    sdfr = true;
  }
  drive.TankDrive((-0.025 * leftDriveEncoder.GetDistance()),(0.025 * rightDriveEncoder.GetDistance()), false);
}

void Robot::Abort(){
  ballIntake.StopMotor();
  liftMotor.StopMotor();
  climber.StopMotor();
 // Pneumatic1.Set(frc::DoubleSolenoid::Value::kReverse);
 // Pneumatic2.Set(frc::DoubleSolenoid::Value::kReverse);
  //Pneumatic3.Set(frc::DoubleSolenoid::Value::kReverse);
  //Pneumatic4.Set(frc::DoubleSolenoid::Value::kReverse);
  auxSpedCtrlr4DefState = 0;
  auxSpedCtrlr5DefState = 0;
  auxSpedCtrlr6DefState = 0;
}

void Robot::Lock(){
  if (gamepad.GetLeftBumper()) auxSpedCtrlr4DefState = AUXSPDCTL_SPD;
  if (gamepad.GetRightBumper()) auxSpedCtrlr5DefState = AUXSPDCTL_SPD;
  if (rightDriveStick.GetTop()) auxSpedCtrlr6DefState = AUXSPDCTL_SPD;
}

int Robot::DistanceDrive (float speed, float distance, bool brake)
{
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
	static float autoStartSpeed;
  static float direction;
	static double lastDistance, speedUpDistance, slowDownDistance;
  static int sameCounter;
  static bool brakingFlag;
  static units::time::second_t brakeStartTime;

	float newSpeed;
	double curDistance;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    brakingFlag = false;
    FirstCallFlag = false;
    if (speed < 0) {
      direction = -1;
    } else {
      direction = 1;
    }
    autoStartSpeed = direction * AUTOSTARTSPEED;
    if (distance < (DRIVERAMPUPDISTANCE * 2)) {
	    speedUpDistance = distance / 2;
	    slowDownDistance = speedUpDistance;
    } else {
	    speedUpDistance = DRIVERAMPUPDISTANCE;
     	slowDownDistance = distance - DRIVERAMPUPDISTANCE;
    }
	  frc::SmartDashboard::PutNumber(  "DistanceDrive Distance", distance);
  	lastDistance = 0;
    sameCounter = 0;
    leftDriveEncoder.Reset();
  }

 	if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < 0.2_s) {
    	drive.TankDrive(-0.2 * direction *FORWARD, -0.2 * direction * FORWARD);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}
  
	curDistance = abs(leftDriveEncoder.GetDistance());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((speed - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}

	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
	curDistance = abs(leftDriveEncoder.GetDistance());
  if (curDistance < distance) {
    return NOTDONEYET;
  } else {
    if (brake) {
      brakingFlag = true;
      brakeStartTime = AutoTimer.Get();
      return NOTDONEYET;
    } else {
      FirstCallFlag = true;
      drive.TankDrive(0, 0);
      return DONE;
    }
  }
  
  // should never get here
  drive.TankDrive(0, 0);
  FirstCallFlag = true;
  return DONE;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
