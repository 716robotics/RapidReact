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
  m_chooser.AddOption(kAutoPickup, kAutoPickup);
  m_chooser.AddOption(kAutoShootFromClose, kAutoShootFromClose);
  m_chooser.AddOption(kAutoDoNothing, kAutoDoNothing);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  leftDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
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
  AutoTimer.Reset();
  AutoTimer.Start();
}

void Robot::AutonomousPeriodic() {
  switch(autoMode)
  {
    case AutoDriveForward:
     if (autoactive) {
        if (DistanceDrive(.7,AUTODIST, true) == DONE) autoactive = false;
     }
     else drive.TankDrive(0,0,false);
     break;
    case AutoShootClose:
     switch(AutoStage){
       case 0:
       //shooting the ball
       ballIntake.Set(-1);
       if ((float)AutoTimer.Get() > 2) {
         ballIntake.Set(0);
         AutoStage = 1;
       }
      break;
      case 1:
      //Turn
      AutoStage = 2;
      break;
      case 2:
       if (DistanceDrive(-1, 60, true) == DONE) {
         drive.TankDrive(0,0,false);
         autoMode = AutoNothing;
          } 

     }
    break;
    case AutoPickup: 
    break;
    case AutoNothing:
    drive.TankDrive(0,0,false); 
    break;
    default:
      drive.TankDrive(0,0,false); 
  }
}

void Robot::TeleopInit() {
  sdfr = false;
  climbLocker.Set(1);
}

void Robot::TeleopPeriodic() {
  if (rightDriveStick.GetTrigger()) HoldTheLine();
  else if (leftDriveStick.GetTrigger()) StraightDrive(leftDriveStick.GetY());
  else {
    drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY()));
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
    climber.Set(-1*gamepad.GetRightY());}
  else {climber.Set(0);}
  if (gamepad.GetYButton()){climbLocker.Set(1);}
  if (gamepad.GetAButton()){climbLocker.Set(.5);}
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::StraightDrive(float throttle){
  if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
    sdfr = true;
  }
  double difference = (rightDriveEncoder.GetDistance()) + (leftDriveEncoder.GetDistance());
  drive.TankDrive(-1*(throttle - (difference * 0.15)), (throttle + (difference * 0.15)), false);
  }

//Should keep the robot from moving, never tested it
void Robot::HoldTheLine(){
    if (!sdfr){
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
    std::cout << "Encoders Reset!" << std::endl;
    sdfr = true;
  }
  drive.TankDrive((0.05 * leftDriveEncoder.GetDistance()),(0.05 * rightDriveEncoder.GetDistance()), false);
}

void Robot::Abort(){
  ballIntake.StopMotor();
  liftMotor.StopMotor();
  climber.StopMotor();
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
      StraightDrive(-0.2);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}
  
	curDistance = (abs(leftDriveEncoder.GetDistance())+abs(rightDriveEncoder.GetDistance()))/2;
	  frc::SmartDashboard::PutNumber(  "DistanceDrive curDistance", curDistance);

	if (curDistance == lastDistance) {
	  frc::SmartDashboard::PutNumber(  "DistanceDrive sameCounter", sameCounter);
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = AUTOSTARTSPEED + ((speed - AUTOSTARTSPEED) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}
	  frc::SmartDashboard::PutNumber(  "DistanceDrive newSpeed", newSpeed);

	/*drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
  drive.TankDrive(-1*newSpeed*AUTOFORWARD,newSpeed*AUTOFORWARD);*/
  StraightDrive(-1*newSpeed);
	curDistance = (abs(leftDriveEncoder.GetDistance())+abs(rightDriveEncoder.GetDistance()))/2;
 std::cout << "Curdist: " <<curDistance<<" target: "<<distance<<" speed: "<<newSpeed<<std::endl;
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
