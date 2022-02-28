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
  else if (m_autoSelected==kAutoPickup) {autoMode=AutoPickup;}
  else autoMode=AutoNothing;

  m_autoDistance = frc::SmartDashboard::GetNumber("Auto Distance", 100);

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
        // if (DistanceDrive(.7,AUTODIST, true) == DONE) autoactive = false;
        if (DistanceDrive(.7, m_autoDistance, true) == DONE) autoactive = false;
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
       if (DistanceTurn(rightSide, -30, true) == DONE) {
         drive.TankDrive(0,0,false);
         AutoStage = 2;
       }
      break;
      case 2:
       if (DistanceDrive(-.7, 90, true) == DONE) {
         drive.TankDrive(0,0,false);
         autoMode = AutoNothing;
       }

     }
    break;

    case AutoPickup: 
    switch (AutoStage){
    case 0:
    //lower pickup
     liftMotor.Set(-.7);
    if ((float)AutoTimer.Get() > 2) {
         liftMotor.Set(0);
         AutoStage = 1;    
         ballIntake.Set(1);

       }
    break;
    case 1:
    //drive forward 
       if (DistanceDrive(.7, 90, true) == DONE) {
         drive.TankDrive(0,0,false);
         AutoStage = 2;
         AutoTimer.Reset();
         ballIntake.Set(0);
       }
    break;
    case 2:
    //shut off the pickup
    liftMotor.Set(1);
    if ((float)AutoTimer.Get() > 2) {
         liftMotor.Set(0);
         autoMode = AutoNothing;
       }
    break;
    default:
    break;
    }
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
  
  if (fabs(gamepad.GetLeftY()) > 0.4 ){ // check deadzone
    liftMotor.Set(gamepad.GetLeftY());}
  else {liftMotor.Set(0);} 
  if (fabs(gamepad.GetRightY()) > 0.4 ){ // check deadzone
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

//
// DistanceDrive: Drive straight for a distance
// Input parms
//    targetSpeed: Speed provided to speed controllers.
//                 minimum value is AUTOSTARTSPEED and max value is 1
//                 specify a negative value to drive backwards
//    distance:    Distance to drive, in inches.
//                 Always a positive value, even if speed is negative
//    brake:       Specify TRUE to apply a brief bit of power in reverse
//                 after reaching target distance to avoid going too far
//
// Robot will start driving at a speed equal to AUTOSTARTSPEED and increase speed
// proportionally until it hits the target speed at a distance of DRIVERAMPUPDISTANCE.
// The drive at the specified speed until it gets within DRIVERAMPUPDISTANCE of the
// target distance at which point it slows down to AUTOSTARTSPEED by the end of the
// drive, then stops. When the brake parm is TRUE, it drives at low speed in the opposite
// direction for a fraction of a second to try to stop without overrunning the target
// distance.
//
int Robot::DistanceDrive (float targetSpeed, float distance, bool brake)
{

  static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static float speedMult;           // -1 or 1 speed multiplier based on direction and AUTOFORWARD value
  static float speed;
  static double lastDistance, speedUpDistance, slowDownDistance;

  static int sameCounter;

  static bool brakingFlag;
  static units::time::second_t brakeStartTime;

  float newSpeed;
  double curDistance;

  //
  // Set up initial values on first call to DistanceDrive only
  //
  if (FirstCallFlag) {
    brakingFlag = false;
    FirstCallFlag = false;


    //
    // If speed is negative, it means drive backwards.
    //
    if (targetSpeed < 0) {
       speed = fabs(targetSpeed);
       speedMult = -1 * AUTOFORWARD;
    } else {
       speed = targetSpeed;
       speedMult = 1 * AUTOFORWARD;
    }

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
    rightDriveEncoder.Reset();
  }

  //
  // See if we're done driving and just braking
  //
  if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < 0.2_s) {
      // Not done braking yet, continue to drive slowly in opposite direction
      StraightDrive(speedMult * (-0.2));
      return NOTDONEYET;
    } else {
      // Done braking yet, stop
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
  }

  //
  // Determine distance travelled so far
  //
  curDistance = (fabs(leftDriveEncoder.GetDistance())+fabs(rightDriveEncoder.GetDistance()))/2;
  frc::SmartDashboard::PutNumber(  "DistanceDrive curDistance", curDistance);

  //
  // Check to see if we're not moving (stuck on object or encoder failure)
  // If we read the same distance on several subsequent calls, stop with error
  //
  if (curDistance == lastDistance) {
     frc::SmartDashboard::PutNumber(  "DistanceDrive sameCounter", sameCounter);
     if (sameCounter++ == 50) {
        drive.TankDrive(0, 0);
        return ERROR;
     }
  } else {
     sameCounter = 0;
     lastDistance = curDistance;
  }

  //
  // Calculate speed
  // If robot is in the ramp up or ramp down portion of the drive distance, then speed is in
  // between AUTOSTARTSPEED and target speed
  //
  std::cout << "Curdist: " <<curDistance<<" speedUpDist: "<<speedUpDistance<<" absspeed: "<<speed<<std::endl;
  std::cout << "slowDownDist: "<<slowDownDistance<<std::endl;

  if (curDistance < speedUpDistance) {
	newSpeed = AUTOSTARTSPEED + ((speed - AUTOSTARTSPEED) * curDistance)/DRIVERAMPUPDISTANCE;
  std::cout << "1 newSpeed: " <<newSpeed<<std::endl;
  } else if (curDistance > slowDownDistance) {
	newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
  std::cout << "2 newSpeed: " <<newSpeed<<std::endl;
  } else {
	newSpeed = speed;
  std::cout << "3 newSpeed: " <<newSpeed<<std::endl;
  }
  frc::SmartDashboard::PutNumber(  "DistanceDrive newSpeed", newSpeed);

  /* drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
     drive.TankDrive(-1*newSpeed*AUTOFORWARD,newSpeed*AUTOFORWARD); */

  StraightDrive(speedMult * newSpeed);
  curDistance = (fabs(leftDriveEncoder.GetDistance())+fabs(rightDriveEncoder.GetDistance()))/2;
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

//
// DistanceTurn: Drive only one side of robot a specified distance
//                 (make a turn)
// Input parms
//    robotSide:   Which side to drive.
//    targetDistance:    Distance to drive, in inches.
//                 Specify a negative value to drive backwards.
//    brake:       Specify TRUE to apply a brief bit of power in reverse
//                 after reaching target distance to avoid going too far
//
// Robot will start driving at a speed equal to AUTOSTARTSPEED and increase speed
// proportionally until it hits the target speed at a distance of DRIVERAMPUPDISTANCE.
// The drive at the specified speed until it gets within DRIVERAMPUPDISTANCE of the
// target distance at which point it slows down to AUTOSTARTSPEED by the end of the
// drive, then stops. When the brake parm is TRUE, it drives at low speed in the opposite
// direction for a fraction of a second to try to stop without overrunning the target
// distance.
//
int Robot::DistanceTurn (enum robotSideTypes robotSide, float targetDistance, bool brake)
{

  static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static float speedMult;           // -1 or 1 speed multiplier based on direction and AUTOFORWARD value
  static float leftSpeed, rightSpeed, leftBrakeSpeed, rightBrakeSpeed;
  static double lastDistance, distance;

  static int sameCounter;

  static bool brakingFlag;
  static units::time::second_t brakeStartTime;

  double curDistance;

  //
  // Set up initial values on first call to DistanceDrive only
  //
  if (FirstCallFlag) {
    brakingFlag = false;
    FirstCallFlag = false;

    //
    // If speed is negative, it means drive backwards.
    //
    if (targetDistance  < 0) {
       distance = fabs(targetDistance);
       speedMult = -1 * AUTOFORWARD;
    } else {
       distance = targetDistance;
       speedMult = 1 * AUTOFORWARD;
    }
    if (robotSide == leftSide) {
       leftSpeed = AUTOTURNSPEED;
       rightSpeed = 0;
       leftBrakeSpeed = -0.2;
       rightBrakeSpeed = 0;
    } else {
       leftSpeed = 0;
       rightSpeed = AUTOTURNSPEED;
       leftBrakeSpeed = 0;
       rightBrakeSpeed = -0.2;
    }

    frc::SmartDashboard::PutNumber(  "DistanceTurn Distance", distance);
    lastDistance = 0;
    sameCounter = 0;
    leftDriveEncoder.Reset();
    rightDriveEncoder.Reset();
  }

  //
  // See if we're done driving and just braking
  //
  if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < 0.2_s) {
      // Not done braking yet, continue to drive slowly in opposite direction
      drive.TankDrive(-1*speedMult*leftBrakeSpeed, speedMult*rightBrakeSpeed);
      return NOTDONEYET;
    } else {
      // Done braking yet, stop
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
  }

  //
  // Determine distance travelled so far
  //
  curDistance = (fabs(leftDriveEncoder.GetDistance())+fabs(rightDriveEncoder.GetDistance()))/2;
  frc::SmartDashboard::PutNumber(  "DistanceDrive curDistance", curDistance);

  //
  // Check to see if we're not moving (stuck on object or encoder failure)
  // If we read the same distance on several subsequent calls, stop with error
  //
  if (curDistance == lastDistance) {
     frc::SmartDashboard::PutNumber(  "DistanceDrive sameCounter", sameCounter);
     if (sameCounter++ == 50) {
        drive.TankDrive(0, 0);
        return ERROR;
     }
  } else {
     sameCounter = 0;
     lastDistance = curDistance;
  }

  drive.TankDrive(-1*speedMult*leftSpeed, speedMult*rightSpeed);

  if (robotSide == leftSide) {
     curDistance = fabs(leftDriveEncoder.GetDistance());
  } else {
     curDistance = fabs(rightDriveEncoder.GetDistance());
  }

//  std::cout << "Curdist: " <<curDistance<<" target: "<<distance<<" speed: "<<newSpeed<<std::endl;
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
