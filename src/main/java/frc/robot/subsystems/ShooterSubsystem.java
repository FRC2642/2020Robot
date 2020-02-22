/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.GeneralUtil.PIDProfile;

public class ShooterSubsystem extends SubsystemBase {
  
  /**
   * Creates a new ShooterSubsystem.
   */

  CANSparkMax leftShooterMotor;
  CANSparkMax righShooterMotor;
  //declare PIDs
  CANEncoder lShooterEncoder;
  CANEncoder rShooterEncoder;
  CANPIDController lShooterPID;
  CANPIDController rShooterPID;

  public ShooterSubsystem() {
    //declare motors
    leftShooterMotor = new CANSparkMax(ID_LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    righShooterMotor = new CANSparkMax(ID_RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    //reset motor
    leftShooterMotor.restoreFactoryDefaults();
    righShooterMotor.restoreFactoryDefaults();
    //set to not inverted
    leftShooterMotor.setInverted(false);
    righShooterMotor.setInverted(true);
    //set current limit
    leftShooterMotor.setSmartCurrentLimit(kCurrentLimit);
    righShooterMotor.setSmartCurrentLimit(kCurrentLimit);
    //PID
    lShooterPID = leftShooterMotor.getPIDController();
    rShooterPID = righShooterMotor.getPIDController();
    lShooterEncoder = leftShooterMotor.getEncoder();
    rShooterEncoder = righShooterMotor.getEncoder();
    lShooterPID.setFeedbackDevice(lShooterEncoder);
    rShooterPID.setFeedbackDevice(rShooterEncoder);
    rShooterPID.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    lShooterPID.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    setPIDGains(lShooterPID, PIDProfile.SHOOTER);
    setPIDGains(rShooterPID, PIDProfile.SHOOTER);
  }

  //sets speed for shooter
  public void shoot() {
 //20, 30, 40, and 60 are example numbers, can and will be changed
 //gets distance to wall from JeVois camera
 double distance = Robot.getDistanceToWall();
 //does math and gets accurate distance
 distance = distance * kArmAngleConversionFactor;
 //does more math with the correct distance and finds the necessary RPM to shoot the ball a certain distance
double shooterRPM = distance * kShooterRPMConversionFactor;
//sets PIDs to make motors run at previously determined RPM
rShooterPID.setReference(shooterRPM, ControlType.kVelocity);
lShooterPID.setReference(shooterRPM, ControlType.kVelocity);
}

public boolean getRightTrigger() {
  double rt = RobotContainer.driveController.getTriggerAxis(Hand.kRight);
  return (rt > .5);
}
  
  public void stop() {
    leftShooterMotor.set(0);
    righShooterMotor.set(0);
  }
  
  public void setShooterSpeed(double targetVelocity){
    lShooterPID.setReference(targetVelocity, ControlType.kVelocity);
    rShooterPID.setReference(targetVelocity, ControlType.kVelocity);  
  }

  public void setShooterSpeed(){
    lShooterPID.setReference(kShooterRPM, ControlType.kVelocity);
    rShooterPID.setReference(kShooterRPM, ControlType.kVelocity);
  }

  double speed;
  public void test(double speed){
    this.speed = speed;
    leftShooterMotor.set(speed);
    righShooterMotor.set(speed);
  }

  public double getSpeed(){
    return speed;
  }

  public boolean getLTrigger(){
    return (RobotContainer.auxController.getTriggerAxis(Hand.kLeft) > .5);
  }

  @Override
  public void periodic() {
  }
}
