/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.setPIDGains;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.GeneralUtil.PIDProfile;

public class ShooterSubsystem extends SubsystemBase {
  
  /**
   * Creates a new ShooterSubsystem.
   */

  CANSparkMax leftShooterMotor;
  CANSparkMax rightShooterMotor;
  //declare PIDs
  CANEncoder leftShooterEncoder;
  CANEncoder rightShooterEncoder;
  CANPIDController leftShooterPID;
  CANPIDController rightShooterPID;

  double targetVelocity;

  double velTolerance = 50; //RPM

  public ShooterSubsystem() {
    //declare motors
    leftShooterMotor = new CANSparkMax(ID_LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ID_RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    //reset motor
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    //set to not inverted
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(false);
    //set current limit
    leftShooterMotor.setSmartCurrentLimit(kCurrentLimit);
    rightShooterMotor.setSmartCurrentLimit(kCurrentLimit);

    //PID
    leftShooterPID = leftShooterMotor.getPIDController();
    rightShooterPID = rightShooterMotor.getPIDController();

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    leftShooterPID.setFeedbackDevice(leftShooterEncoder);
    leftShooterPID.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    setPIDGains(leftShooterPID, PIDProfile.SHOOTER);
    setPIDGains(rightShooterPID, PIDProfile.SHOOTER);
/* 
    rightShooterMotor.follow(leftShooterMotor, true); */

    targetVelocity = kShooterDefaultRPM;
  }

  /**
   * VELOCITY SETTERS
   */
  /** */
  public void setShooterSpeed(double targetVel){
    targetVelocity = targetVel;

    rightShooterPID.setReference(targetVelocity, ControlType.kVelocity);
    leftShooterPID.setReference(targetVelocity, ControlType.kVelocity);
  }

  public void setShooterSpeed(){
    //shooterPID.setReference(kShooterDefaultRPM, ControlType.kVelocity);
    targetVelocity = kShooterDefaultRPM;

    rightShooterPID.setReference(targetVelocity, ControlType.kVelocity);
    leftShooterPID.setReference(targetVelocity, ControlType.kVelocity);
    
  }

  public void stop() {
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

  /**
   * VELOCITY GETTERS
   */
  /** */
  public double getAverageVelocity(){
    return ((getLeftVelocity() + getRightVelocity()) / 2);
  }
  
  public double getLeftVelocity(){
    return leftShooterEncoder.getVelocity();
  }

  public double getRightVelocity(){
    return rightShooterEncoder.getVelocity();
  }

  public boolean isAtTargetVelocity(){
    boolean aboveMinThreshold = getAverageVelocity() >= targetVelocity - velTolerance;
    boolean belowMaxThreshhold = getAverageVelocity() <= targetVelocity + velTolerance;
    return (aboveMinThreshold && belowMaxThreshhold);
    //return getAverageVelocity() >= targetVelocity;
  }

  /**
   * TRIGGERS
   */
  /** */
  public boolean getLTrigger(){
    return (RobotContainer.auxController.getTriggerAxis(Hand.kLeft) > .5);
  }

  public boolean getRTrigger(){
    return (RobotContainer.auxController.getTriggerAxis(Hand.kRight) > .5);
  }


  ShuffleboardTab driverView = Shuffleboard.getTab("Driver View");
  
  @Override
  public void periodic() {


    //SmartDashboard.putBoolean("atTargetVelocity", isAtTargetVelocity());

    /* SmartDashboard.putNumber("av vel", getAverageVelocity());*/
    SmartDashboard.putNumber("l vel", getLeftVelocity());
    SmartDashboard.putNumber("r vel", getRightVelocity()); 
  }
}
