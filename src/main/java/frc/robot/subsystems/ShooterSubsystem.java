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
  CANPIDController shooterPID;

  double targetVelocity;

  public ShooterSubsystem() {
    //declare motors
    leftShooterMotor = new CANSparkMax(ID_LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ID_RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    //reset motor
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    //set to not inverted
    leftShooterMotor.setInverted(true);
    rightShooterMotor.setInverted(true);
    //set current limit
    leftShooterMotor.setSmartCurrentLimit(kCurrentLimit);
    rightShooterMotor.setSmartCurrentLimit(kCurrentLimit);

    //PID
    shooterPID = leftShooterMotor.getPIDController();
    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    shooterPID.setFeedbackDevice(leftShooterEncoder);
    shooterPID.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    setPIDGains(shooterPID, PIDProfile.SHOOTER);

    rightShooterMotor.follow(leftShooterMotor, true);

    targetVelocity = kDefaultShooterRPM;
  }

  /**
   * VELOCITY SETTERS
   */
  /** */
  public void setShooterSpeed(double targetVel){
    shooterPID.setReference(targetVelocity, ControlType.kVelocity);
    targetVelocity = targetVel;
  }

  public void setShooterSpeed(){
    shooterPID.setReference(kDefaultShooterRPM, ControlType.kVelocity);
    targetVelocity = kDefaultShooterRPM;
  }

  public void stop() {
    leftShooterMotor.set(0);
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
    return getAverageVelocity() >= targetVelocity;
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


    SmartDashboard.putBoolean("atTargetVelocity", isAtTargetVelocity());

    SmartDashboard.putNumber("av vel", getAverageVelocity());
    SmartDashboard.putNumber("l vel", getLeftVelocity());
    SmartDashboard.putNumber("r vel", getRightVelocity());
  }
}
