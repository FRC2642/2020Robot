/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
    righShooterMotor.setInverted(false);
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
    rShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    lShooterPID.setOutputRange(kMinOutput, kMaxOutput);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
