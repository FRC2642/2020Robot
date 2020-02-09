/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ControlType;

//ekatni is intake backwards, as shooting is the reverse of grabbing
public class ShooterSubsystem extends SubsystemBase {
  
  /**
   * Creates a new EkatniSubsystem.
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
  }

  public void stop() {
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
