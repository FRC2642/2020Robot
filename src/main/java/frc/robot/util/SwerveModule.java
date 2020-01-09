/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import static frc.robot.Constants.kAngleD;
import static frc.robot.Constants.kAngleFF;
import static frc.robot.Constants.kAngleI;
import static frc.robot.Constants.kAngleP;
import static frc.robot.Constants.kAnglePositionConversionFactor;
import static frc.robot.Constants.kDriveD;
import static frc.robot.Constants.kDriveFF;
import static frc.robot.Constants.kDriveI;
import static frc.robot.Constants.kDriveP;
import static frc.robot.Constants.kDriveVelocityConversionFactor;
import static frc.robot.Constants.kMaxOutput;
import static frc.robot.Constants.kMinOutput;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * Contains swerve module motors and current states given by SwerveModuleStates
 */
public class SwerveModule {

    CANSparkMax driveMotor, angleMotor;
    CANPIDController drivePID, anglePID;
    CANEncoder driveEncoder;
    CANAnalog angleEncoder;

    double targetVelocity;
    Rotation2d targetAngle;
    double targetMotorAngle;

    /**
     * Constructs a SwerveModule with an assigned drive motor and angle motor
     * 
     * @param driveMotor Motor used to drive module wheel
     * @param angleMotor Motor used to rotate module wheel
     */
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor){
        //creates reference to assigned motor
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;

        //assigns encoders (used for diagnostics)
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

        //assigns PID Controllers
        drivePID = driveMotor.getPIDController();
        anglePID = angleMotor.getPIDController();

        //sets PID constants
        setPIDTerms(drivePID, true);
        setPIDTerms(anglePID, false);

        angleEncoder.setPositionConversionFactor(kAnglePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(kDriveVelocityConversionFactor);
    }

    /**
     * Gets a target velocity from a SwerveModuleState object
     * 
     * @param state What SwerveModuleState object to read
     * @return Target velocity in m/s
     */
    public double getTargetVelocity(SwerveModuleState state){
        targetVelocity = state.speedMetersPerSecond;
        return targetVelocity;
    }

    /**
     * Gets a target angle from a SwerveModuleState object
     * 
     * @param state What SwerveModuleState object to read
     * @return Target angle in degrees
     */
    public Rotation2d getTargetAngle(SwerveModuleState state){
        targetAngle = state.angle;
        //System.out.println(targetAngle);
        return targetAngle;
  
    }

    /**
    * Sets drive motor power to reach and maintain a desired velocity
    * 
    * @param targetVelocity desired velocity of the module in meters/second
    */
    public void setModuleVelocity(double targetVelocity){
     // System.out.println("velocity = " + targetVelocity);
      drivePID.setReference(targetVelocity, ControlType.kVelocity);
    }

    /**
     * Sets angle motor power to reach and maintain a desired angle
     * 
     * @param targetAngle
     */
    public void setModuleAngle(Rotation2d targetAngle){

      anglePID.setReference(targetAngle.getDegrees(), ControlType.kPosition);
    }

     /**
   * Sets PID terms for swerve module controllers
   * 
   * @param pid the PID controller to set values for
   * @param drive is module a drive module (if false, angle module)
   */
  public void setPIDTerms(CANPIDController pid, boolean isDrive){
    if(isDrive){
      pid.setP(kDriveP);
      pid.setI(kDriveI);
      pid.setD(kDriveD);
      pid.setFF(kDriveFF);
    } else {
      pid.setP(kAngleP);
      pid.setI(kAngleI);
      pid.setD(kAngleD);
      pid.setFF(kAngleFF);
    }
    pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  //DIAGNOSTIC METHODS 

  //encoder getters
  public double getDriveEncoder(){
    return driveEncoder.getVelocity();
  }

  public double getDriveVelocity(){
    return getDriveEncoder();
  }

  public double getAngleEncoder(){
    return angleEncoder.getPosition();
  }

  public double getDriveEncoderPosition(){
    return driveEncoder.getPosition();
  }

  public double getTargetVelocity(){
    return targetVelocity;
  }

  public double getTargetAngle(){
    return targetMotorAngle;
  }

  public void testDriveMotor(double input){
    driveMotor.set(input);
  }

  public void testAngleMotor(double input){
    angleMotor.set(input);
  }

  public double getDriveMotorOutput(){
    return driveMotor.get();
  }

  public double getAngleMotorOutput(){
    return angleMotor.get();
  }

  public void setAngleSetpoint(double xInput, double yInput){
    Rotation2d targetAngle = new Rotation2d(xInput, yInput);
    System.out.println(targetAngle);
    double error = targetAngle.getDegrees() - getAngleEncoder();

    if(error >= 180 || error <= 180){
      
    }

    //half rotation logic = reverses direction of rotation

    //quarter rotation logic = reverses direction of rotation and drive


    anglePID.setReference(targetAngle.getDegrees(), ControlType.kPosition);
  }
}
