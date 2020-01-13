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
    double angleOffset;
    double trueTargetAngle;

    /**
     * Constructs a SwerveModule with an assigned drive motor and angle motor
     * 
     * @param driveMotor Motor used to drive module wheel
     * @param angleMotor Motor used to rotate module wheel
     */
    public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, double angleOffset){
        //creates reference to assigned motor
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;

        //assigns encoders (used for diagnostics)
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);

        //assigns PID Controllers
        drivePID = driveMotor.getPIDController();
        anglePID = angleMotor.getPIDController();

        anglePID.setFeedbackDevice(angleEncoder);

        //sets PID constants
        setPIDTerms(drivePID, true);
        setPIDTerms(anglePID, false);

        this.angleOffset = angleOffset;

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
        //System.out.println("target velocity = " + targetVelocity);
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

        targetMotorAngle = realignAndOffsetEncoder(targetAngle.getDegrees());
        //System.out.println("targetAngle = " + targetMotorAngle);

        return targetAngle;
    }

    /**
    * Sets drive motor power to reach and maintain a desired velocity
    * 
    * @param targetVelocity desired velocity of the module in meters/second
    */
    public void setModuleVelocity(double targetVelocity){
      //System.out.println("velocity = " + targetVelocity);
      drivePID.setReference(targetVelocity, ControlType.kVelocity);
    }

    /**
     * Sets angle motor power to reach and maintain a desired angle
     * 
     * @param targetAngle
     */
    public void setModuleAngle(Rotation2d targetAngle){

      trueTargetAngle = targetAngle.getDegrees();
      
      trueTargetAngle = realignAndOffsetEncoder(trueTargetAngle);
      
      double error = trueTargetAngle - getAngleEncoder();
      System.out.println("error = " + error);

      /*boolean isInverted = false;
      if(!isInverted){

      }*/

      anglePID.setReference(trueTargetAngle, ControlType.kPosition);
    }

    public double getModuleAngle(){
     // System.out.println(trueTargetAngle);
      return trueTargetAngle;
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

  public double realignAndOffsetEncoder(double encoderAngle){
  
    double realignedAngle = realignEncoder(encoderAngle);
    //System.out.println("realignedAngle = " + realignedAngle);

    realignedAngle = offsetEncoder(encoderAngle);
    //System.out.println("offsetAngle = " + realignedAngle);

    return realignedAngle;
  }

  public double realignEncoder(double encoderAngle){

    double realignedAngle = encoderAngle;
    if(realignedAngle < 0){
      realignedAngle += 360;
    }
    return realignedAngle;

  }

  public double offsetEncoder(double encoderAngle){
  
    double realignedAngle = encoderAngle;
    //System.out.println("offset = " + angleOffset);
    realignedAngle = ((realignedAngle + angleOffset) % 360);
    if(realignedAngle < 0){
      realignedAngle += 360;
    }

    return realignedAngle;
  }

  public void flipAngleMotorInversion(){
    if(angleMotor.getInverted()){
      angleMotor.setInverted(false);
    } else if(!angleMotor.getInverted()){
      angleMotor.setInverted(true);
    }
    
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

  public double getAngleEncoderWithOffset(){
    return offsetEncoder(getAngleEncoder());
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

    anglePID.setReference(targetAngle.getDegrees(), ControlType.kPosition);
  }
}
