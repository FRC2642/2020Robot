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
 * This class assigns motors to a given swerve module on the robot (eg frontLeft, etc) and
 * and uses information from an instance of a SwerveModuleState object to set the modular wheel angle
 * and modular velocity to the desired value.
 */
public class SwerveModule {

  CANSparkMax driveMotor, angleMotor;
  CANPIDController drivePID, anglePID;
  CANEncoder driveEncoder;
  CANEncoder relativeAngleEncoder;
  CANAnalog absoluteAngleEncoder;

  double targetVelocity;
  Rotation2d targetAngle;
  double targetMotorAngle;
  double angleOffset;
  double trueTargetAngle;

  /**
   * Constructs a SwerveModule with an assigned angle and drive motor and an offset value
   * 
   * @param driveMotor Spark MAX used to drive module wheel
   * @param angleMotor Spark MAX used to rotate module wheel
   * @param angleOffset Angular offset (degrees)
   */
  public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, double angleOffset){
    //creates reference to assigned motor
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;

    //assigns encoders 
    driveEncoder = driveMotor.getEncoder();
    absoluteAngleEncoder = angleMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
    relativeAngleEncoder = angleMotor.getEncoder();

    //assigns PID Controllers
    drivePID = driveMotor.getPIDController();
    anglePID = angleMotor.getPIDController();

    //assigns analog enconder to angle PID
    anglePID.setFeedbackDevice(absoluteAngleEncoder);

    //sets PID constants
    setPIDTerms(drivePID, true);
    setPIDTerms(anglePID, false);

    //assigns angle offset value
    this.angleOffset = angleOffset;

    //sets conversion factors (native unit into usable unit)
    absoluteAngleEncoder.setPositionConversionFactor(kAnglePositionConversionFactor); //voltage into degrees
    driveEncoder.setVelocityConversionFactor(kDriveVelocityConversionFactor); //rpm into MPS  
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
   * Feeds a desired modular velocity into the closed-loop velocity controller
   * 
   * @param targetVelocity desired velocity of the module in meters/second
   */
  public void setModuleVelocity(double targetVelocity){
    //System.out.println("velocity = " + targetVelocity);
    drivePID.setReference(targetVelocity, ControlType.kVelocity);
  }

  /**
   * Feeds a desired modular wheel angle into the closed-loop position controller
   * 
   * @param targetAngle
   */
  public void setModuleAngle(Rotation2d targetAngle){

    trueTargetAngle = targetAngle.getDegrees();  
    trueTargetAngle = realignAndOffsetEncoder(trueTargetAngle);
      
    //calculates error for optimization purposes
    double error = trueTargetAngle - getAbsoluteAngleEncoder();
    //System.out.println("error = " + error);

    anglePID.setReference(trueTargetAngle, ControlType.kPosition);
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

  /**
   * Realigns a target angle in the -180 to 180 degree range into the 0 to 360 degree range
   * and applys offset to the angle
   * 
   * @param encoderAngle angle in -180 to 180 degree range
   * @return Offset angle in 0 to 360 degree range
   */
  public double realignAndOffsetEncoder(double encoderAngle){
  
    double realignedAngle = realignEncoder(encoderAngle);
    realignedAngle = offsetEncoder(encoderAngle);

    return realignedAngle;
  }

  /**
   * Realigns a target angle in the -180 to 180 degree range into the 0 to 360 degree range
   * 
   * @param encoderAngle angle in -180 to 180 degree range
   * @return angle in 0 to 360 degree range
   */
  public double realignEncoder(double encoderAngle){

    double realignedAngle = encoderAngle;
    if(realignedAngle < 0){
      realignedAngle += 360;
    }
    return realignedAngle;
  }

  /**
   * Applies an offset to the target angle
   * 
   * @param encoderAngle angle in 0 to 360 degree range
   * @return offset angle in 0 to 360 degree range
   */
  public double offsetEncoder(double encoderAngle){
  
    double realignedAngle = encoderAngle;
    //System.out.println("offset = " + angleOffset);
    realignedAngle = ((realignedAngle + angleOffset) % 360);
    if(realignedAngle < 0){
      realignedAngle += 360;
    }
    return realignedAngle;
  }

  /**
   * DIAGNOSTIC METHODS 
   */

  /**
   * encoder getters
   */

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getAbsoluteAngleEncoder(){
    return absoluteAngleEncoder.getPosition();
  }

  public double getAbsoluteAngleEncoderWithOffset(){
    return offsetEncoder(getAbsoluteAngleEncoder());
  }

  public double getRelativeAngleEncoder(){
    return relativeAngleEncoder.getPosition();
  }

  public double[] getAbsoluteAndRelativeAngleEncoderPositions() {
    
    double absoluteEncoderPosition = getAbsoluteAngleEncoderWithOffset();
    double relativeEncoderPosition = getRelativeAngleEncoder();
    double[] encoderPositions = {absoluteEncoderPosition, relativeEncoderPosition};
    return encoderPositions;
  }

  /**
   * module variable getters 
   */

  public double getTargetVelocity(){
    return targetVelocity;
  }

  public double getTargetAngle(){
    return targetMotorAngle;
  }

  public double getModuleAngle(){
    return trueTargetAngle;
  }

  public double getSwerveModuleSpeed(){
    return targetVelocity;
  }

  public Rotation2d getSwerveModuleAngle() {
    return targetAngle;
  }

  
  /**
   * status information about motors
   */

  public double getDriveMotorOutput(){
    return driveMotor.get();
  }

  public double getAngleMotorOutput(){
    return angleMotor.get();
  }

  /**
   * testing methods
   */
  
  public void testDriveMotor(double input){
    driveMotor.set(input);
  }

  public void testAngleMotor(double input){
    angleMotor.set(input);
  }

  public void setAngleSetpoint(double xInput, double yInput){
    Rotation2d targetAngle = new Rotation2d(xInput, yInput);
    System.out.println(targetAngle);

    anglePID.setReference(targetAngle.getDegrees(), ControlType.kPosition);
  }

}
