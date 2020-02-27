/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import static frc.robot.Constants.*;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import static frc.robot.util.GeneralUtil.*;

import frc.robot.RobotContainer;
import frc.robot.util.GeneralUtil.PIDProfile;

/**
 * This class assigns motors to a given swerve module on the robot (eg frontLeft, etc)
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
  double trueTargetAngle;
  boolean isWheelAligned;

  double absoluteOffset;
  double dashboardOffset;
  double relativeOffset;

  /**
   * Constructs a SwerveModule with an assigned angle and drive motor and an offset value
   * 
   * @param driveMotor Spark MAX used to drive module wheel
   * @param angleMotor Spark MAX used to rotate module wheel
   * @param angleOffset Angular offset (degrees)
   */
  public SwerveModule(CANSparkMax driveMotor, CANSparkMax angleMotor, double angleOffset, double dashboardOffset){
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

    //assigns angle encoder to PID
    //anglePID.setFeedbackDevice(absoluteAngleEncoder);
    anglePID.setFeedbackDevice(relativeAngleEncoder);
    
    //sets PID constants
    setPIDGains(drivePID, PIDProfile.DRIVE);
    setPIDGains(anglePID, PIDProfile.ANGLE);

    //assigns absolute encoder offset values
    this.absoluteOffset = angleOffset;
    this.dashboardOffset = dashboardOffset;

    //sets conversion factors (native unit into usable unit)
    absoluteAngleEncoder.setPositionConversionFactor(kAnglePositionConversionFactor); //voltage into degrees
    driveEncoder.setVelocityConversionFactor(kDriveVelocityConversionFactor); //rpm into MPS  

    //disables soft limits on Spark MAXs
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    isWheelAligned = false;

    zeroEncoder();
  }

  /**
   * METHODS
   */

  /** */
  public double getTargetVelocity(SwerveModuleState state){
    targetVelocity = state.speedMetersPerSecond;

    return targetVelocity;
  }

  public Rotation2d getTargetAngle(SwerveModuleState state){
    targetAngle = state.angle;

    targetMotorAngle = realignAndOffsetEncoder(targetAngle.getDegrees());

    return targetAngle;
  }


  public void setModuleVelocity(double targetVelocity){

    //System.out.println("velocity = " + targetVelocity);
  
    drivePID.setReference(targetVelocity, ControlType.kVelocity);
    
  }

  public void setModuleAngle(Rotation2d targetAngle){

      double target = targetAngle.getDegrees();
      target *= kModuleDegreesToRelativeRotations;
      double current = getRelativeAngleEncoder();

      //adjusts target to be in appropriate range of rotation based on current position
      if(Math.abs(current) > kRelativeRotationsPerModuleRotation){
        double rotError = 0.0;
        if(current > 0){
          rotError = Math.floor(current / kRelativeRotationsPerModuleRotation);
        } else if(current < 0){
          rotError = Math.ceil(current / kRelativeRotationsPerModuleRotation);
        }
        target += (rotError * kRelativeRotationsPerModuleRotation);
      }
      
      double error = target - current;

      //increases target by rotation if taking a inefficient path
      if(Math.abs(error) > kRelativeRotationsPerModuleRotation / 2){
        if(current > 0){
          target += kRelativeRotationsPerModuleRotation;
        } else if(current < 0){
          target -= kRelativeRotationsPerModuleRotation;
        }
      }
      trueTargetAngle = target;

      anglePID.setReference(target, ControlType.kPosition); 
  }

  /**
   * Realigns a target angle in the -180 to 180 degree range into the 0 to 360 degree range
   * and applys offset to the angle
   * 
   * @param encoderAngle angle in -180 to 180 degree range
   * @return Offset angle in 0 to 360 degree range
   */
  public double realignAndOffsetEncoder(double encoderAngle){
  
    double realignedAngle = realignEncoderRange(encoderAngle);
    realignedAngle = offsetEncoder(encoderAngle);

    return realignedAngle;
  }
  
  /**
   * Realigns a target angle in the -180 to 180 degree range into the 0 to 360 degree range
   * 
   * @param encoderAngle angle in -180 to 180 degree range
   * @return angle in 0 to 360 degree range
   */
  public double realignEncoderRange(double encoderAngle){

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
    realignedAngle = ((realignedAngle + absoluteOffset) % 360);
    if(realignedAngle < 0){
      realignedAngle += 360;
    }
    return realignedAngle;
  }

  public void zeroModules(){
    if(!getIsWheelAligned()){
      zeroEncoder();
      
      double moduleAngle = getModulePosition();
      double relativeAngle = moduleAngle * kModuleDegreesToRelativeRotations;
      setEncoder(relativeAngle);
      isWheelAligned = true;
    }
  }

  public void zeroEncoder(){
    setEncoder(0.0);
  }

  public void setEncoder(double position){
    relativeAngleEncoder.setPosition(position);
  }

  public void stop(){
    setModuleVelocity(0);
    driveMotor.set(0);
  }

  /**
   * DIAGNOSTIC METHODS 
   */

  /**
   * encoder getters
   */

  public double getDriveVelocity(){
    double vel = driveEncoder.getVelocity();
    vel /= kMaxSpeedConversionFactor;
    return Units.metersToFeet(vel);
  }

  public double getAbsoluteAngleEncoder(){
    return absoluteAngleEncoder.getPosition();
  }

  public double getAbsoluteAngleEncoderWithOffset(){
    return offsetEncoder(getAbsoluteAngleEncoder());
  }

  public double getModulePosition(){
    double angle = getAbsoluteAngleEncoderWithOffset() - dashboardOffset;
    if(angle < 0){
      angle += 360;
    } 
    return angle;
  }

  public double getRelativeAngleEncoder(){
    return relativeAngleEncoder.getPosition();
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

  public double getTrueTargetAngle(){
    return trueTargetAngle;
  }

  //unused
  public double getRelativeOffset(){
    return relativeOffset;
  }

  public double getDashboardOffset(){
    return dashboardOffset;
  }

  public boolean getIsWheelAligned(){
    return isWheelAligned;
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

  //uses absolute encoder
  public void setAngleSetpoint(double xInput, double yInput){
    Rotation2d targetAngle = new Rotation2d(xInput, yInput);
    System.out.println(targetAngle);

    anglePID.setReference(targetAngle.getDegrees(), ControlType.kPosition);
  }
}
