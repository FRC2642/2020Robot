/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

  public VictorSPX armMotor;
  public AnalogPotentiometer armPot;

  public double input;

  public double target;

  public ArmSubsystem() {
  
    armMotor = new VictorSPX(ID_MAG_TILT_MOTOR);
    armMotor.setInverted(true);

    armPot = new AnalogPotentiometer(kArmPotPort, 100);

    target = 0;
  }

  /**
   * ARM MOTOR SETTERS
   */
  /** */
  public void moveArm(double speed){
    
      if(getPot() <= kArmTrenchRunPos && speed < 0){
      stop();
    } else if(getPot() >= kArmClimbPos && speed > 0){
      stop();
    } else {      
      setPower(speed); 
    }
  }

  public void setPower(double input){

    SlewRateLimiter speedLimiter = new SlewRateLimiter(2);
    speedLimiter.calculate(input);

    if(input > .6){
      armMotor.set(ControlMode.PercentOutput, .6);
    }
    armMotor.set(ControlMode.PercentOutput, input);

    this.input = input;
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * AIMING METHODS (pot + vision) AND MANUAL OVERRIDE
   */
  /** */
  public boolean getManualOverride(){
    return (RobotContainer.auxController.getRawAxis(5) > .2 || RobotContainer.auxController.getRawAxis(5) < -.2);
  }

  public double getPot(){
    return armPot.get();
  }

  public void setTarget(double target){
    this.target = target;
  }

  public double getTarget(){
    return target;
  }

  public boolean isArmAtGoal(){
    if((getPot() >= getTarget() - .5) && (getPot() <= getTarget() + .5)){
      return true;
    } else {
      return false;
    }
  }

  public double getAngleFromVision(){
    
    //double dist = Robot.jevoisCam.getDistFromTarget(); //m
    //calculates pot value based on distance from base of target 
    //angle increases as distance decreases
    double targetPos = kArmAngleConversionFactor / 1;

    return targetPos;
  }

  /**
   * TRIGGERS
   */
  /** */
  public boolean getUpDPad(){
    return RobotContainer.auxController.getPOV() == 0;
  }

  public boolean getDownDPad(){
    return RobotContainer.auxController.getPOV() == 180;
  }

  @Override
  public void periodic(){
    //SmartDashboard.putBoolean("arm manual override", getManualOverride());
    //SmartDashboard.putNumber("target", getTarget());
  }
}
