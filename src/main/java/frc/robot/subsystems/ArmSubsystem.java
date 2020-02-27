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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

  public VictorSPX armMotor;
  public AnalogPotentiometer armPot;

  public double input;

  public ArmSubsystem() {
  
    armMotor = new VictorSPX(ID_MAG_TILT_MOTOR);
    armMotor.setInverted(false);

    armPot = new AnalogPotentiometer(kArmPotPort, 100);
  }

  //basic motor methods
  public void moveArm(double speed){
    if(getPot() <= kTrenchPos && speed < 0){
      stop();
    } else if(getPot() >= kClimbPos && speed > 0){
      stop();
    } else {    
      setPower(speed); 
    }
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setPower(double input){
    if(input > .6){
      armMotor.set(ControlMode.PercentOutput, .6);
    }
    armMotor.set(ControlMode.PercentOutput, input);

    this.input = input;
  }

  //aiming inputs
  public boolean isManualOverride(){
    return (RobotContainer.auxController.getRawAxis(5) > .2 || RobotContainer.auxController.getRawAxis(5) < -.2);
  }

  public double getPot(){
    return armPot.get();
  }

  public double getAngleFromVision(){
    
    double dist = Robot.jevoisCam.getDistFromTarget(); //m
    //calculates pot value based on distance from base of target 
    //angle increases as distance decreases
    double targetPos = kArmAngleConversionFactor / dist;

    return targetPos;
  }

  public double getMotorPower(){
    return input;
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("arm manual override", isManualOverride());
  }
}
