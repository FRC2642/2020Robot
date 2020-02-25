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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmSubsystem extends ProfiledPIDSubsystem {

  public VictorSPX armMotor;

  public AnalogPotentiometer armPot;

  //public ArmFeedforward armFF;


  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(kTiltP, kTiltI, kTiltD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(kTiltMaxVel, kTiltMaxAccel)));

    armMotor = new VictorSPX(ID_MAG_TILT_MOTOR);

    armPot = new AnalogPotentiometer(kArmPotPort);

    //armFF = new ArmFeedforward(kTiltFFStatic, kTiltFFGrav, kTiltFFVel);
  }

  public void armTrenchPos() {
    enable();
    setGoal(kTrenchPos);
  }

  public void armBasePos() {
    enable();
    setGoal(kNormalPos);
  }

  public void armClimbPos() { 
    enable();
    setGoal(kClimbPos);
  }

  public void goToPosition(double position){
    enable();
    setGoal(position);
  }
  
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    
  }

  @Override
  public double getMeasurement() {
    return armPot.get();
  }

  //non-profiled methods

  public void moveArm(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isManualOverride(){
    return (RobotContainer.auxController.getRawAxis(5) > .2 || RobotContainer.auxController.getRawAxis(5) < -.2);
  }

  public void setVoltage(double outputVoltage){
    
    double realOutput = (outputVoltage / RobotController.getBatteryVoltage());
    armMotor.set(ControlMode.PercentOutput, realOutput);
   }
}
