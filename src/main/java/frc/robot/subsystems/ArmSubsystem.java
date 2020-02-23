/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmSubsystem extends ProfiledPIDSubsystem {

  public VictorSPX armMotor;
  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(0, 0, 0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));

    armMotor = new VictorSPX(ID_MAG_TILT_MOTOR);
           // armMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 2);
  }

 // public ErrorCode getEncoderValue() {

 // }
  public void armLift(double d) {
    //20, 30, 40, and 60 are example numbers, can and will be changed
    double distance = Robot.getDistanceToWall();
    distance = distance * kArmAngleConversionFactor;
    if (distance > 20 && distance < 40) {
      setGoal(20);
    } else if (distance > 40 && distance < 60) {
      setGoal(30);
    } //add more criterias
  }
  public double getArmPos() {
    int armPos = 0;
   // armMotor.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    return armPos;
}

  public void armDown() {
        if (getArmPos() == 0) {
      armStop();
    } else {
      setGoal(0);
    }
  }
  public void armTrenchPos() {
    setGoal(227.555556);
  }
  public void armBasePos() {
    setGoal(512.000001);
  }
  public void armClimbPos() { 
    if(getArmPos() == 1024){
      armStop();
    }
    setGoal(1024);

    }

  public void moveArm(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public void armStop() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isManualOverride(){
    return (RobotContainer.auxController.getRawAxis(5) > .2 || RobotContainer.auxController.getRawAxis(5) < -.2);
  }
  
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
