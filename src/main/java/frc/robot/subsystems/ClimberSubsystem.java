/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ClimberSubsystem extends SubsystemBase {
  
  public DigitalInput climberLowerLimitSwitch = new DigitalInput(Constants.khangerLowerLimitSwitch);  
  private CANEncoder climberEncoder;
  public CANSparkMax climberMotor;
  public CANPIDController climberPID;

  public ClimberSubsystem(){
    climberMotor = new CANSparkMax(ID_CLIMBER_MOTOR, MotorType.kBrushless);
    climberMotor.restoreFactoryDefaults(); // set motor to defaults
    climberMotor.setInverted(false); // makes sure the motor is not inverted
    climberMotor.setSmartCurrentLimit(kCurrentLimit); // sets limit on motor

    climberEncoder = climberMotor.getEncoder();

    climberPID = climberMotor.getPIDController();
    climberPID.setFeedbackDevice(climberEncoder);
    
  }

  public void climberReference(double setPoint){
    climberPID.setReference(setPoint, ControlType.kPosition);
  }
  public void climbMove(double d, double e) {
  }

  public void stop() {
    climberMotor.set(0);
  }

  public double getEncoder(){
    return climberEncoder.getPosition();
  }

}