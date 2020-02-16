/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Hanger Articulating Network Generating Ethernet Redirecter

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.util.GeneralUtil.*;`
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */

public class ClimberSubsystem extends SubsystemBase {
  
  private CANEncoder climberEncoder;
  public CANSparkMax climberMotor;
  public CANPIDController climberPID;
  public Solenoid climberPis = new Solenoid(kMagazinePistonPort);


  public ClimberSubsystem(){
    climberMotor = new CANSparkMax(ID_CLIMBER_MOTOR, MotorType.kBrushless);
    climberMotor.restoreFactoryDefaults(); // set motor to defaults
    climberMotor.setInverted(false); // makes sure the motor is not inverted
    climberMotor.setSmartCurrentLimit(kCurrentLimit); // sets limit on motor

    climberEncoder = climberMotor.getEncoder();

    climberPID = climberMotor.getPIDController();
    climberPID.setFeedbackDevice(climberEncoder);
  }

  public void climberMove(double setPoint){
    climberPID.setReference(setPoint, ControlType.kPosition);
  }

  public double getEncoder(){
    return climberEncoder.getPosition();
  }

  public void climberLimits(){
    if ( getEncoder() == 0 || getEncoder() == 1){ // These are just sample numbers, will be changed
      stop();
    }
  }
  
  public void climberDown(){
    climberPis.set(false);
    climberMotor.set(0); // These are just sample numbers, will be changed
  }

  public void stop(){
    climberMotor.set(0);
    climberPis.set(true);
  }

  public void climberUp(){
    climberPis.set(false);
    climberMotor.set(0); // These are just sample numbers, will be changed
  }
}