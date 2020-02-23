/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Hanger Articulating Network Generating Ethernet Redirecter

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Add your docs here.
 */

public class ClimberSubsystem extends SubsystemBase {
  
  VictorSPX climberMotor;

  public Solenoid climberPis = new Solenoid(kClimberPistonPort);
  public DigitalInput climberLimitSwitch = new DigitalInput(8);

  public ClimberSubsystem(){
    climberMotor = new VictorSPX(ID_CLIMBER_MOTOR);
    climberMotor.setInverted(true);
  }
  
  public void setClimbPower(double power){
    climberMotor.set(ControlMode.PercentOutput, power);
  }

  public void climbUp(){
    if(!getLimitSwitch()){
      climberPis.set(false);
      setClimbPower(.7);
    } else {
      stop();
    }
  }

  public void climbDown(){
    climberPis.set(false);
    setClimbPower(-.7);
  }

  public void climb(double speed){
    if(speed > .5){
      climbUp();
    } else if(speed < -.5){
      climbDown();
    } else {
      stop();
    }
  }

  public boolean getLimitSwitch(){
    return climberLimitSwitch.get();
  }

  public void stop(){
    climberMotor.set(ControlMode.PercentOutput, 0);
    climberPis.set(true);
  }

}