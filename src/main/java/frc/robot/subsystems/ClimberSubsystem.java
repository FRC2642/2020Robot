/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Hanger Articulating Network Generating Ethernet Redirecter

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  //VictorSPX climberMotor;
  TalonSRX climberMotor;
  public Solenoid climberPis = new Solenoid(kClimberPistonPort);

  public boolean isClimbLocked;

  public ClimberSubsystem(){

    /* climberMotor = new VictorSPX(ID_CLIMBER_MOTOR);
    climberMotor.setInverted(true); */

    climberMotor = new TalonSRX(ID_CLIMBER_MOTOR);
    climberMotor.configFactoryDefault();
    climberMotor.setInverted(true);
    //climberMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, climberMotor.getDeviceID());

    isClimbLocked = getClimbLock();
  }
  
  /**
   * CLIMBER MOTOR SETTERS
   */
  /** */
  public void climb(double speed){
    if(!getClimbLock()){
      if(speed > .5){
        climbUp();
      } else if(speed < -.5){
        climbDown();
      } else {
        stop();
      }
    } else {
      stop();
    }
  }

  public void setClimbPower(double power){
    climberMotor.set(ControlMode.PercentOutput, power);
  }

  public void climbUp(){
      setClimbPower(.7);
  }

  public void climbDown(){
    setClimbPower(-0.7);
  }

  public void stop() {
    climberMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * CLIMB LOCK GETTERS AND SETTERS
   */
  /** */
  public void toggleClimbLock(){
    if(isClimbLocked){
      setClimbPiston(false);
      isClimbLocked = false;

    } else if(!isClimbLocked){
      climberPis.set(true);
      isClimbLocked = true;
    }
  }

  public boolean getClimbLock(){
    return !climberPis.get();
  }

  public void setClimbPiston(boolean state){
    climberPis.set(state);
    isClimbLocked = state;
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("climb", getClimbLock());
  }
}