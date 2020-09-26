/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Hanger Articulating Network Generating Ethernet Redirecter

package frc.robot.subsystems;

//import frc.robot.RobotContainer;
import static frc.robot.Constants.ID_CLIMBER_MOTOR;
import static frc.robot.Constants.kClimberLimitSwitch;
import static frc.robot.Constants.kClimberPistonPort;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Add your docs here.
 */

public class ClimberSubsystem extends SubsystemBase {
  
  VictorSPX climberMotor;
  public Solenoid climberPis = new Solenoid(kClimberPistonPort);
  public DigitalInput climberLimitSwitch = new DigitalInput(kClimberLimitSwitch);

  public boolean isClimbLocked;

  public ClimberSubsystem(){

    climberMotor = new VictorSPX(ID_CLIMBER_MOTOR);
    climberMotor.setInverted(false);

    isClimbLocked = getClimbLock();
  }
  
  public void setClimbPower(double power){
    climberMotor.set(ControlMode.PercentOutput, power);
  }

  public void climbUp(){
      setClimbPower(.7);
  }

  public void climbDown(){
    setClimbPower(-.7);
  }

  public void climb(double speed){
    if(getClimbLock()){
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

  public boolean getClimbLock(){
    return !climberPis.get();
  }

  public void toggleClimbLock(){
    if(isClimbLocked){
      setClimbPiston(false);
      isClimbLocked = false;

    } else if(!isClimbLocked){
      climberPis.set(true);
      isClimbLocked = true;
    }
  }

  public void setClimbPiston(boolean state){
    if(state){
      climberPis.set(true);
    } else if(!state){
      climberPis.set(false);
    }
  }

  public boolean getLimitSwitch(){
    return climberLimitSwitch.get();
  }

  public void stop() {
    climberMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("climb", getClimbLock());
  }
}