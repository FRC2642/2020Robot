/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberBarSubsystem extends SubsystemBase {
 
  public CANSparkMax barMotor;

  public ClimberBarSubsystem() {
    barMotor = new CANSparkMax(ID_CLIMB_BAR_MOTOR, MotorType.kBrushless);
    barMotor.restoreFactoryDefaults(); // set motor to defaults
    barMotor.setInverted(false); // makes sure the motor is not inverted
    barMotor.setSmartCurrentLimit(kCurrentLimit); // sets limit on motor
  }

  public void moveAlongBar(double power){
    barMotor.set(power);
  }

  public void moveLeftAlongBar(){
    moveAlongBar(.7);
  }

  public void moveRightAlongBar(){
    moveAlongBar(-.7);
  }

  public void move(double speed){
    if(speed > .5){
      moveLeftAlongBar();
    } else if(speed < -.5){
      moveRightAlongBar();
    } else {
      stop();
    }
  }

  public void stop(){
    moveAlongBar(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
