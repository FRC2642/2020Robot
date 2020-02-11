/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
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

  public ClimberSubsystem(){
    climberMotor = new CANSparkMax(ID_CLIMBER_MOTOR, MotorType.kBrushless);
    climberMotor.restoreFactoryDefaults(); // set motor to defaults
    climberMotor.setInverted(false); // makes sure the motor is not inverted
    climberMotor.setSmartCurrentLimit(kCurrentLimit); // sets limit on motor

    climberEncoder = climberMotor.getEncoder();
  }
  
  public void climberUp(){
    climberMotor.set(-.5);
  }

  public void climberDown(){
    climberMotor.set(.5);
  }

  public void stop(){
    climberMotor.set(0);
  }

  public void g(){
    if (getLowerLimitSwitch() || ){
      stop();
    }
  }
  
  public boolean getLowerLimitSwitch(){
    return climberLowerLimitSwitch.get();
  }

  public double getEncoder(){
    return climberEncoder.getPosition();
  }

}