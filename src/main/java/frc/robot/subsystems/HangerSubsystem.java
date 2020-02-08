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
public class HangerSubsystem extends SubsystemBase {
  
  public DigitalInput khangerLowerLimitSwitch = new DigitalInput(Constants.khangerLowerLimitSwitch);  
  private CANEncoder hangerEncoder;
  public CANSparkMax hangerMotor;

  public void hangerSubsystem(){
    hangerMotor = new CANSparkMax(ID_HANGER_MOTOR, MotorType.kBrushless);
    hangerMotor.restoreFactoryDefaults(); // set motor to defaults
    hangerMotor.setInverted(false); // makes sure the motor is not inverted
    hangerMotor.setSmartCurrentLimit(kCurrentLimit); // sets limit on motor
    hangerMotor.getEncoder();
  }
  
  public void hangerUp(){
    hangerMotor.set(-.5);
  }

  public void hangerDown(){
    hangerMotor.set(.5);
  }

  public void stop(){
    hangerMotor.set(0);
  }

  public boolean getLowerLimitSwitch(){
    return khangerLowerLimitSwitch.get();
  }

}