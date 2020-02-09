/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  public Solenoid intakePiston;

  public IntakeSubsystem(){
    intakeMotor = new CANSparkMax(ID_INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(kCurrentLimit);

    intakePiston = new Solenoid(kIntakePistonPort);
  }

  //extends and runs intake
  public void intakeIn() {
    intakeMotor.set(.6);
    intakePiston.set(false);
  }

  //stops intake
  public void stop() {
    intakeMotor.set(0);
    intakePiston.set(true);
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
