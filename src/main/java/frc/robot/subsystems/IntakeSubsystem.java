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
  // neo motor
  CANSparkMax intakeMotor;
  //piston
  public Solenoid intakePiston;

  public IntakeSubsystem(){
    //defines neoMotor
    intakeMotor = new CANSparkMax(ID_RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    intakePiston = new Solenoid(kIntakePiston);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(kCurrentLimit);
  }

//intakes balls
  public void intakeIn() {
  intakeMotor.set(.6);
  intakePiston.set(true);
  }
//stops
  public void stop() {
intakeMotor.set(0);
intakePiston.set(false);
  }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
