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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // neo motor
  CANSparkMax intakeMotor;

  public IntakeSubsystem(){
    //defines neoMotor
    intakeMotor = new CANSparkMax(ID_RIGHT_SHOOTER_MOTOR, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(kCurrentLimit);
  }

//grabs balls
  public void intakeIn() {

  }

  public void stop() {

  }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
