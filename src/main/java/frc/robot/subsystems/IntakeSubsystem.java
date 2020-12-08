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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  public DoubleSolenoid intakePiston;

  public IntakeSubsystem(){
    intakeMotor = new CANSparkMax(ID_INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(kCurrentLimit);

    intakePiston = new DoubleSolenoid(kIntakePistonPort1, kIntakePistonPort2);
  }

  /**
   * INTAKE MOTOR + PISTONS SETTERS
   */

   /** */
  public void intakeIn() {
    intakeMotor.set(-.8);
    if(intakePiston.get() != Value.kReverse){
      intakePiston.set(Value.kReverse);
    } else {
      intakePiston.set(Value.kOff);
    }
  }

  public void intakeOut(){
    intakeMotor.set(.8);
    intakePiston.set(Value.kReverse);
  }

  public void intakePistonOut(){
    intakePiston.set(Value.kReverse);
  }

  public void stop() {
    intakeMotor.set(0);
    intakePiston.set(Value.kForward);
  }
    
  /**
   * TRIGGERS
   */

  /** */
  public boolean getLeftTrigger() {
    //stop intake i guess
    double lt = RobotContainer.driveController.getTriggerAxis(Hand.kLeft);
    return (lt > .5);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}



