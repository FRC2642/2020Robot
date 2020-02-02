/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommand;

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

  public DigitalInput intakeSwitch = new DigitalInput(kIntakeLimitSwitch);
  
  public void intake() {
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

   setDefaultCommand(new IntakeCommand());
    }
//grabs balls
  public void intakeIn() {

  }

  public void stop() {

  }
    public boolean getIntakeLimitSwitch(){
      return !intakeSwitch.get();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
	public void setDefaultCommand(final IntakeCommand intakeCommand, final IntakeSubsystem intake) {
	}
}
