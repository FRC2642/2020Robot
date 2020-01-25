/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommand;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
  //intake motors
  Spark intakeMotor1 = new Spark(kIntakeMotorPort1);
  Spark intakeMotor2 = new Spark(kIntakeMotorPort2);

  public DigitalInput intakeSwitch = new DigitalInput(kIntakeLimitSwitch);
  
  public IntakeSubsystem() {
    intakeMotor2.setInverted(true);
  }
  public void intake() {
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new IntakeCommand());
    }

  public void intakeIn() {
    	intakeMotor1.set(-0.6);
    	intakeMotor2.set(-0.6);
  }
  public void intakeOut(double d, double e) {
    intakeMotor1.set(0.6);
    intakeMotor2.set(0.6);
  }
  public void stop() {
    intakeMotor1.set(0.0);
    intakeMotor2.set(0.0);
  }
    public boolean getIntakeLimitSwitch(){
      return !intakeSwitch.get();
    }
}
