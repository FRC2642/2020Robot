/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommand;

//ekatni is intake backwards, as shooting is the reverse of grabbing
public class EkatniSubsystem extends SubsystemBase {
  /**
   * Creates a new EkatniSubsystem.
   */

//declares shooter motors
  Spark ekatniMotor1 = new Spark(kIntakeMotorPort1);
  Spark ekatniMotor2 = new Spark(kIntakeMotorPort2);

  public DigitalInput intakeSwitch = new DigitalInput(kIntakeLimitSwitch);

  public EkatniSubsystem() {
  }
  public void ekatni() {

  }

 private void initDefaultCommand() {
    setDefaultCommand(new IntakeCommand());
 }
  //sets speed for shooter
 public void ekatniOut() {
  ekatniMotor1.set(-0.6);
  ekatniMotor2.set(-0.6);
}
public void stop() {
  ekatniMotor1.set(0.0);
  ekatniMotor2.set(0.0);
}

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
