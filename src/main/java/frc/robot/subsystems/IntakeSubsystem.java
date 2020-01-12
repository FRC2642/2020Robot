/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;


public class IntakeSubsystem extends SubsystemBase {

  Spark intakeMotor1 = new Spark(Constants.intakeMotor1Port);
  Spark intakeMotor2 = new Spark(Constants.intakeMotor2Port);

  public DigitalInput intakeSwitch = new DigitalInput(Constants.intakeLimitSwitch);
  
  public IntakeSubsystem() {
    intakeMotor2.setInverted(true);
  }
    public void intake() {
   }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new IntakeCommand());
  }
  public void intakeIn() {
    intakeMotor1.set(-0.6);
    intakeMotor2.set(-0.6);
}
public void intakeOut() {
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








