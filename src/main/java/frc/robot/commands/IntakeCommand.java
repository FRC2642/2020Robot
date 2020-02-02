
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeCommand extends CommandBase {
  public IntakeCommand() {
    // Use requires() here to declare subsystem dependencies
  }

 

// Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //getRawAxis axies values may need changing
    if (RobotContainer.auxController.getRawAxis(3) > 0.6)
      RobotContainer.intake.intakeIn();
    else if (RobotContainer.auxController.getRawAxis(3) < 0.6)
    //ekatni is intake backwards, as shooting is the reverse of grabbing
     RobotContainer.ekatni.ekatniOut();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
   protected void end() {
    RobotContainer.intake.stop();
   }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
