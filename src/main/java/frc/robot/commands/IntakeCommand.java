/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeCommand extends CommandBase {
  public IntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
  }

  private void requires(IntakeSubsystem intake) {
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
      Robot.intake.intakeIn();
    else if (RobotContainer.auxController.getRawAxis(3) < 0.6)
     Robot.intake.intakeOut();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
   protected void end() {
    Robot.intake.stop();
   }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
