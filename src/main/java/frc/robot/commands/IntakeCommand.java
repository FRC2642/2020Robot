/*----------------------------------------------------------------------------
 Copyright (c) 2019 FIRST. All Rights Reserved.                             
 Open Source Software - may be modified and shared by FRC teams. The code   
 must be accompanied by the FIRST BSD license file in the root directory of 
 the project.                                                               
----------------------------------------------------------------------------
*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class IntakeCommand extends CommandBase {
  
IntakeSubsystem intake;
MagazineSubsystem magazine;

  public IntakeCommand(IntakeSubsystem incom, MagazineSubsystem magcom) {
    intake = incom;
    magazine = magcom;
    addRequirements(intake);
  }


  //Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  //Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.intakeIn();
    magazine.magLoad();
  }

  //Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
 
  // Called once after isFinished returns true
   protected void end() {
   }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
  }
}
