/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

//import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
//import edu.wpi.first.wpilibj.command.Command;
import frc.robot.util.SwerveModule;



public class InchesDrive extends CommandBase {
  /**
   * Creates a new drive.
   */
  double inches;
  

  public InchesDrive(double inches) {

    SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    addRequirements(drive);
    
    //sets timeout
    if (drive.getPoseXInFeet() > inches/12) {
      drive.drive(inches, 0,0);
    }
    else if (drive.getPoseXInFeet() <= (inches/12) +0.0001) {
      drive.drive(0,0,0);
    }
    this.inches = inches;
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //SwerveModule.zeroDriveEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}