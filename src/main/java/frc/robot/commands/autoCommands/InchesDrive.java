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
  //double x;
  //double y;
  //double r;
  double ticks;

  public InchesDrive(double ticks) {

    SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    addRequirements(drive);
    //drive.drive(ticks, 0.0, 0.0);
    //sets timeout

    //homie, hear me out, full send, no speed regulation
    //idk how the ticks are gonna work tbh

    if (drive.getDrivePosition() < ticks) {
      drive.setWheelsStraight();
    }
    else if (drive.getDrivePosition() >= ticks) {
      drive.lockWheels();
    }
    

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
