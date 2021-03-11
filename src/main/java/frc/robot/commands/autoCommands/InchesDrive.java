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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;





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
   // SwerveModule module = new SwerveModule();
    addRequirements(drive);
    //drive.drive(ticks, 0.0, 0.0);
    //sets timeout
    /*if (drive.getDrivePosition() < ticks) {
      new WaitCommand(5.0)
          .deadlineWith(
            new RunCommand(
              () -> drive.drive(-.3, 0.0, 0.0), drive)
          );  
    }
    else if (drive.getDrivePosition() >= ticks) {
      System.out.println("I am doing the wrong thing right here but this is okay");
      drive.stop();    
    }*/
    
    //drive.drive((ticks - drive.getDrivePosition()) + (Math.abs(ticks - drive.getDrivePosition())), 0.0, 0.0);

    new WaitCommand(30)
      .deadlineWith(
        new RunCommand(
      () -> drive.drive((ticks - drive.getDrivePosition()) + (Math.abs(ticks - drive.getDrivePosition())), 0.0, 0.0), drive)); 
    //System.out.print("I have gotten the drive position");

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // module.zeroDriveEncoder();
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
