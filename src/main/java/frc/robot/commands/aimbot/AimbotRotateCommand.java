/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.aimbot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AimbotRotateCommand extends CommandBase {
  
  SwerveDriveSubsystem drive;

  double x;
  double y;
  double rotate;

  public AimbotRotateCommand(SwerveDriveSubsystem driveSub) {
    drive = driveSub;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    x = RobotContainer.driveController.getRawAxis(0);
    y = -(RobotContainer.driveController.getRawAxis(1));

    rotate = 0; //rotate based whether the camera is left or right of the x center of the bounding rectangle

    drive.driveByAimbot(x, y, rotate);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      drive.lockWheels();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
/* ;allows robot to take a fat rip
pick up juul
move juul to intake aka mouth
take a fat hit
50 nic bro
get buzzed
send it into the sunset */