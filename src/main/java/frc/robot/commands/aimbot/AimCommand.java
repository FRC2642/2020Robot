/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.aimbot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AimCommand extends CommandBase {
  
  SwerveDriveSubsystem drive;
  ArmSubsystem arm;
  ShooterSubsystem shooter;

  double x;
  double y;
  double rotate;

  public AimCommand(SwerveDriveSubsystem driveSub, ArmSubsystem armSub, ShooterSubsystem shootSub) {
    drive = driveSub;
    arm = armSub;
    shooter = shootSub;
    addRequirements(drive, arm, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(kShooterRPM);

    x = RobotContainer.driveController.getRawAxis(0);
    y = -RobotContainer.driveController.getRawAxis(1);

    rotate = 0; //rotate based whether the camera is left or right of the x center of the bounding rectangle

    drive.driveByAimbot(x, -y, rotate);

    
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
