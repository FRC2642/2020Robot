/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoCommand extends CommandBase {
  /**
   * Creates a new AutoCommand.
   */
  public AutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  /*@Override
  public void initialize() {
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxMPS, 
    Constants.kMaxAcceleration)
    .setKinematics(kinematics)
    .addConstraint(autoVoltageConstraint);
  }
*/
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /*Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      config
      );
  }
*/
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
