/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.SwerveModule;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;



import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDrive extends CommandBase {
  /**
   * Creates a new TurnDrive.
   */
  private double degrees;
  private double negDegrees;
  
  public TurnDrive(double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    addRequirements(drive);
    this.degrees  = degrees;
    this.negDegrees = -degrees;
    //setTimeout(2.0);*/
    drive.frontLeftAngleMotor.setInverted(false);
    drive.backRightAngleMotor.setInverted(false); 


    double frontLeftVelocity = 0.0;
    double frontRightVelocity = 0.0;
    double backLeftVelocity = 0.0;
    double backRightVelocity = 0.0;

    Rotation2d frontLeftAngle = drive.toRotation2d(negDegrees);
    Rotation2d frontRightAngle = drive.toRotation2d(degrees);
    Rotation2d backLeftAngle = drive.toRotation2d(degrees);
    Rotation2d backRightAngle = drive.toRotation2d(negDegrees);

    drive.frontLeftModule.setModuleVelocity(frontLeftVelocity);
    drive.frontRightModule.setModuleVelocity(frontRightVelocity);
    drive.backLeftModule.setModuleVelocity(backLeftVelocity);
    drive.backRightModule.setModuleVelocity(backRightVelocity);


    drive.frontLeftModule.setModuleAngle(frontLeftAngle);   
    drive.frontRightModule.setModuleAngle(frontRightAngle);
    drive.backLeftModule.setModuleAngle(backLeftAngle);
    drive.backRightModule.setModuleAngle(backRightAngle);

    //updates swerve module states for pose 
    SwerveModuleState frontLeft = new SwerveModuleState(frontLeftVelocity, frontLeftAngle);
    SwerveModuleState frontRight = new SwerveModuleState(frontRightVelocity, frontRightAngle);
    SwerveModuleState backLeft = new SwerveModuleState(backLeftVelocity, backLeftAngle);
    SwerveModuleState backRight = new SwerveModuleState(backRightVelocity, backRightAngle);
    
    ChassisSpeeds chassisSpeeds = drive.kinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight);

    drive.moduleStates = drive.kinematics.toSwerveModuleStates(chassisSpeeds);

    

}
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
