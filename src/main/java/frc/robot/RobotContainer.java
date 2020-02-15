/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSpinnerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final MagazineSubsystem magazine = new MagazineSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem(); 
  public static final ColorSpinnerSubsystem spinner = new ColorSpinnerSubsystem();
  public static final ClimberSubsystem climb = new ClimberSubsystem();
  public static final ArmSubsystem arm = new ArmSubsystem();
 
  //public final Command intakeCommand = new IntakeCommand(intake);


  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);
  public static Trigger leftTrigger = new Trigger(intake::getLeftTrigger);
  public static Trigger rightTrigger = new Trigger(shooter::getRightTrigger);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();
   
    drive.setDefaultCommand(
      new RunCommand(
        () -> drive.drive(
          //.5, 0, 0),
             -(driveController.getRawAxis(1)) * .7, 
          driveController.getRawAxis(0) * .7, 
          driveController.getRawAxis(4)),  
          drive)
      );
    
    /* arm.setDefaultCommand(
      new RunCommand(
        () -> arm.armLift(
          (auxController.getRawAxis(5) * .5)
       )
      )
    );

    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.stop()
       )
     );

    magazine.setDefaultCommand(
      new RunCommand (
        () -> magazine.stop()
      )
    );

    climb.setDefaultCommand(
      new RunCommand (
        () -> climb.stop()
      )
    );

    spinner.setDefaultCommand(
      new RunCommand(
        () -> spinner.stop()
      )
    ); */

    
    
    //manually drives motors, leave out unless testing 
    /*drive.setDefaultCommand(
      new RunCommand(
        () -> drive.motorTest(drive.frontRightModule,
              -driveController.getRawAxis(1), -driveController.getRawAxis(5)),
              drive)
    );*/ 
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //instantiates drive toggle button
    new JoystickButton(driveController, Button.kBack.value)
      .whenPressed(new InstantCommand(drive::toggleIsDriveFieldCentric));

    /**
     * Everything below here requires reworking.
     */
    //toggles aiming mode
    /* new JoystickButton(driveController, Button.kStart.value)
      .whenPressed(new InstantCommand(drive::toggleIsAimingMode, drive));
    //rotates ColorSpinner motor left/Counter Clockwise
    new JoystickButton(auxController, Button.kX.value)
      .whenHeld(new RunCommand(spinner::spinL, spinner));
    //rotates ColorSpinner motor right/Clockwise
    new JoystickButton(auxController, Button.kB.value)
      .whenHeld(new RunCommand(spinner::spinR, spinner));
    //magazine belt
    new JoystickButton(driveController, Button.kA.value)
        .whenHeld(new RunCommand(magazine::magBeltOn, magazine)); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    drive.exampleTrajectory,
    drive::getPoseMeters, 
    drive.kinematics,

    //Position controllers
    new PIDController(Constants.kPXController, 0, 0),
    new PIDController(Constants.kPYController, 0, 0),
    new ProfiledPIDController(Constants.kPThetaController, 0, 0,
                              Constants.kThetaControllerConstraints),
    drive::setModuleStates,
    drive
  );

    return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0));
  }
}