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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.EkatniSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorSpinnerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  //ekatni is intake backwards, as shooting is the reverse of grabbing
  //get rid of this -__- -dylan
  public static final EkatniSubsystem ekatni = new EkatniSubsystem();
  public final ColorSpinnerSubsystem spinner = new ColorSpinnerSubsystem();
  public final MagazineSubsystem magazine = new MagazineSubsystem();

  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);

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

    intake.setDefaultCommand(new IntakeCommand(), intake);
    
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
      .whenPressed(new InstantCommand(drive::toggleIsDriveFieldCentric, drive));
    //toggles aiming mode
    new JoystickButton(driveController, Button.kStart.value)
      .whenPressed(new InstantCommand(drive::toggleIsAimingMode, drive));
    //rotates colorspinner motor left/Counter Clockwise
    new JoystickButton(auxController, Button.kX.value)
      .whenHeld(new RunCommand(spinner::spinL, spinner));
    //rotates colorspinner motor right/Clockwise
    new JoystickButton(auxController, Button.kB.value)
      .whenHeld(new RunCommand(spinner::spinR, spinner));
    //magazine belt goes forward
    new JoystickButton(auxController, Axis.kRightTrigger.value)
      .whenHeld(new RunCommand(magazine::magBeltForward, magazine));
    //magazine belt goes backward
      new JoystickButton(auxController, Button.kBumperRight.value)
        .whenPressed(new RunCommand(magazine::magBeltBackward, magazine));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
