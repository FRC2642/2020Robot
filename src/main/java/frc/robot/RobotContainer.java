/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.kDriveControllerPort;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.EkatniSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
  public static XboxController auxController = new XboxController(Constants.kAuxControllerPort);
  XboxController driveController = new XboxController(kDriveControllerPort);
  public static final IntakeSubsystem intake = new IntakeSubsystem();
//ekatni is intake backwards, as shooting is the reverse of grabbing
public static final EkatniSubsystem ekatni = new EkatniSubsystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
configureButtonBindings();


  
    
    //basic drive command using left stick for strafe control and right stick for rotate control

    /*drive.setDefaultCommand(
      new RunCommand(
        () -> drive.drive(
          -(driveController.getRawAxis(1)), 
          driveController.getRawAxis(0), 
          driveController.getRawAxis(4)), 
          drive)
      );*/

    /*drive.setDefaultCommand(
      new RunCommand(
        () -> drive.motorTest(drive.frontRightModule,
              -driveController.getRawAxis(1), -driveController.getRawAxis(5)),
              drive)
    );*/
    
   intake.setDefaultCommand(new IntakeCommand(), intake);

     /* drive.setDefaultCommand(
        new RunCommand(
          () -> drive.testDrivePIDFLoop(drive.modules,
           -driveController.getRawAxis(1)),
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
