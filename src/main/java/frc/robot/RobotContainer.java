/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.colorSpinner.SpinByAmount;
import frc.robot.commands.colorSpinner.SpinToColor;
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
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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

  public final Command intakeCommand = new IntakeCommand(intake, magazine);
  public final Command spinToColor = new SpinToColor(spinner);
  public final Command spinByAmount = new SpinByAmount(spinner);

  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);
  public static Trigger leftTrigger = new Trigger(intake::getLeftTrigger);
  public static Trigger rightTrigger = new Trigger(shooter::getRightTrigger);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();

    drive.setDefaultCommand(new RunCommand(
      () -> drive.drive( // .5, 0, 0),
        -(driveController.getRawAxis(1)) * .7, 
        driveController.getRawAxis(0) * .7, 
        driveController.getRawAxis(4)),
        drive)
      );
    
     arm.setDefaultCommand(
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
        () -> magazine.magIdle()
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
    );

    arm.setDefaultCommand(new RunCommand(
      () -> arm.armLift((auxController.getRawAxis(5) * .5)
     )
    )
  );

    intake.setDefaultCommand(new RunCommand(() -> intake.stop()));

    magazine.setDefaultCommand(new RunCommand(() -> magazine.stop()));

    climb.setDefaultCommand(new RunCommand(
      () -> climb.climbMove((auxController.getRawAxis(1)) *.5, (auxController.getRawAxis(0) * .5))
   )
  );

    spinner.setDefaultCommand(new RunCommand(
      () -> spinner.stop()
    )
   );

    // manually drives motors, leave out unless testing
    /*
     * drive.setDefaultCommand( new RunCommand( () ->
     * drive.motorTest(drive.frontRightModule, -driveController.getRawAxis(1),
     * -driveController.getRawAxis(5)), drive) );
     */
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //instantiates drive toggle button

    //-=+=-DRIVE Controller Buttons-=+=-//

    new JoystickButton(driveController, Button.kBack.value)
      .whenPressed(new InstantCommand(drive::toggleIsDriveFieldCentric));
    //puts arm in highest/climb position
    new JoystickButton(driveController, Button.kY.value)
    .whenPressed(new InstantCommand(arm::armClimbPos));
    //puts arm in midway position
    new JoystickButton(driveController, Button.kB.value)
    .whenPressed(new InstantCommand(arm::armBasePos));
    //puts arm in lowest/trench position
    new JoystickButton(driveController, Button.kA.value)
    .whenPressed(new InstantCommand(arm::armTrenchPos));

//-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-// 
  
    //-=+=-AUX Controller Buttons-=+=-//

    //spins color spinner to certain color
    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(spinToColor.andThen(spinToColor));
    //spins color spinner by set ammount
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(spinByAmount);
    //extends the color spinner
    new JoystickButton(auxController, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(spinner::extend));
    //retracts the color spinner 
    new JoystickButton(auxController, Button.kBumperLeft.value)
    .whenPressed(new InstantCommand(spinner::retract));

    rightTrigger.whileActiveContinuous(intakeCommand);
    //leftTrigger.whileActiveContinuous();
    
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