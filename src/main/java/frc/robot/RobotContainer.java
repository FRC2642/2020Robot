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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.aimbot.AimbotRotateCommand;
import frc.robot.commands.aimbot.AimbotSpinupCommand;
import frc.robot.commands.aimbot.AimbotTiltCommand;
import frc.robot.commands.aimbot.ShootCommand;
import frc.robot.commands.colorSpinner.EndSpinRoutine;
import frc.robot.commands.colorSpinner.spinByAmount;
import frc.robot.commands.colorSpinner.SpinToColor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberBarSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSpinnerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

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
  //public static final ShooterSubsystem shooter = new ShooterSubsystem();
  public static final ColorSpinnerSubsystem spinner = new ColorSpinnerSubsystem();
  //public static final ClimberSubsystem climb = new ClimberSubsystem();
  //public static final ClimberBarSubsystem bar = new ClimberBarSubsystem();
  public static final ArmSubsystem arm = new ArmSubsystem();

  //public final Command intakeCommand = new IntakeCommand(intake, magazine);
  public final Command spinToColor = new SpinToColor(spinner);
  public final Command spinByAmount = new spinByAmount(spinner);
  public final Command endSpinRoutine = new EndSpinRoutine(spinner, drive); //empty command atm, needs code
 
  public final Command aimbotRotate = new AimbotRotateCommand(drive);
  public final Command aimbotTilt = new AimbotTiltCommand(arm);
  //public final Command aimbotSpinup = new AimbotSpinupCommand(shooter);
  //public final Command shoot = new ShootCommand(magazine);

  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);

  public static Trigger leftTrigger = new Trigger(intake::getLeftTrigger);
  //public static Trigger rightTrigger = new Trigger(shooter::getRightTrigger);
  //public static Trigger auxLeftTrigger = new Trigger(shooter::getLTrigger);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();

    drive.setDefaultCommand(new RunCommand(
      () -> drive.drive( // .5, 0, 0),
        -(driveController.getRawAxis(1)) * .7, 
        driveController.getRawAxis(0) * .7, 
        driveController.getRawAxis(4) * .7),
        drive)
      );
    
     arm.setDefaultCommand(
      new RunCommand(
        () -> arm.moveArm(
          (-auxController.getRawAxis(5) * .5)
       ), arm
      )
    );

    intake.setDefaultCommand(
      new RunCommand(intake::stop, intake)
     );

   /*  magazine.setDefaultCommand(
      new RunCommand(magazine::magDisengage, magazine)
    ); */

    /*climb.setDefaultCommand(
      new RunCommand(
        () -> climb.climb(-auxController.getRawAxis(1)), climb
      )
    );/*

    /* shooter.setDefaultCommand(
      new RunCommand(shooter::stop, shooter)
    ); */

   /*  bar.setDefaultCommand(
      new RunCommand(
        () -> bar.move(auxController.getRawAxis(0))
      )
    ); */

    spinner.setDefaultCommand(
      new RunCommand(spinner::stop, spinner)
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

    //toggles field drive and robot drive
    new JoystickButton(driveController, Button.kBack.value)
      .whenPressed(new InstantCommand(drive::toggleIsDriveFieldCentric));
    new JoystickButton(driveController, Button.kStart.value)
      .whenHeld(new RunCommand(drive::alignWheels, drive));
    //puts arm in highest/climb position
    new JoystickButton(driveController, Button.kY.value)
    .whenPressed(new InstantCommand(arm::armClimbPos));
    //puts arm in midway position
    new JoystickButton(driveController, Button.kB.value)
    .whenPressed(new InstantCommand(arm::armBasePos));
    //puts arm in lowest/trench position
    new JoystickButton(driveController, Button.kA.value)
    .whenPressed(new InstantCommand(arm::armTrenchPos));
    //activates aiming mode
   /*  new JoystickButton(driveController, Button.kBumperRight.value)
    .whenHeld(aimbotRotate.alongWith(
      new ConditionalCommand(aimbotTilt, arm.getDefaultCommand(), () -> !arm.isManualOverride()),
      aimbotSpinup
    )); */
    /* new JoystickButton(driveController, Button.kBumperRight.value)
    .whenHeld(new RunCommand(shooter::setShooterSpeed, shooter));
 */

    //intakes balls
    //leftTrigger.whileActiveContinuous(intakeCommand);
    //activates shooting mode
   /*  rightTrigger.whileActiveContinuous(
      new RunCommand(magazine::magEngage, magazine));
 */
      //new RunCommand(drive::lockWheels)
      //.alongWith(shoot));

//-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-// 
  
    //-=+=-AUX Controller Buttons-=+=-//

     //spins color spinner to certain color
    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(spinToColor);//.andThen(endSpinRoutine));
    //spins color spinner by set ammount
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(spinByAmount);//.andThen(endSpinRoutine));
    //extends the color spinner
    new JoystickButton(auxController, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(spinner::extend));
    //retracts the color spinner 
    new JoystickButton(auxController, Button.kBumperLeft.value)
    .whenPressed(new InstantCommand(spinner::retract));
 

    //auxLeftTrigger.whileActiveContinuous(new RunCommand(() -> shooter.setShooterSpeed(kShooterRPM)));
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