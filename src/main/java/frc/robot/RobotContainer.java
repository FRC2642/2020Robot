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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.aimbot.AimbotRotateCommand;
import frc.robot.commands.aimbot.AimbotSpinupCommand;
import frc.robot.commands.aimbot.AimbotTiltCommand;
import frc.robot.commands.aimbot.ShootCommand;
import frc.robot.commands.armTilt.ArmToSetPosition;
import frc.robot.commands.colorSpinner.EndSpinRoutine;
import frc.robot.commands.colorSpinner.PositionControlCommand;
import frc.robot.commands.colorSpinner.RotationControlCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.subsystems.ArmSubsystem;

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
  //public static final ClimberBarSubsystem bar = new ClimberBarSubsystem();
  public static final ArmSubsystem arm = new ArmSubsystem();

  //COMMANDS 
  public final Command intakeCommand = new IntakeCommand(intake, magazine);
  public final Command intakeOutCommand = new IntakeOutCommand(intake, magazine);

  public final Command positionControl = new PositionControlCommand(spinner);
  public final Command rotationControl = new RotationControlCommand(spinner);
  public final Command endSpinRoutine = new EndSpinRoutine(spinner, drive); //empty command atm, needs code
 
  public final Command aimbotRotate = new AimbotRotateCommand(drive);
  public final Command aimbotTilt = new AimbotTiltCommand(arm);
  public final Command aimbotSpinup = new AimbotSpinupCommand(shooter);
  public final Command shoot = new ShootCommand(magazine);

 /*  public final Command armToTrenchPosition = new ArmToTrenchPosition(arm);
  public final Command armToBasePosition = new ArmToBasePosition(arm);
  public final Command armToClimbPosition = new ArmToClimbPosition(arm); */

  public final Command armToTrenchPosition = new ArmToSetPosition(kTrenchPos, arm);
  public final Command armToBasePosition = new ArmToSetPosition(kNormalPos ,arm);
  public final Command armToClimbPosition = new ArmToSetPosition(kClimbPos, arm);

  //CONTROLLERS STUFF
  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);

  public static Trigger leftTrigger = new Trigger(intake::getLeftTrigger);
  public static Trigger rightTrigger = new Trigger(shooter::getRightTrigger);
  public static Trigger auxLeftTrigger = new Trigger(shooter::getLTrigger);
  
  public static SwerveControllerCommand swerveControllerCommandCenter;
  public static SwerveControllerCommand swerveControllerCommandLeft;
  public static SwerveControllerCommand swerveControllerCommandRight1;
  public static SwerveControllerCommand swerveControllerCommandRight2;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();
    drive.setSlowDrive(false);

    //add conditional command that runs either normal or slow based on color piston state
     drive.setDefaultCommand(  
      new RunCommand(
        () -> drive.drive( 
          -(driveController.getRawAxis(1)) * .7, 
          driveController.getRawAxis(0) * .7, 
          driveController.getRawAxis(4) * .7),
          drive)
      ); 
    
     arm.setDefaultCommand(
      new RunCommand(
        () -> arm.moveArm(
          (-auxController.getRawAxis(5) * .75)
       ), arm
      )
    );

    intake.setDefaultCommand(
      new RunCommand(intake::stop, intake)
     );

    magazine.setDefaultCommand(
      new RunCommand(magazine::stopAtIdle, magazine)
    );

    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.climb(-auxController.getRawAxis(1)), climb
      )
    );

    shooter.setDefaultCommand(
      new RunCommand(shooter::stop, shooter)
    );

   /*  bar.setDefaultCommand(
      new RunCommand(
        () -> bar.move(auxController.getRawAxis(0))
      )
    ); */


    spinner.setDefaultCommand(
      new RunCommand(spinner::stop, spinner)
    );
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
/*
    new JoystickButton(driveController, Button.kBumperLeft.value)
      .whenPressed(swerveControllerCommandCenter.andThen(
      (new RunCommand(magazine::magLoad, magazine)), 
      (new RunCommand(intake::intakeIn, intake)),
      (new RunCommand(magazine::magShoot)), 
      (new RunCommand(shooter::shoot))
      ));

    new JoystickButton(driveController, Button.kBack.value)
    .whenPressed(swerveControllerCommandLeft.andThen(
      (new RunCommand(magazine::magLoad, magazine)), 
      (new RunCommand(intake::intakeIn, intake)),
      (new RunCommand(magazine::magShoot)), 
      (new RunCommand(shooter::shoot))
    ));

    new JoystickButton(driveController, Button.kBack.value)
    .whenPressed(new RunCommand(magazine::magShoot).andThen(
      (new RunCommand(shooter::shoot, shooter)),
      (swerveControllerCommandRight1),
      (new RunCommand(magazine::magLoad)), 
      (new RunCommand(intake::intakeIn, intake)),
      (swerveControllerCommandRight2), 
      (new RunCommand(magazine::magShoot)),
      (new RunCommand(shooter::shoot))
    ));
*/
    //toggles field drive and robot drive
    new JoystickButton(driveController, Button.kBack.value)
      .whenPressed(new InstantCommand(drive::toggleDriveFieldCentric));
    //aligns swerve wheels 
    new JoystickButton(driveController, Button.kStart.value)
      .whenHeld(new RunCommand(drive::alignWheels, drive));
    //extends pistons without wheels
    new JoystickButton(driveController, Button.kX.value)
    .whenHeld(new RunCommand(intake::intakePistonOut));
    //puts arm in highest/climb position
    new JoystickButton(driveController, Button.kY.value)
    .whenPressed(armToClimbPosition, true);
    //puts arm in midway position
    new JoystickButton(driveController, Button.kB.value)
    .whenPressed(armToBasePosition, true);
    //puts arm in lowest/trench position
    new JoystickButton(driveController, Button.kA.value)
    .whenPressed(armToTrenchPosition, true);
    //activates aiming mode
   /*  new JoystickButton(driveController, Button.kBumperRight.value)
    .whenHeld(aimbotRotate.alongWith(
      new ConditionalCommand(aimbotTilt, arm.getDefaultCommand(), () -> !arm.isManualOverride()),
      aimbotSpinup
    )); */
    new JoystickButton(driveController, Button.kBumperRight.value)
    .whenHeld(new RunCommand(shooter::setShooterSpeed, shooter));

    new JoystickButton(driveController, Button.kBumperLeft.value)
    .whenHeld(intakeOutCommand);

    //intakes balls
    leftTrigger.whileActiveContinuous(intakeCommand);
    //activates shooting mode
    rightTrigger.whileActiveContinuous(
      new RunCommand(magazine::magEngage, magazine));

      //new RunCommand(drive::lockWheels)
      //.alongWith(shoot));

//-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-// 
  
    //-=+=-AUX Controller Buttons-=+=-//

     //spins color spinner to certain color
    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(positionControl);
    //spins color spinner by set ammount
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(rotationControl);
    //extends the color spinner
    new JoystickButton(auxController, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(spinner::extend)
      .alongWith(
        new FunctionalCommand(
          () -> drive.setSlowDrive(true),
          () -> drive.drive( 
            -(driveController.getRawAxis(1)) * .7, 
            driveController.getRawAxis(0) * .7, 
            driveController.getRawAxis(4) * .7), 
          (interupted) -> drive.doNothing(),
          () -> drive.getIsSlowDrive(), 
          drive)
      ));
    //retracts the color spinner 
    new JoystickButton(auxController, Button.kBumperLeft.value)
    .whenPressed(new InstantCommand(spinner::retract)
      .alongWith(
        new InstantCommand( () -> drive.setSlowDrive(false) )
      ));
    //toggle climb lock
    new JoystickButton(auxController, Button.kStickLeft.value)
    .whenPressed(new InstantCommand(climb::toggleClimbLock));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

/*
    SwerveControllerCommand swerveControllerCommandCenter = new SwerveControllerCommand(
    drive.centerTrajectory,
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

  SwerveControllerCommand swerveControllerCommandLeft = new SwerveControllerCommand(
    drive.leftTrajectory,
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

  SwerveControllerCommand swerveControllerCommandRight1 = new SwerveControllerCommand(
    drive.rightTrajectory1,
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

  SwerveControllerCommand swerveControllerCommandRight2 = new SwerveControllerCommand(
    drive.rightTrajectory2,
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
    return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0));   */
    return null;
  }
}