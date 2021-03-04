/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.aimbot.AimbotRotateCommand;
import frc.robot.commands.aimbot.AimbotSpinupCommand;
import frc.robot.commands.aimbot.AimbotTiltCommand;
import frc.robot.commands.armTilt.ArmToSetPosition;
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
import frc.robot.subsystems.MagazineSubsystem.ShootMode;



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
  public static final ArmSubsystem arm = new ArmSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem();
  public static final ColorSpinnerSubsystem spinner = new ColorSpinnerSubsystem();
  public static final ClimberSubsystem climb = new ClimberSubsystem();

  //COMMANDS 
  public final Command intakeCommand = new IntakeCommand(intake, magazine);
  public final Command intakeOutCommand = new IntakeOutCommand(intake, magazine);

  public final Command positionControl = new PositionControlCommand(spinner);
  public final Command rotationControl = new RotationControlCommand(spinner);
 
  public final Command aimbotRotate = new AimbotRotateCommand(drive);
  public final Command aimbotTilt = new AimbotTiltCommand(arm);
  public final Command aimbotSpinup = new AimbotSpinupCommand(shooter);

  public final Command armToStartingPosition = new ArmToSetPosition(kArmStartingPos, arm);

  public final Command armToTrenchPosition = new ArmToSetPosition(kArmTrenchRunPos, arm);
  public final Command armToInitLineShootPosition = new ArmToSetPosition(kArmInitLineShootPos, arm);
  public final Command armToFrontTrenchShootPosition = new ArmToSetPosition(kArmFrontTrenchShootPos, arm);
  public final Command armToBackTrenchShootPosition = new ArmToSetPosition(kArmBackTrenchShootPos, arm);
  public final Command armToClimbPosition = new ArmToSetPosition(kArmClimbPos, arm);

  public final Command autoArmInitLineShootPosition = new ArmToSetPosition(kArmAutoInitLineShootPos, arm);
  public final Command autoArmToIntakePosition = new ArmToSetPosition(kArmTrenchRunPos, arm);

  //CONTROLLERS STUFF
  public static XboxController driveController = new XboxController(kDriveControllerPort);
  public static XboxController auxController = new XboxController(kAuxControllerPort);

  public static Trigger leftTrigger = new Trigger(intake::getLeftTrigger);
  public static Trigger rightTrigger = new Trigger(magazine::getRightTrigger);


  public static Trigger auxLeftTrigger = new Trigger(shooter::getLTrigger);
  public static Trigger auxRightTrigger = new Trigger(shooter::getRTrigger);
  public static Trigger auxLDPad = new Trigger(spinner::getLDPad);
  public static Trigger auxRDPad = new Trigger(spinner::getRDPad);
  public static Trigger auxUpDPad = new Trigger(arm::getUpDPad);
  public static Trigger auxDownDPad = new Trigger(arm::getDownDPad);
  
  public static SwerveControllerCommand swerveControllerCommandCenter;
  public static SwerveControllerCommand swerveControllerCommandLeft;
  public static SwerveControllerCommand swerveControllerCommandRight1;
  public static SwerveControllerCommand swerveControllerCommandRight2;
  public static SwerveControllerCommand swerveControllerCommandExample;

  public boolean ticksAt5Feet;
  //actual swerve controller command
  public static SwerveControllerCommand swervecontrollercommand;

  public TrajectoryConfig config;
  public Trajectory autonav3;
  //public double ticks;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();
    drive.setSlowDrive(false);

    //add conditional command that runs either normal or slow based on color piston state
    drive.setDefaultCommand(  
      new RunCommand(
        () -> drive.drive( //-.15, 0, 0),
          -(driveController.getRawAxis(1)) * .5, 
          driveController.getRawAxis(0) * .5, 
          driveController.getRawAxis(4) * .5),
          drive)
      );  

    arm.setDefaultCommand(
      new RunCommand(
        () -> arm.moveArm(
          (-auxController.getRawAxis(5) )
       ), arm
      )
    );

    intake.setDefaultCommand(
      new RunCommand(intake::stop, intake)
     );

    magazine.setDefaultCommand(
      new RunCommand(magazine::setToIdleState, magazine)
    );

    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.climb(-auxController.getRawAxis(1)), climb
      )
    );

    shooter.setDefaultCommand(
      new RunCommand(shooter::stop, shooter)
    );

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

    //-=+=-DRIVE Controller Buttons-=+=-//

    /**
     * swerve stuff
     */
    //toggles field drive and robot drive
    new JoystickButton(driveController, Button.kBack.value)
      .whenPressed(new InstantCommand(drive::toggleDriveFieldCentric));
    //aligns swerve wheels 
    new JoystickButton(driveController, Button.kStart.value)
      .whenPressed(new InstantCommand(drive::alignWheels, drive));

    /**
     * intake buttons
     */
    //extends pistons without wheels
    new JoystickButton(driveController, Button.kX.value)
    .whenHeld(new RunCommand(intake::intakePistonOut));
    //ejects balls
    new JoystickButton(driveController, Button.kBumperLeft.value)
    .whenHeld(intakeOutCommand);
    //intakes balls
    leftTrigger.whileActiveContinuous(intakeCommand);

    /**
     * shooting buttons
     */
    new JoystickButton(driveController, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(magazine::toggleIdleState, magazine));

    //activates shooting mode
      rightTrigger.whileActiveContinuous(
      new RunCommand(magazine::setToShootingStateShortRange, magazine));

//-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-=+=-// 
  
    //-=+=-AUX Controller Buttons-=+=-//

    /**
     * presets 
     */
    
    //preset for shooting at init line position
    new JoystickButton(auxController, Button.kY.value)
     .whileHeld(new RunCommand(
      () -> shooter.setShooterSpeed(kShooterInitLineRPM), shooter)
      .alongWith( 
        armToInitLineShootPosition,
        new RunCommand(
          () -> magazine.setShootMode(ShootMode.SHORT), magazine
          )
        ), true
      ); 

    //preset for shooting in front trench position
    new JoystickButton(auxController, Button.kB.value)
      .whileHeld(new RunCommand(
       () -> shooter.setShooterSpeed(kShooterFrontTrenchRPM), shooter)
       .alongWith(
         armToFrontTrenchShootPosition,
         new RunCommand(
          () -> magazine.setShootMode(ShootMode.MID), magazine
          )
         )); 

    //preset for shooting at back trench position
    new JoystickButton(auxController, Button.kA.value)
     .whileHeld(new RunCommand(
      () -> shooter.setShooterSpeed(kShooterBackTrenchRPM), shooter)
      .alongWith( 
        armToBackTrenchShootPosition,
        new RunCommand(
          () -> magazine.setShootMode(ShootMode.LONG), magazine
          )
        )
      ); 

    /* new JoystickButton(auxController, Button.kX.value)
     .whileHeld(new RunCommand(
       () -> shooter.setShooterSpeed(kShooterLongShotRPM)
     ));  */

    //spin up shooter at arbitrary speed
    auxRightTrigger.whileActiveContinuous(new RunCommand(shooter::setShooterSpeed, shooter));

    /**
     * color spinner buttons
     */
    //extends the color spinner
    new JoystickButton(auxController, Button.kBumperRight.value)
    .whenPressed(new InstantCommand(spinner::extend)
    );

    //retracts the color spinner 
    new JoystickButton(auxController, Button.kBumperLeft.value)
    .whenPressed(new InstantCommand(spinner::retract)
      .alongWith(
        new InstantCommand( () -> drive.setSlowDrive(false) )
      ));

    //activates rotation control
    auxLDPad.whenActive(rotationControl);

    //activates position control
    auxRDPad.whenActive(positionControl);

    /**
     * climb buttons
     */
    //move arm to climb position
    auxUpDPad.whenActive(armToClimbPosition);

    //toggle climb lock
    new JoystickButton(auxController, Button.kStickLeft.value)
    .whenPressed(new InstantCommand(climb::toggleClimbLock));

    /**
     * starting position button (pit settings)
     */
    new JoystickButton(auxController, Button.kBack.value)
    .whenPressed(armToStartingPosition);

    
    new JoystickButton(auxController, Button.kStart.value)
      .whenPressed(new InstantCommand(drive::resetPose, drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    Command auto = 

        
    new WaitCommand(5.0)
          .deadlineWith(
            new RunCommand(
              () -> drive.drive(-.3, 0.0, 0.0), drive)
          );  
    
    /*if (drive.getDrivePosition() < 41.3) {
      new WaitCommand(.25)
      .andThen(new WaitCommand(10.0)
        .deadlineWith(
          new RunCommand(
          () -> drive.drive(-0.3, 0.0, 0.0), drive)
                )  
        ).andThen(
          new RunCommand(
          () -> drive.drive(0.0, 0.0, 0.0), drive
                )); 
    }
    else if (drive.getDrivePosition() >= 41.3) {
      drive.stop(); 
    }*/

    //new RunCommand(() -> drive.drive(0.3, 0.0,0.0));
    
      /*//begin with(
          new WaitUntilCommand(
            () -> magazine.isMagReadyToShoot()
          )
        .deadlineWith(
            autoArmInitLineShootPosition,
            new RunCommand(
              () -> shooter.setShooterSpeed(1600), shooter
              )
    ).andThen( 
        new WaitCommand(3)
          .deadlineWith(
            new RunCommand(
              () -> magazine.setToShootingStateShortRange(), magazine
          )
        )
    ).andThen(
        new InstantCommand(
          () -> drive.alignWheels(), drive
        )
    ).andThen(
        new WaitCommand(.25)
    ).andThen(
        new WaitCommand(.5)
          .deadlineWith(
            new RunCommand(
              () -> drive.drive(-.3, 0.0, 0.0), drive)
          )  
    ).andThen(
          new RunCommand(
            () -> drive.drive(0.0, 0.0, 0.0), drive
            )
    ).andThen(
        autoArmToIntakePosition
    ); */

       
    
  
   
    //new RunCommand(
     // () -> drive.drive(.6, 0.0, 0.0), drive);

  //return auto.andThen(() -> drive.drive(0.0,0.0,0.0));







  /*new WaitUntilCommand(
    () -> magazine.isMagReadyToShoot()
  )
 .deadlineWith(
    autoArmInitLineShootPosition,
    new RunCommand(
      () -> shooter.setShooterSpeed(1600), shooter
      )
).andThen( 
new WaitCommand(3)
  .deadlineWith(
    new RunCommand(
      () -> magazine.setToShootingStateShortRange(), magazine
  )
)
).andThen(
new InstantCommand(
  () -> drive.alignWheels(), drive
)
).andThen(
new WaitCommand(.25)
).andThen(
new WaitCommand(.5)
  .deadlineWith(
    new RunCommand(
      () -> drive.drive(-.3, 0.0, 0.0), drive)
  )  
).andThen(
  new RunCommand(
    () -> drive.drive(0.0, 0.0, 0.0), drive
    )
).andThen(
autoArmToIntakePosition
); */
return auto;
}
    /*TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxMPS, Constants.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.kinematics);
      
      config.setReversed(true);

        /*Trajectory autonav3 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),*/
          /*List.of(
                new Translation2d(1, 4),
                //new Rotation2d(0.242535625036333, 0.970142500145332),
                new Translation2d(1.5, -5.5),
                //new Rotation2d(0.263117405792109, -0.964763821237732),
                new Translation2d(4, -4.5),
                //new Rotation2d(0.66436383882992, -0.74740931868366),
                new Translation2d(2, 10.5),
                //new Rotation2d(0.187112107889995, 0.982338566422475),
                new Translation2d(1.5, -9.5),
                //new Rotation2d(0.155962573473011, -0.987762965329069),
                new Translation2d(5, -0.5),
                //new Rotation2d(0.995037190209989, -0.0995037190209989),
                new Translation2d(1, 9.5),
               // new Rotation2d(0.104684784518043, 0.994505452921406),
                new Translation2d(5.5, -5),
               // new Rotation2d(0.739940073395944, -0.672672793996312),
                new Pose2d(21.5, -11.5, Rotation2d.fromDegrees(0)),
          config));*/
          /*Trajectory autonav3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
              new Translation2d(1, 4),
              //new Rotation2d(0.242535625036333, 0.970142500145332),
              new Translation2d(1.5, -5.5),
              //new Rotation2d(0.263117405792109, -0.964763821237732),
              new Translation2d(4, -4.5),
              //new Rotation2d(0.66436383882992, -0.74740931868366),
              new Translation2d(2, 10.5),
              //new Rotation2d(0.187112107889995, 0.982338566422475),
              new Translation2d(1.5, -9.5),
              //new Rotation2d(0.155962573473011, -0.987762965329069),
              new Translation2d(5, -0.5),
              //new Rotation2d(0.995037190209989, -0.0995037190209989),
              new Translation2d(1, 9.5),
             // new Rotation2d(0.104684784518043, 0.994505452921406),
              new Translation2d(5.5, -5)),
             // new Rotation2d(0.739940073395944, -0.672672793996312),
              new Pose2d(21.5, -11.5, Rotation2d.fromDegrees(0)),
            config);*/
    /* Command auto =
        new InstantCommand(
          () -> RobotContainer.drive.alignWheels()
        )*/
    //.andThen( 
        /*SwerveControllerCommand swervecontrollercommand = new SwerveControllerCommand(
          autonav3,
          drive::getPose, 
          drive.kinematics,
          //Position controllers
          new PIDController(1.0, 0.4, 0),
          new PIDController(1.0, 0.4, 0),
          new ProfiledPIDController(1.0, 0.4, 0,
                              Constants.kThetaControllerConstraints),
          drive::setModuleStates,
          drive
        );
    //); 
    return swervecontrollercommand.andThen(() -> drive.drive(0.0,0.0,0.0));
      //return auto; */
  //} 

  }

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

 

  


        /* .alongWith(
          autoArmInitLineShootPosition,
          new RunCommand(
            () -> shooter.setShooterSpeed(kShooterInitLineRPM)
          )
        ) 

        /* TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxMPS, Constants.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.kinematics);
      
      config.setReversed(true);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          List.of(
              new Translation2d(-4.0, 4.0),
              new Translation2d(-8.0, 0.0)
              ),
          
              new Pose2d(-12.0, 4.0, Rotation2d.fromDegrees(0)),
          config);

    Command auto =
        new InstantCommand(
          () -> RobotContainer.drive.alignWheels()
        )
    .andThen( 
        new SwerveControllerCommand(
          exampleTrajectory,
          drive::getPose, 
          drive.kinematics,
          //Position controllers
          new PIDController(1.0, 0.4, 0),
          new PIDController(1.0, 0.4, 0),
          new ProfiledPIDController(1.0, 0.4, 0,
                              Constants.kThetaControllerConstraints),
          drive::setModuleStates,
          drive
        )
    ); */
    
      //return auto;
  


/*
    new JoystickButton(driveController, Button.kBumperLeft.value)
      .whenPressed(swerveControllerCommandCenter.andThen(
      (new RunCommand(magazine::magLoad, magazine)), 
      (new RunCommand(intake::intakeIn, intake)),
      (new RunCommand(magazine::magShoot)), 
      (new RunCommand(shooter::
      shoot))
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