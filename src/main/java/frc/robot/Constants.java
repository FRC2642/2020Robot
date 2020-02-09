/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    /**
     * Please use the following format when creating new constants
     * 
     * public static final [data-type] kVariableName = value;
     * 
     * This kName format will make values imported from Constants easy to identify in other classes.
     * 
     * Thanks!
     */

    /** 
     * IDS FOR CAN MOTORS AND PORTS FOR SOLENOIDS AND SENSORS
     */

      /**
       * CAN IDS
       */
    //CAN IDs for swerve drive and angle motors
    public static final int ID_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int ID_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int ID_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int ID_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int ID_BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int ID_BACK_LEFT_ANGLE_MOTOR = 6;
    public static final int ID_BACK_RIGHT_DRIVE_MOTOR = 7;
    public static final int ID_BACK_RIGHT_ANGLE_MOTOR = 8;
    //CAN ID for mag tilt motor
    public static final int ID_MAG_TILT_MOTOR = 11;
    //CAN ID for Spinner Motor
    public static final int ID_SPINNER_MOTOR = 10;
    //CAN ID for Mag Belt
    public static final int ID_MAG_BELT_MOTOR = 13;
    //CAN IDs for Intake
    public static final int ID_INTAKE_MOTOR = 12;
    //CAN IDs for Shooter
    public static final int ID_RIGHT_SHOOTER_MOTOR = 16;
    public static final int ID_LEFT_SHOOTER_MOTOR = 15;
    //CAN IDs for Hanger
    public static final int ID_CLIMBER_MOTOR = 14;
    //CAN ID for climb bar motor
    public static final int ID_CLIMB_BAR_MOTOR = 9;

      /**
       * SOLENOID PORTS
       */
    //mag piston port 
    public static final int kMagazinePistonPort = 0;
    //intake piston port
    public static final int kIntakePistonPort = 1;
    //color spinner piston port
    public static final int kColorSpinnerPistonPort = 2;

      /**
       * ANALOG
       */
    //sonar
    public static final int kMagazineSonarInput = 0;
    public static final int kMagazineSonarOutput = 1;
    
      /**
       * DIO
       */
    //hanger limit switch
    public static final int khangerLowerLimitSwitch = 0;
    public static final int kArmLimitSwitch = 1;

      /**
       * USB
       */
    //USB Camera
    public static int kUsbCamera = 0;

    /**
     *  CONVERSION FACTORS
     */
    public static final double kAnglePositionConversionFactor = 359.0 / 3.3; //degrees / volts
    public static final double kRPMToMPSConversionFactor = (1.0 / 60) * (4 * Math.PI) * .0254;
    public static final double kDriveVelocityConversionFactor = kRPMToMPSConversionFactor;
    public static final double kRelativeRotationsPerModuleRotation = 18.05; //relative rots
    public static final double kModuleDegreesToRelativeRotations 
                               = kRelativeRotationsPerModuleRotation / 360.0; //rots / degrees
    public static final double kMaxSpeedConversionFactor = 8.0;

    /**
     * ROBOT CONSTANTS
     */
    //distances from robot center (x = length (forward/backward), y = width (left/right))
    public static final double kRobotLength = 0.6858;   //meters
    public static final double kRobotWidth = 0.6858;    //meters
    public static final double kXDistanceFromCenter = kRobotLength / 2;
    public static final double kYDistanceFromCenter = kRobotWidth / 2;
    //absolute encoder offsets (swerve)
    public static final double kFrontLeftAngleOffset = 159.5;
    public static final double kFrontRightAngleOffset = 45.3;
    public static final double kBackLeftAngleOffset = 130.6;
    public static final double kBackRightAngleOffset = 57.8;

    //Dashboard reading offsets (swerve)
    public static final double kFrontLeftAngleDashboardOffset = 318.0;
    public static final double kFrontRightAngleDashboardOffset = 89.6;
    public static final double kBackLeftAngleDashboardOffset = 261.6;
    public static final double kBackRightAngleDashboardOffset = 114.8;


    /**
     * MOTOR CONSTANTS
     */
    //current limit for Spark MAXs 
    public static final int kCurrentLimit = 30; //amps
    //motor neutral deadband
    public static final double kMotorNeutralDeadband = .15;
    //mag belt speed
    public static final double kMagBeltSpeed = 15;
    //swerve max speeds
    public static final double kMaxModuleRPM = 12.0 * kMaxSpeedConversionFactor; //desired module rotation speed * gear ratio conversion
    public static final double kMaxMPS = 12.0 * kMaxSpeedConversionFactor; //desired movement speed * gear ratio conversion

    /**
     * PID GAINS AND OTHER PID CONSTANTS
     */
    //PID constants
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;                                           
    //PIDF values for closed-loop velocity control for drive modules
    public static final double kDriveFF = .5 / 15.989; //approx .03127
    public static final double kDriveP = 0.0;
    public static final double kDriveI = kDriveFF / 2000.0;
    public static final double kDriveD = 0.0;
    //PIDF values for closed-loop position control for angle modules
    public static final double kAngleFF = 0.0;
    public static final double kAngleP = .3;
    public static final double kAngleI = 0.0005;
    public static final double kAngleD = 0.04;
    //PIDF values for closed-loop velocity control for the magazine belt
    public static final double kMagFF = 0.0;
    public static final double kMagP = .3;
    public static final double kMagI = 0.0005;
    public static final double kMagD = 0.04;
    //PIDF values for closed-loop position control for the arm tilt motor
    public static final double kTiltFF = 0.0;
    public static final double kTiltP = .3;
    public static final double kTiltI = 0.0005;
    public static final double kTiltD = 0.04;
    //PIDF values for closed-loop velocity control for the shooter wheels
    public static final double kShooterFF = 0.0;
    public static final double kShooterP = .3;
    public static final double kShooterI = 0.0005;
    public static final double kShooterD = 0.04;
    //PIDF values for closed-loop position control for the climbing motor
    public static final double kClimbFF = 0.0;
    public static final double kClimbP = .3;
    public static final double kClimbI = 0.0005;
    public static final double kClimbD = 0.04;
                               
    /**
     * CONTROLLER PORTS
     */
    //controller ports 
    public static final int kDriveControllerPort = 0;
    public static final int kAuxControllerPort = 1;
}
