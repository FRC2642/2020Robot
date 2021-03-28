/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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
    public static final int ID_MAG_TILT_MOTOR = 11; //victor
    //CAN ID for Spinner Motor
    public static final int ID_SPINNER_MOTOR = 10;
    //CAN ID for Mag Belt
    public static final int ID_TOP_MAG_BELT_MOTOR = 13;
    //CAN IDs for Intake
    public static final int ID_INTAKE_MOTOR = 12;
    //CAN IDs for Shooter
    public static final int ID_RIGHT_SHOOTER_MOTOR = 15;
    public static final int ID_LEFT_SHOOTER_MOTOR = 16;
    //CAN IDs for Hanger
    public static final int ID_CLIMBER_MOTOR = 14; //talon
    //CAN ID for climb bar motor
    public static final int ID_BOTTOM_MAG_BELT_MOTOR = 9; 

      /**
       * ANALOG
       */
    //arm potentiometer
    public static final int kArmPotPort = 3;
    
      /**
       * DIO
       */
    //hanger limit switch
    public static final int kClimberLimitSwitch = 0; // not 0
    public static final int kArmLimitSwitch = 1;
    public static final int kColorSpinnerLimitSwitch = 2;

    // Right Sight
    public static final int kRightSight = 3;

      /**
       * USB
       */
    //USB Camera
    public static int kUsbCamera = 0;

          /**
       * SOLENOID PORTS
       */

    //mag piston port 
    public static final int kMagazinePistonPort = 0;
    //intake piston port
    public static final int kIntakePistonPort1 = 1;
    public static final int kIntakePistonPort2 = 2;
    //color spinner piston port
    public static final int kColorSpinnerPistonPort = 3;
    //climb piston port
    public static final int kClimberPistonPort = 4;
     //light ring
    public static final int kLightRing = 7;


    /**
     *  CONVERSION FACTORS
     */

    public static final double kMaxSpeedConversionFactor = 8.3; //gear ratio conversion

    public static final double kAnglePositionConversionFactor = 359.0 / 3.3; //degrees / volts

    public static final double kRPMToMPSConversionFactor = (1.0 / 60) * (4 * Math.PI) * .0254;
    public static final double kDriveVelocityConversionFactor = kRPMToMPSConversionFactor;

    public static final double kRelativeRotationsPerModuleRotation = 17.738054; //18.05; //relative rots 
    public static final double kModuleDegreesToRelativeRotations 
                               = kRelativeRotationsPerModuleRotation / 360.0; //rots / degrees

    public static final double kShooterRPMConversionFactor = 18.84954;

    public static final double kArmAngleConversionFactor = 10.0;

    /**
     * ROBOT CONSTANTS
     */
    //distances from robot center (x = length (forward/backward), y = width (left/right))
    public static final double kRobotLength = 0.889;    //meters, 35 in
    public static final double kRobotWidth = 0.6223;    //meters, 24.5
    public static final double kXDistanceFromCenter = kRobotLength / 2;
    public static final double kYDistanceFromCenter = kRobotWidth / 2;
    //gyro offset
    public static final double kGyroOffset = 0.0;//180.0;
    //Dashboard reading offsets (swerve)
    public static final double kFrontLeftAngleModuleOffset = 67.0;//344.5;
    public static final double kFrontRightAngleModuleOffset = 68.6;//248.6;//124.0;
    public static final double kBackLeftAngleModuleOffset = 120.2;
    public static final double kBackRightAngleModuleOffset = 272.4;

    /**
     * MOTOR CONSTANTS
     */
    //current limit for Spark MAXs 
    public static final int kCurrentLimit = 30; //amps
    //motor neutral deadband
    public static final double kMotorNeutralDeadband = 0.15;
    //swerve max speeds
    public static final double kRealMaxMPS = 12.0; //12
    public static final double kMaxModuleRPM = kRealMaxMPS * kMaxSpeedConversionFactor; //desired module rotation speed * gear ratio conversion
    public static final double kMaxMPS = kRealMaxMPS * kMaxSpeedConversionFactor; //desired movement speed * gear ratio conversion
    public static final double kMaxAcceleration = 1.2192;
    //mag belt speed
    public static final double kMagShortRangeShootSpeed = 5500; //RPM     auto rpm = 2500
    public static final double kMagMidRangeShootSpeed = 3500; //RPM
    public static final double kMagLongRangeShootSpeed = 2600; //RPM
    public static final double kMagLoadSpeed = 3000; //RPM
    public static final double kMagEjectSpeed = 3000; //RPM
    //shooter rpm
    public static final double kShooterDefaultRPM = 1800; //RPM     
    public static final double kShooterInitLineRPM = 1800; //RPM
    public static final double kShooterFrontTrenchRPM = 1800; //RPM
    public static final double kShooterBackTrenchRPM = 4800; //RPM
    public static final double kShooterLongShotRPM = 4500; //RPM
    //tilt presets
    public static final double kArmTrenchRunPos = 0.0;//19.8;
    public static final double kArmStartingPos = 41.1;                            //33.75
    public static final double kArmAutoInitLineShootPos = 31;
    public static final double kArmInitLineShootPos = 28.0;
    public static final double kArmFrontTrenchShootPos = 25.0;
    public static final double kArmBackTrenchShootPos = 22.9;
    public static final double kArmClimbPos = 100.0;//81.4;

    /**
     * PID GAINS AND CONSTANTS AND PROFILING CONSTANTS
     */
    //PID constants
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;                                           
    //PIDF values for closed-loop velocity control for drive modules
    public static final double kDriveFF = .5 / 16.171; 
    public static final double kDriveP = 0.0;
    public static final double kDriveI = kDriveFF / 1500.0;
    public static final double kDriveD = 0.0;
    //PIDF values for closed-loop position control for angle modules
    public static final double kAngleFF = 0.0;
    public static final double kAngleP = 0.4;
    public static final double kAngleI = 0.0002;
    public static final double kAngleD = 0.04;
    //PIDF values for closed-loop velocity control for the magazine belt
    public static final double kMagFF = .25 / 5400;
    public static final double kMagP = 0.0;
    public static final double kMagI = kMagFF / 250;
    public static final double kMagD = 0.0;
    //PIDF values for closed-loop velocity control for the shooter wheels
    public static final double kShooterFF = .34 / 2000.0;            //.36
    public static final double kShooterP = 0.0;
    public static final double kShooterI = 5e-8 * (4);
    public static final double kShooterD = (kShooterI * (2) ) / 5;
    //PID values for profiled closed-loop position control for the arm tilt motor
    public static final double kTiltP = .3;
    public static final double kTiltI = kTiltP / 500;
    public static final double kTiltD = 0.0;
    public static final double kTiltMaxVel = 7; 
    public static final double kTiltMaxAccel = 2;

    /**
     * AUTO TRAJECTORY PID GAINS
     */

    //PID Controllers for auto command
    public static final double kPXController = .3;
    public static final double kPYController = .4;
    public static final double kPThetaController = .5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
    //Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
       kMaxAngularSpeedRadiansPerSecondSquared);

    /**
     * CONTROLLER PORTS
     */
    //controller ports 
    public static final int kDriveControllerPort = 0;
    public static final int kAuxControllerPort = 1;


    //practice auto stuff
    public static final double kDriveDistance = 5.4;
    public static final double kDriveSpeed = 10.77;

}
