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
    public static final int ID_MAG_BELT_MOTOR = 13;
    //CAN IDs for Intake
    public static final int ID_INTAKE_MOTOR = 12;
    //CAN IDs for Shooter
    public static final int ID_RIGHT_SHOOTER_MOTOR = 16;
    public static final int ID_LEFT_SHOOTER_MOTOR = 15;
    //CAN IDs for Hanger
    public static final int ID_CLIMBER_MOTOR = 14; //victor
    //CAN ID for climb bar motor
    public static final int ID_CLIMB_BAR_MOTOR = 9; //victor

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

      /**
       * ANALOG
       */
      
    
      /**
       * DIO
       */
    //hanger limit switch
    public static final int kClimberLimitSwitch = 2; // not 0
    public static final int kArmLimitSwitch = 1;

    // Right Sight
    public static final int kRightSight = 2;

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
    public static final double kMaxSpeedConversionFactor = 8.31; //gear ratio conversion
    public static final double kShooterRPMConversionFactor = 18.84954;
    //value isn't accurate, change later
   public static final double kArmAngleConversionFactor = 10.0;

    /**
     * ROBOT CONSTANTS
     */
    //distances from robot center (x = length (forward/backward), y = width (left/right))
    public static final double kRobotLength = 0.889;    //meters, 35 in
    public static final double kRobotWidth = 0.6223;    //meters, 24.5
    public static final double kXDistanceFromCenter = kRobotLength / 2;
    public static final double kYDistanceFromCenter = kRobotWidth / 2;

    public static final double kGyroOffset = 180.0;
    //absolute encoder offsets (swerve)
    public static final double kFrontLeftAngleOffset = 159.5;
    public static final double kFrontRightAngleOffset = 45.3;
    public static final double kBackLeftAngleOffset = 130.6;
    public static final double kBackRightAngleOffset = 57.8;

    //Dashboard reading offsets (swerve)
    public static final double kFrontLeftAngleDashboardOffset = 212.5;
    public static final double kFrontRightAngleDashboardOffset = 73.0;
    public static final double kBackLeftAngleDashboardOffset = 185.0;
    public static final double kBackRightAngleDashboardOffset = 33.0;


    /**
     * MOTOR CONSTANTS
     */
    //current limit for Spark MAXs 
    public static final int kCurrentLimit = 30; //amps
    //motor neutral deadband
    public static final double kMotorNeutralDeadband = .15;
    //mag belt speed
    public static final double kMagShootSpeed = 6500;
    public static final double kMagLoadSpeed = 5000;
    public static final double kMagIdleSpeed = 2000;
    //swerve max speeds
    public static final double kMaxModuleRPM = 12.0 * kMaxSpeedConversionFactor; //desired module rotation speed * gear ratio conversion
    public static final double kMaxMPS = 12.0 * kMaxSpeedConversionFactor; //desired movement speed * gear ratio conversion
    public static final double kMaxAcceleration = 1.2192;
    //shooter rpm
    public static final double kShooterRPM = 2500; //RPM
  
    /**
     * ENCODER SOFT LIMITS
     */

    public static final double kClimberUpperLimit = 3000.0; //TEMP VAL
    public static final double kClimberLowerLimit = 0.0; //TEMP VAL


    /**
     * VISION CONSTANTS
     */
  
     //light ring
  public static final int kLightRing = 7;

     //JeVois Camera Number
	public static final int kJevoisCamNumber = 1; //JEVOIS_CAM_NUMBER
	// Serial Port Constants 
	public static final int kBaudRate = 115200; //BAUD_RATE
	// MJPG Streaming Constants 
	public static final int kMjpgStreamPort = 1180; //MJPG_STREAM_PORT
	// JeVois Program Selection Constants - must align with JeVois .cfg files
	public static final int kPixleWidth1 = 320; //MAPPING_WIDTH_PXL_1
	public static final int kPixleHeight1 = 240; //MAPPING_HEIGHT_PXL_1
	public static final int kFPS1 = 20; //MAPPING_FRMRT_FPS_1
	// JeVois Program Selection Constants - must align with JeVois .cfg files
	public static final int kPixleWidth2 = 320; //MAPPING_WIDTH_PXL_2
	public static final int kPixleHeight2 = 240; //MAPPING_HEIGHT_PXL_2
	public static final int kFPS2 = 20; //MAPPING_FRMRT_FPS_2
	// Packet format constants (how the string is sent from the JeVois)
	public static final String kPacketStart = "{"; //PACKET_START_CHAR
	public static final String kPacketEnd = "}"; //PACKET_END_CHAR
	public static final String kPacketSpacer = ","; //PACKET_DILEM_CHAR
	// Status variables 
	public static boolean camStreamRunning = false;
    public static  boolean trackingOnline = false;
    public static  boolean trackingEnable = true;
    public static  boolean serOutEnable = false;
	// Most recently seen target 
	public static double trk;			//how many targets 
    public static double xCntr;			//x coordinate of target center
	public static double yCntr;			//y coordinate of target center
    public static double camMode;		//Camera mode either vision processing, driver mode, or another vision processing mode

    /**
     * PID GAINS AND OTHER PID CONSTANTS
     */
    //PID constants
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;                                           
    //PIDF values for closed-loop velocity control for drive modules
    public static final double kDriveFF = .5 / 16.171; //approx .03127
    public static final double kDriveP = 0.0;
    public static final double kDriveI = kDriveFF / 4000.0;
    public static final double kDriveD = 0.0;
    //PIDF values for closed-loop position control for angle modules
    public static final double kAngleFF = 0.0;
    public static final double kAngleP = 0.4;
    public static final double kAngleI = 0.0002;
    public static final double kAngleD = 0.04;
    //PIDF values for closed-loop velocity control for the magazine belt
    public static final double kMagFF = .5 / 5400;
    public static final double kMagP = 0.0;
    public static final double kMagI = 0.0;
    public static final double kMagD = 0.0;
    //PIDF values for closed-loop position control for the arm tilt motor
    public static final double kTiltFF = 0.0;
    public static final double kTiltP = 0.0;
    public static final double kTiltI = 0.00;
    public static final double kTiltD = 0.0;
    //PIDF values for closed-loop velocity control for the shooter wheels
    public static final double kShooterFF = .5 / 2700.0;
    public static final double kShooterP = 0.0;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    //PIDF values for closed-loop position control for the climbing motor
    public static final double kClimbFF = 0.0;
    public static final double kClimbP = 0.0;
    public static final double kClimbI = 0.0;
    public static final double kClimbD = 0.0;
    //PIDF values for initial alignment of the wheels
    public static final double kAlignmentFF = 0.0;
    public static final double kAlignmentP = 0.0;
    public static final double kAlignmentI = 0.0;
    public static final double kAlignmentD = 0.0;
    
  /**
     * CONTROLLER PORTS
     */
    //controller ports 
    public static final int kDriveControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    //PID Controllers for auto command
    public static final double kPXController = .3;
    public static final double kPYController = .4;
    public static final double kPThetaController = .5;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
 

     //Constraint for the motion profilied robot angle controller
     public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
     new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
       kMaxAngularSpeedRadiansPerSecondSquared);
}

