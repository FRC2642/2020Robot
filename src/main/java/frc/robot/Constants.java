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
     * MOTOR IDS AND SENSOR PORTS
     */
    //CAN IDs for Spark Max drive motor controllers
    public static final int ID_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int ID_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int ID_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int ID_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int ID_BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int ID_BACK_LEFT_ANGLE_MOTOR = 6;
    public static final int ID_BACK_RIGHT_DRIVE_MOTOR = 7;
    public static final int ID_BACK_RIGHT_ANGLE_MOTOR = 8;

    //CAN IDs for Spinner Motor
    public static final int ID_SPINNER_MOTOR = 9;
    /**
     * MOTOR CONSTANTS
     */
    //angle offsets
    public static final double kFrontLeftAngleOffset = 157.0;
    public static final double kFrontRightAngleOffset = 38.3;//90;//226.3;
    public static final double kBackLeftAngleOffset = 130.6;//90;//134.4;
    public static final double kBackRightAngleOffset = 57.1;//90;//242.5;

    //current limit for Spark MAXs 
    public static final int kCurrentLimit = 30; //amps

    //motor neutral deadband
    public static final double kMotorNeutralDeadband = .15;

    /**
     * PID GAINS AND OTHER PID CONSTANTS
     */
    //swerve PID constants
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
    public static final double kMaxModuleRPM = 17.5;  //wip, MPS
    public static final double kMaxMPS = 17.5; //slowly bring up to 25 MPS
                                                            
    //PIDF values for closed-loop velocity control for drive modules
    public static final double kDriveFF = .5 / 15.989; //approx .03127
    public static final double kDriveP = 0.0;
    public static final double kDriveI = kDriveFF / 2000.0;
    public static final double kDriveD = 0.0;

    //PIDF values for closed-loop position control for angle modules
    public static final double kAngleP = .01;
    public static final double kAngleI = 1e-8;
    public static final double kAngleD = 0.0;
    public static final double kAngleFF = 0.0;

    /**
     * ROBOT CONSTANTS, CONVERSION FACTORS, ETC
     */
    //distances from robot center (x = length (forward/backward), y = width (left/right))
    public static final double kRobotLength = 0.6858;   //meters
    public static final double kRobotWidth = 0.6858;    //meters
    public static final double kXDistanceFromCenter = kRobotLength / 2;
    public static final double kYDistanceFromCenter = kRobotWidth / 2;

    //conversion factors
    public static final double kAnglePositionConversionFactor = 359.0 / 3.3; //degrees / volts
    public static final double kRPMToMPSConversionFactor = (1.0 / 60) * (4 * Math.PI) * .0254;
    public static final double kDriveVelocityConversionFactor = kRPMToMPSConversionFactor;
    public static final double kRelativeRotationsPerModuleRotation = 18.05; //relative rots
    public static final double kModuleDegreesToRelativeRotations 
                               = kRelativeRotationsPerModuleRotation / 360.0; //rots / degrees

    /**
     * CONTROLLER PORTS
     */
    //controller ports 
    public static final int kDriveControllerPort = 0;
    public static final int kAuxControllerPort = 1;
}
