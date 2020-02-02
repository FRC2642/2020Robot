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
    
    //CAN IDs for Spark Max drive motor controllers
    public static final int ID_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int ID_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int ID_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int ID_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int ID_BACK_LEFT_DRIVE_MOTOR = 5;
    public static final int ID_BACK_LEFT_ANGLE_MOTOR = 6;
    public static final int ID_BACK_RIGHT_DRIVE_MOTOR = 7;
    public static final int ID_BACK_RIGHT_ANGLE_MOTOR = 8;
    public static final int ID_MAG_TILT_MOTOR = 9;
    //sonar
    public static final int kMagazineSonarInput = 19;
    public static final int kMagazineSonarOutput = 20;
    /**
     * PNEUMATICS
     */ 
    //Magazine Pistons (temp values)
    public static final int kLeftMagazinePis = 17;
    public static final int kRightMagazinePis = 18;

    //motor neutral deadband
    public static final double kMotorNeutralDeadband = .15;

    //distances from robot center (x = length (forward/backward), y = width (left/right))
    public static final double kRobotLength = 0.6858;   //meters
    public static final double kRobotWidth = 0.6858;    //meters
    public static final double kXDistanceFromCenter = kRobotLength / 2;
    public static final double kYDistanceFromCenter = kRobotWidth / 2;

    //swerve PID constants
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
    public static final double kMaxRPM = 5700;  //wip values
    public static final double kMaxMPS = 17.5; //MPS

    //conversion factor for drive motor rpm to meters per second
    public static final double kRPMToMPSConversionFactor = (1.0 / 60) * (4 * Math.PI) * .0254; 
                                                        
    //PIDF values for closed-loop velocity control for drive modules
    public static final double kDriveFF = .5 / 15.989; //approx .03127
    public static final double kDriveP = 0.0;
    public static final double kDriveI = kDriveFF / 100.0;
    public static final double kDriveD = 0.0;

    //PIDF values for closed-loop position control for angle modules
    public static final double kAngleP = 0.1;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.0;
    public static final double kAngleFF = 0.0;

    //conversion factors
    public static final double kAnglePositionConversionFactor = 360.0 / 3.3; //degrees / volts
    public static final double kDriveVelocityConversionFactor = kRPMToMPSConversionFactor;

    //controller ports 
    public static final int kDriveControllerPort = 0;
  public static final int kAuxControllerPort = 1;
  
  //sonar port
  public static final int kSonarPort = 0;
     //intake motors
  public static int kIntakeMotorPort1 = 15;
  public static int kIntakeMotorPort2 = 16;
  public static int kEkatniMotorPort1 = 17;
  public static int kEkatniMotorPort2 = 18;
  //intake limit switch
  public static int kIntakeLimitSwitch = 9;
  
  //USB Camera
  public static int kUsbCamera = 23;
  
  //
    /*
    //encoder distance constants
    public static final double kWheelRotationsPerMeterTraveled = 12.57; //wheel rotations
    public static final double kMotorRotationsPerWheelRotation = 8.38; //motor rotations
    public static final double kMotorRotationsPerMeterTraveled = kWheelRotationsPerMeterTraveled * kMotorRotationsPerWheelRotation;  
    public static final double kMetersTraveledPerRotation = 1.0 / kMotorRotationsPerMeterTraveled;

    //conversion factors 
    /**
     * Factor used to convert a target velocity in meters per second into 
     * RPM for a motor
     *
    public static final double kMPSToRPMFactor = 60.0 * kMotorRotationsPerMeterTraveled;
    /**
     * Factor used to convert a target RPM into a velocity in meters per second 
     *
    public static final double kRPMtoMPSFactor = 1.0 / kMPSToRPMFactor; 
    //uses max RPM to find max velocity in meters per second
    public static final double kMaxMPS = kMaxRPM * kRPMtoMPSFactor;
    */
}
