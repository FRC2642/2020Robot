/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.ID_BACK_LEFT_ANGLE_MOTOR;
import static frc.robot.Constants.ID_BACK_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.ID_BACK_RIGHT_ANGLE_MOTOR;
import static frc.robot.Constants.ID_BACK_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.ID_FRONT_LEFT_ANGLE_MOTOR;
import static frc.robot.Constants.ID_FRONT_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.ID_FRONT_RIGHT_ANGLE_MOTOR;
import static frc.robot.Constants.ID_FRONT_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.kBackLeftAngleDashboardOffset;
import static frc.robot.Constants.kBackLeftAngleOffset;
import static frc.robot.Constants.kBackRightAngleDashboardOffset;
import static frc.robot.Constants.kBackRightAngleOffset;
import static frc.robot.Constants.kCurrentLimit;
import static frc.robot.Constants.kFrontLeftAngleDashboardOffset;
import static frc.robot.Constants.kFrontLeftAngleOffset;
import static frc.robot.Constants.kFrontRightAngleDashboardOffset;
import static frc.robot.Constants.kFrontRightAngleOffset;
import static frc.robot.Constants.kMaxMPS;
import static frc.robot.Constants.kMaxModuleRPM;
import static frc.robot.Constants.kMotorNeutralDeadband;
import static frc.robot.Constants.kXDistanceFromCenter;
import static frc.robot.Constants.kYDistanceFromCenter;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import static frc.robot.Constants.*;



public class SwerveDriveSubsystem extends SubsystemBase {
  CANSparkMax frontLeftDriveMotor, frontLeftAngleMotor;
  CANSparkMax frontRightDriveMotor, frontRightAngleMotor;
  CANSparkMax backLeftDriveMotor, backLeftAngleMotor;
  CANSparkMax backRightDriveMotor, backRightAngleMotor;

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;
  public List<SwerveModule> modules;
  SwerveModuleState[] moduleStates;

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;

  public AHRS navx;

  public boolean isDriveFieldCentric;
  public boolean areWheelsLocked;
 

  /**
   * Creates a new SwerveDriveSubsystem.
   */
  public SwerveDriveSubsystem() {
    //instantiates all 8 module motors
    frontLeftDriveMotor = new CANSparkMax(ID_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
    frontLeftAngleMotor = new CANSparkMax(ID_FRONT_LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    frontRightDriveMotor = new CANSparkMax(ID_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
    frontRightAngleMotor = new CANSparkMax(ID_FRONT_RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    backLeftDriveMotor = new CANSparkMax(ID_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
    backLeftAngleMotor = new CANSparkMax(ID_BACK_LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    backRightDriveMotor = new CANSparkMax(ID_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
    backRightAngleMotor = new CANSparkMax(ID_BACK_RIGHT_ANGLE_MOTOR, MotorType.kBrushless);

    //sets motor settings in a known state
    frontLeftDriveMotor.restoreFactoryDefaults();
    frontLeftAngleMotor.restoreFactoryDefaults();
    frontRightDriveMotor.restoreFactoryDefaults();
    frontRightAngleMotor.restoreFactoryDefaults();
    backLeftDriveMotor.restoreFactoryDefaults();
    backLeftAngleMotor.restoreFactoryDefaults();
    backRightDriveMotor.restoreFactoryDefaults();
    backRightAngleMotor.restoreFactoryDefaults(); 

    //sets default inversion settings for motors
    frontLeftDriveMotor.setInverted(true);
    frontLeftAngleMotor.setInverted(true);
    frontRightDriveMotor.setInverted(true);
    frontRightAngleMotor.setInverted(true);
    backLeftDriveMotor.setInverted(false);
    backLeftAngleMotor.setInverted(true);
    backRightDriveMotor.setInverted(true);
    backRightAngleMotor.setInverted(true); 

    //sets current limits 
    frontLeftDriveMotor.setSmartCurrentLimit(kCurrentLimit);
    frontLeftAngleMotor.setSmartCurrentLimit(kCurrentLimit);
    frontRightDriveMotor.setSmartCurrentLimit(kCurrentLimit);
    frontRightAngleMotor.setSmartCurrentLimit(kCurrentLimit);
    backLeftDriveMotor.setSmartCurrentLimit(kCurrentLimit);
    backLeftAngleMotor.setSmartCurrentLimit(kCurrentLimit);
    backRightDriveMotor.setSmartCurrentLimit(kCurrentLimit);
    backRightAngleMotor.setSmartCurrentLimit(kCurrentLimit);

    //assigns drive and angle motors to their respective swerve modules
    frontLeftModule = new SwerveModule(frontLeftDriveMotor, frontLeftAngleMotor, kFrontLeftAngleOffset, kFrontLeftAngleDashboardOffset);
    frontRightModule = new SwerveModule(frontRightDriveMotor, frontRightAngleMotor, kFrontRightAngleOffset, kFrontRightAngleDashboardOffset);
    backLeftModule = new SwerveModule(backLeftDriveMotor, backLeftAngleMotor, kBackLeftAngleOffset, kBackLeftAngleDashboardOffset);
    backRightModule = new SwerveModule(backRightDriveMotor, backRightAngleMotor, kBackRightAngleOffset, kBackRightAngleDashboardOffset);

    //assigns swerve modules to an array 
    //this makes doing repetitive actions, such as updating states, much more convienent 
    modules = new ArrayList<SwerveModule>();
        modules.add(frontLeftModule);
        modules.add(frontRightModule);
        modules.add(backLeftModule);
        modules.add(backRightModule);

    //sets module distances from center of rotation
    //forward = postive x, right = positive y
    Translation2d frontLeft = new Translation2d(kXDistanceFromCenter, -kYDistanceFromCenter);
    Translation2d frontRight = new Translation2d(kXDistanceFromCenter, kYDistanceFromCenter);
    Translation2d backLeft = new Translation2d(-kXDistanceFromCenter, -kYDistanceFromCenter);
    Translation2d backRight = new Translation2d(-kXDistanceFromCenter, kYDistanceFromCenter);

    //assigns module distance to kinematic object
    kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
    
    //creates odometry object from kinematics object
    odometry = new SwerveDriveOdometry(kinematics, getRobotYawInRotation2d());
    

    //instantiates navx
    try{
      navx = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    //sets angle adjustment
    navx.setAngleAdjustment(0);

    //assigns values to togglables
    isDriveFieldCentric = true;
  }

  /**
   * METHODS
   */

  /**
   * drive methods
   */

  /**
   * Drives with either robot-centric or field-centric
   * 
   * @param rawXInput Forward velocity
   * @param rawYInput Sideways velocity
   * @param rawRotate Angular velocity
   */
  public void drive(double rawXInput, double rawYInput, double rawRotate){
     //sets deadbands
     double xInput = deadband(rawXInput);
     double yInput = deadband(rawYInput);
     double rotate = deadband(rawRotate);

    //if there is no stick input
    if(xInput == 0 && yInput == 0 && rotate == 0){
        lockWheels();
    } else {
      if(isDriveFieldCentric){
        fieldCentricDrive(xInput, yInput, rotate);
      } else if(!isDriveFieldCentric){
        robotCentricDrive(xInput, yInput, rotate);
      }
    }
  }

  /**
   * Drives the robot using given x, y, and angular stick inputs.
   * See the front of the robot as forward.
   * 
   * @param rawXInput Forward velocity
   * @param rawYInput Sideways velocity
   * @param rawRotate Angular velocity
   */
  public void robotCentricDrive(double xInput, double yInput, double rotate){
   
    //sets target angle and velocity based on stick input
    double xVelocity = xInput * kMaxMPS;
    double yVelocity = yInput * kMaxMPS;
    double rotateVelocity = rotate * kMaxModuleRPM;
    
    //converts input targets to individual module states (robot-centric)
    ChassisSpeeds targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
    moduleStates = kinematics.toSwerveModuleStates(targetVelocity);

    setModuleStates(moduleStates);
  } 

  /**
   * Drives the robot using given x, y, and angular stick inputs.
   * See the Navx gyro reading of 0 as forwardz
   */
  public void fieldCentricDrive(double xInput, double yInput, double rotate){
    //sets target angle and velocity based on stick input
    double xVelocity = xInput * kMaxMPS;
    double yVelocity = yInput * kMaxMPS;
    double rotateVelocity = rotate * kMaxModuleRPM;

    //converts input targets to individual module states (field centric)
    ChassisSpeeds targetVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocity, yVelocity, rotateVelocity, getRobotYawInRotation2d());
    moduleStates = kinematics.toSwerveModuleStates(targetVelocity);

    setModuleStates(moduleStates);
  }

  /**
   * Takes module state data and converts it into module velocities and angles
   */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    for(SwerveModule module: modules){
      
      //for testing indv modules; leave out unless running only one module
      //SwerveModule module = backRightModule;

      int i = modules.indexOf(module);

      //sets module velocity using closed loop velocity control
      //module.setModuleVelocity(module.getTargetVelocity(moduleStates[i]));
      
      //sets angle of module using closed loop position control
      module.setModuleAngle(module.getTargetAngle(moduleStates[i]));
    }
  }

  /**
   * Sets all drive input to 0
   */
  public void stop(){
    robotCentricDrive(0, 0, 0);
  }

  /**
   * Sets wheels into locked position (most resistant to being pushed)
   */
  public void lockWheels(){
    
    ChassisSpeeds wheelLock = new ChassisSpeeds(0.0, 0.0, 1.0);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(wheelLock);

    for(SwerveModule module: modules){

      int i = modules.indexOf(module);
      module.setModuleVelocity(0);
      module.setModuleAngle(module.getTargetAngle(moduleStates[i]));
    }
  }

  /**
   * Toggles between field-centric drive and robot-centric drive
   */
  public void toggleIsDriveFieldCentric(){
    isDriveFieldCentric = !isDriveFieldCentric;
  }

  public boolean getIsDriveFieldCentric(){
    return isDriveFieldCentric;
  }

  //navx methods 
  public double getRobotYaw(){
    return navx.getYaw();
  }

  public Rotation2d getRobotYawInRotation2d(){
    double yaw = getRobotYaw();
    return Rotation2d.fromDegrees(yaw);
  }

  public void zeroNavx(){
    navx.zeroYaw();
  }
  //gets the x and y coordinates of the robot's position on the field in meters
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  

  /**
   * Applies a deadband to raw joystick input
   * 
   * @param input raw joystick input
   * @return deadbanded joystick input
   */
  public double deadband(double input){
    double outMax = 1.0;
    double outMin = -1.0;
    double inMax = 1.0;
    double inMin = -1.0; 

    double output = 0.0;
    //System.out.println("input = " + input);
    if(input <= kMotorNeutralDeadband && input >= (-kMotorNeutralDeadband)){
      //System.out.println("at 0");
      output = 0.0;
    }

    if(input >= kMotorNeutralDeadband){
      //System.out.println("above deadband");
                //new slope for motor output                 //repositions constant based on deadband
      output = (outMax / (inMax - kMotorNeutralDeadband)) * (input - kMotorNeutralDeadband);
    }

    if(input <= -kMotorNeutralDeadband){
      //System.out.println("below deadband");
               //new slope for motor output                  //repositions constant based on deadband
      output = (outMin / (kMotorNeutralDeadband + inMin)) * (input + kMotorNeutralDeadband);
    }
    //System.out.println("output = " + output);
    return output;
  }

  /**
   * DIAGNOSTIC 
   */

  //motor test drive
  public void motorTest(SwerveModule module, double driveInput, double angleInput){
    module.testDriveMotor(driveInput);
    module.testAngleMotor(angleInput);

    SmartDashboard.putNumber("driveStickInput", driveInput);
    SmartDashboard.putNumber("angleStickInput", angleInput);
  }

  public void testAnglePIDLoop(SwerveModule module, double rawXInput, double rawYInput){
    //System.out.println("raw x = " + rawXInput);
    //System.out.println("raw y = " + rawYInput);
    double xInput = deadband(rawXInput);
    double yInput = deadband(rawYInput);
    System.out.println("band x = " + xInput);
    System.out.println("band y = " + yInput);
    module.setAngleSetpoint(xInput, yInput);
  }

  public void testDrivePIDFLoop(List<SwerveModule> modules, double driveInput){
    double input = deadband(driveInput);
    double targetVelocity = input * kMaxMPS;
    
    for(SwerveModule module: modules){
    module.setModuleVelocity(targetVelocity);
    }
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("naxv angle", getRobotYaw());
    SmartDashboard.putBoolean("isDriveFieldCentric", getIsDriveFieldCentric());
    //SmartDashboard.putBoolean("areWheelsLocked", getAreWheelsLocked());
    SmartDashboard.putString("positionOnField", odometry.getPoseMeters().toString());

    //updates the robot's position on the field over time
     odometry.update(getRobotYawInRotation2d(), moduleStates);
  }

}
