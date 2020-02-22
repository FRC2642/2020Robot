/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import static frc.robot.Constants.*;
import static edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics.normalizeWheelSpeeds;
import static frc.robot.util.GeneralUtil.*;

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
  public SwerveModuleState[] moduleStates;
  public SwerveModuleState state;

  public SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;

  public AHRS navx;
  public TrajectoryConfig config;
  public Trajectory exampleTrajectory;
 
  public boolean isDriveFieldCentric;
  public boolean isAimingMode;
  public boolean areAllWheelsAligned;

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
    backLeftDriveMotor.setInverted(true);
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

    //assigns drive and angle motors to their respective swerve modules with offsets
    frontLeftModule = new SwerveModule(frontLeftDriveMotor, frontLeftAngleMotor, kFrontLeftAngleOffset, kFrontLeftAngleDashboardOffset);
    frontRightModule = new SwerveModule(frontRightDriveMotor, frontRightAngleMotor, kFrontRightAngleOffset, kFrontRightAngleDashboardOffset);
    backLeftModule = new SwerveModule(backLeftDriveMotor, backLeftAngleMotor, kBackLeftAngleOffset, kBackLeftAngleDashboardOffset);
    backRightModule = new SwerveModule(backRightDriveMotor, backRightAngleMotor, kBackRightAngleOffset, kBackRightAngleDashboardOffset);

    //assigns swerve modules to an array 
    //this simplifies updating module states
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

    odometry = new SwerveDriveOdometry(kinematics, getRobotYawInRotation2d());

    TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxMPS, Constants.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kinematics);

    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      config
    );
    
    
    //instantiates navx
    try{
      navx = new AHRS();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    //assigns values to togglables
    isDriveFieldCentric = true;
    isAimingMode = false;
    areAllWheelsAligned = false;
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

    //sqaures joystick input
    xInput *= Math.abs(xInput);
    yInput *= Math.abs(yInput);
    rotate *= Math.abs(rotate);

    //if there is no stick input
    if(xInput == 0 && yInput == 0 && rotate == 0){
        lockWheels();
    } else {
      //chooses between field centric mode, robot centric mode, and aiming mode
      if(isDriveFieldCentric){
        fieldCentricDrive(xInput, yInput, rotate);
      } else if(!isDriveFieldCentric){
        robotCentricDrive(xInput, yInput, rotate);
      }
    }
  }

  public void driveByAimbot(double rawXInput, double rawYInput, double rawRotate){
    //sets deadbands
    double xInput = deadband(rawXInput);
    double yInput = deadband(rawYInput);
    double rotate = deadband(rawRotate);

    //sqaures joystick input
    xInput *= Math.abs(xInput);
    yInput *= Math.abs(yInput);
    rotate *= Math.abs(rotate);

    //if there is no stick input
    if(xInput == 0 && yInput == 0 && rotate == 0){
        lockWheels();
    } else {
        aimingModeDrive(xInput, yInput, rotate);
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
    normalizeWheelSpeeds(moduleStates, kMaxMPS);

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
     normalizeWheelSpeeds(moduleStates, kMaxMPS);

    setModuleStates(moduleStates);
  }

  public void aimingModeDrive(double xInput, double yInput, double rotate){
    //may be unnecessary if this is purely for aiming with vision
    double xVelocity = xInput * kMaxMPS;
    double yVelocity = yInput * kMaxMPS;
    double rotateVelocity = rotate * kMaxModuleRPM;
    Translation2d centerOfRotation = new Translation2d(kXDistanceFromCenter, 0);

    //converts input targets to individual module states (aiming mode)
    ChassisSpeeds targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
    moduleStates = kinematics.toSwerveModuleStates(targetVelocity, centerOfRotation);
    normalizeWheelSpeeds(moduleStates, kMaxMPS);

    setModuleStates(moduleStates);
  }

  /**
   * Takes module state data and converts it into module velocities and angles
   */
  public void setModuleStates(SwerveModuleState[] moduleStates){
    for(SwerveModule module: modules){
      
      //for testing indv modules; leave out 
      //SwerveModule module = frontRightModule;

      int i = modules.indexOf(module);

      //sets module velocity using closed loop velocity control
      module.setModuleVelocity(module.getTargetVelocity(moduleStates[i]));
      
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
    double frontLeftVelocity = 0.0;
    double frontRightVelocity = 0.0;
    double backLeftVelocity = 0.0;
    double backRightVelocity = 0.0;

    Rotation2d frontLeftAngle = toRotation2d(-45.0);
    Rotation2d frontRightAngle = toRotation2d(45.0);
    Rotation2d backLeftAngle = toRotation2d(45.0);
    Rotation2d backRightAngle = toRotation2d(-45.0);

    //stops wheels
    frontLeftModule.setModuleVelocity(frontLeftVelocity);
    frontRightModule.setModuleVelocity(frontRightVelocity);
    backLeftModule.setModuleVelocity(backLeftVelocity);
    backRightModule.setModuleVelocity(backRightVelocity);

    //sets wheels in the locked orientation
    frontLeftModule.setModuleAngle(frontLeftAngle);   
    frontRightModule.setModuleAngle(frontRightAngle);
    backLeftModule.setModuleAngle(backLeftAngle);
    backRightModule.setModuleAngle(backRightAngle);

    SwerveModuleState frontLeft= new SwerveModuleState(frontLeftVelocity, frontLeftAngle);
    SwerveModuleState frontRight = new SwerveModuleState(frontRightVelocity, frontRightAngle);
    SwerveModuleState backLeft = new SwerveModuleState(backLeftVelocity, backLeftAngle);
    SwerveModuleState backRight = new SwerveModuleState(backRightVelocity, backRightAngle);
    
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight);

    moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  //needs more precision + better way of knowing when done
  public void alignWheels(){
    
      for(SwerveModule module: modules){
        //begins zeroing modules
        module.zeroModules();

        //checks if all modules are aligned 
        for(SwerveModule mod: modules){
          //breaks loop if a module is not aligned
          if(!mod.getIsWheelAligned()){
            areAllWheelsAligned = false;
            break;
          }
          //if all modules return true, sets all wheels aligned to true
          areAllWheelsAligned = true;
        }
        //breaks if all wheels are aligned
        if(areAllWheelsAligned){
          break;
        }
      }
  }

  public boolean getAreAllWheelsAligned(){
    return areAllWheelsAligned;
  }

  public void toggleIsDriveFieldCentric(){
    isDriveFieldCentric = !isDriveFieldCentric;
  }

  public boolean getIsDriveFieldCentric(){
    return isDriveFieldCentric;
  }

  public void toggleIsAimingMode(){
    isAimingMode = !isAimingMode;
  }

  public boolean getIsAimingMode(){
    return isAimingMode;
  }

  //inverts spark
  public void invertMotor(CANSparkMax motor){
    boolean state = motor.getInverted();
    state = !state;
    motor.setInverted(state);
  }

  //navx methods 
  double lastHeading = 0;
  public double getRobotYaw(){
    double heading = lastHeading;
    try {
      heading = navx.getYaw();
      heading = (heading + kGyroOffset) % 360;
    } catch (NullPointerException e){
      System.out.println(e);
    }
    lastHeading = heading;
    return heading;
  }

  double lastYaw = 0;
  public Rotation2d getRobotYawInRotation2d(){
    double yaw = lastYaw; 
    try{
    yaw = getRobotYaw();
    } catch (NullPointerException e){
      System.out.println(e);
    }
    lastYaw = yaw;
    return Rotation2d.fromDegrees(yaw);
  }

  public Rotation2d toRotation2d(double angle){
    angle *= Math.PI / 180; 
    Rotation2d rot = new Rotation2d(angle);
    return rot;
  }

  public void zeroNavx(){
    navx.zeroYaw();
  }

  public Pose2d getPoseMeters(){
    return odometry.getPoseMeters();
  }

  public double getPoseXInFeet(){
    Pose2d pose2d = getPoseMeters();
    Translation2d poseTrans2d = pose2d.getTranslation();
    double pose = poseTrans2d.getX();
    return Units.metersToFeet(pose);
  }

  /**
   * DIAGNOSTIC 
   */

  //motor test drive
  public void motorTest(SwerveModule module, double driveInput, double angleInput){
    module.testDriveMotor(driveInput);
    module.testAngleMotor(angleInput);
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
    SmartDashboard.putString("positionOnField", odometry.getPoseMeters().toString());

    try{
    odometry.update(getRobotYawInRotation2d(), moduleStates);
    } catch(RuntimeException e){ 
    }

    SmartDashboard.putNumber("driveVelocity", frontLeftModule.getDriveVelocity());
    SmartDashboard.putNumber("poseXInFeet", getPoseXInFeet());

  }
}