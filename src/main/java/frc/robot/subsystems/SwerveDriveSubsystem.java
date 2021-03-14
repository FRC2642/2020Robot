/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics.normalizeWheelSpeeds;
import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.*;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SwerveModule;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANAnalog;



public class SwerveDriveSubsystem extends SubsystemBase {
  public CANSparkMax frontLeftDriveMotor, frontLeftAngleMotor;
  public CANSparkMax frontRightDriveMotor, frontRightAngleMotor;
  public CANSparkMax backLeftDriveMotor, backLeftAngleMotor;
  public CANSparkMax backRightDriveMotor, backRightAngleMotor;

  public SwerveModule frontLeftModule;
  public SwerveModule frontRightModule;
  public SwerveModule backLeftModule;
  public SwerveModule backRightModule;
  public List<SwerveModule> modules;

  public SwerveModuleState[] moduleStates;
  public SwerveModuleState state;

  public SwerveDriveKinematics kinematics;
  public SwerveDriveOdometry odometry;

  public AHRS navx;
  public TrajectoryConfig config;
  public Trajectory centerTrajectory;
  public Trajectory leftTrajectory;
  public Trajectory rightTrajectory1;
  public Trajectory rightTrajectory2;
  public Trajectory exampleTrajectory;

  public CANSparkMax driveMotor;
  public CANEncoder driveEncoder;
  //public CANAnalog absoluteAngleEncoder;
  public CANSparkMax angleMotor;

  public Pose2d updatedPose;
 
  public boolean isDriveFieldCentric;
  public boolean isAimingMode;
  public boolean areAllWheelsAligned;

  public double position;

  public boolean isSlowDrive = false;

  /**
   * Creates a new SwerveDriveSubsystem.
 * @return 
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

    /*angleMotor = new ArrayList<CANSparkMax>();
      angleMotor.add(frontLeftAngleMotor);
      angleMotor.add(frontRightAngleMotor);
      angleMotor.add(backLeftAngleMotor);
      angleMotor.add(backRightAngleMotor);*/

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
    //was true
    frontLeftAngleMotor.setInverted(true);
    
    frontRightDriveMotor.setInverted(true);
    frontRightAngleMotor.setInverted(true);
    
    backLeftDriveMotor.setInverted(true);
    backLeftAngleMotor.setInverted(true);
    
    backRightDriveMotor.setInverted(true);
    //was true
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

    //assign drive encoder
    this.driveMotor = driveMotor;
    driveEncoder = frontLeftDriveMotor.getEncoder();

    //assigns angle encoder
    //this.angleMotor = angleMotor;
    //absoluteAngleEncoder = frontLeftAngleMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);
    //absoluteAngleEncoder.setPositionConversionFactor(kAnglePositionConversionFactor); //voltage into degrees
    driveEncoder.setVelocityConversionFactor(kDriveVelocityConversionFactor); //rpm into MPS  



    //assigns drive and angle motors to their respective swerve modules with offsets
    frontLeftModule = new SwerveModule(frontLeftDriveMotor, frontLeftAngleMotor, kFrontLeftAngleModuleOffset);
    frontRightModule = new SwerveModule(frontRightDriveMotor, frontRightAngleMotor, kFrontRightAngleModuleOffset);
    backLeftModule = new SwerveModule(backLeftDriveMotor, backLeftAngleMotor, kBackLeftAngleModuleOffset);
    backRightModule = new SwerveModule(backRightDriveMotor, backRightAngleMotor, kBackRightAngleModuleOffset);

    //assigns swerve modules to an array 
    //this simplifies updating module states
    modules = new ArrayList<SwerveModule>();
      modules.add(frontLeftModule);
      modules.add(frontRightModule);
      modules.add(backLeftModule);
      modules.add(backRightModule);

    //sets module distances from center of rotation
    //forward = postive x, right = positive y
    Translation2d frontLeft = new Translation2d(-kXDistanceFromCenter, kYDistanceFromCenter);
    Translation2d frontRight = new Translation2d(-kXDistanceFromCenter, -kYDistanceFromCenter);
    Translation2d backLeft = new Translation2d(kXDistanceFromCenter, kYDistanceFromCenter);
    Translation2d backRight = new Translation2d(kXDistanceFromCenter, -kYDistanceFromCenter);

    //assigns module distance to kinematic object
    kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

    odometry = new SwerveDriveOdometry(kinematics, getRobotYawInRotation2d());

     TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxMPS, Constants.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kinematics);
      
      config.setReversed(true);

      /* Trajectory centerTrajectory = TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(
                  new Translation2d(3.1496, 0)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3.1496, 0, new Rotation2d(0)),
              config);

      Trajectory leftTrajectory = TrajectoryGenerator.generateTrajectory(
              new Pose2d(0,0, new Rotation2d(0)), 
                
                List.of(
                  new Translation2d(3.1496, 0),
                  new Translation2d(0, -2.64282819922)
              ),
              new Pose2d(3.1496,-2.64282819922, new Rotation2d(0)), 
              config);
            
              //not final yet maybe possibly :\ ????
      Trajectory rightTrajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                
                List.of(
                    new Translation2d(2.200402, 0)),
                
                    new Pose2d(2.200402, 0, new Rotation2d(0)),
                config);
            
      Trajectory rightTrajectory2 = TrajectoryGenerator.generateTrajectory(
                  new Pose2d(2.200402, 0, new Rotation2d(0)),
                  
                  List.of(
                      new Translation2d(2.7432, 0)),
                  
                      new Pose2d(4.943602, 0, new Rotation2d(0)),
                  config); */

        
    
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
    isSlowDrive = false;
  }

  /**
   * METHODS
   */

  /**
   * drive methods
   */
  public double getDrivePosition(){
    position = driveEncoder.getPosition();
    return position;
  }
  /**
   * Drives with either robot-centric or field-centric
   * 
   * @param rawXInput Joystick input (left hand l/r)
   * @param rawYInput Joyatick input (left hand up/down)
   * @param rawRotate Joystick input (right hand l/r)
   */
  public void drive(double rawXInput, double rawYInput, double rawRotate){

    double xInput = rawXInput;
    double yInput = rawYInput;
    double rotate = rawRotate;

    //sets deadbands
    if(Math.abs(rawXInput) <= kMotorNeutralDeadband && Math.abs(rawXInput) <= kMotorNeutralDeadband){
      //System.out.println("banding");
      xInput = deadband(rawXInput);
      yInput = deadband(rawYInput);
    } /* else {
      System.out.println("not banding");
    } */
    rotate = deadband(rawRotate);

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

  /**
   * Drives with vision-assisted rotation to aim at vision target
   * 
   * @param rawXInput Joystick input (left hand l/r)
   * @param rawYInput Joystick input (left hand up/down)
   * @param rawRotate Input from vision 
   */

  public void driveByAimbot(double rawXInput, double rawYInput, double rawRotate){
    //sets deadbands
    double xInput = deadband(rawXInput);
    double yInput = deadband(rawYInput);
    //double rotate = deadband(rawRotate);
    double rotate = rawRotate;

    //sqaures joystick input
    xInput *= Math.abs(xInput);
    yInput *= Math.abs(yInput);
    //rotate *= Math.abs(rotate);

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
    Translation2d centerOfRotation = new Translation2d(-kXDistanceFromCenter, 0);

    //converts input targets to individual module states (aiming mode)
    ChassisSpeeds targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
    moduleStates = kinematics.toSwerveModuleStates(targetVelocity, centerOfRotation);
    normalizeWheelSpeeds(moduleStates, kMaxMPS);

    setModuleStates(moduleStates);
  }

  /**e
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

    odometry.update(getRobotYawInRotation2d(), moduleStates);
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

    frontLeftModule.zeroOutI();
    frontRightModule.zeroOutI();
    backLeftModule.zeroOutI();
    backRightModule.zeroOutI();

    //sets wheels in the locked orientation
    /*frontLeftModule.setModuleAngle(frontLeftAngle);   
    frontRightModule.setModuleAngle(frontRightAngle);
    backLeftModule.setModuleAngle(backLeftAngle);
    backRightModule.setModuleAngle(backRightAngle);*/

    //updates swerve module states for pose 
    SwerveModuleState frontLeft = new SwerveModuleState(frontLeftVelocity, frontLeftAngle);
    SwerveModuleState frontRight = new SwerveModuleState(frontRightVelocity, frontRightAngle);
    SwerveModuleState backLeft = new SwerveModuleState(backLeftVelocity, backLeftAngle);
    SwerveModuleState backRight = new SwerveModuleState(backRightVelocity, backRightAngle);
    
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight);

    moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  public void alignWheels(){
    for(SwerveModule module: modules){
        module.zeroModules();
    }
  }

  /** 
   * TOGGLEABLES AND GETTERS FOR DRIVING STATES
  */
  /** */
  public void toggleDriveFieldCentric(){
    isDriveFieldCentric = !isDriveFieldCentric;
  }

  public boolean getIsDriveFieldCentric(){
    return isDriveFieldCentric;
  }

  public void toggleAimingMode(){
    isAimingMode = !isAimingMode;
  }

  public boolean getIsAimingMode(){
    return isAimingMode;
  }

  public void setSlowDrive(boolean state){
    isSlowDrive = state;
  }

  public boolean getIsSlowDrive(){
    boolean rv = isSlowDrive;
    return rv;
  }

  /**
   * NAVX METHODS
   */
  /** */

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

  /**
   * POSE GENERATION FOR USE IN AUTO PATHING
   */
  /** */

  public Pose2d getPose(){
    Pose2d pose = odometry.getPoseMeters();
    Translation2d transPose = pose.getTranslation();
    transPose = transPose.div(kMaxSpeedConversionFactor);
    Pose2d realPose = new Pose2d(transPose, pose.getRotation());
    return realPose; 

    //return pose;
  }

  public double getPoseXInFeet(){
    Pose2d pose2d = getPose();
    Translation2d poseTrans2d = pose2d.getTranslation();
    double xPose = poseTrans2d.getX();
    return Units.metersToFeet(xPose);
  }

  public double getPoseXInInches(){
    Pose2d pose2d = getPose();
    Translation2d poseTrans2d = pose2d.getTranslation();
    double xPose = poseTrans2d.getX();
    return Units.metersToInches(xPose);
  }


  public void resetPose(){
    odometry.resetPosition(new Pose2d(), getRobotYawInRotation2d());
  }

  public Pose2d updatedPose(){
    updatedPose = odometry.update(getRobotYawInRotation2d(), frontLeftModule.getState(), frontRightModule.getState(), backLeftModule.getState(), backRightModule.getState());
        driveEncoder.setVelocityConversionFactor(kDriveVelocityConversionFactor); //rpm into MPS  
    return updatedPose;
  }
  
  public void doNothing(){
  }

  /*public double getAbsoluteAngleEncoder(){
    return absoluteAngleEncoder.getPosition();
  }*/

  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
    }



  /**
   * TARGET ACQUISITON FOR AIMING
   */
  /** */

  /* public double getTargetX(){
    double target = Robot.jevoisCam.getCenterOfVisionTarget();
    return target;
  } */

//AUTONOMOUS

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
    SmartDashboard.putBoolean("isDriveFieldCentric", getIsDriveFieldCentric());
    SmartDashboard.putString("positionOnField", getPose().toString());
    SmartDashboard.putNumber("positionInInches", getPoseXInInches());

    

   /*  try{
    odometry.update(getRobotYawInRotation2d(), moduleStates);
    } catch(RuntimeException e){ 
      System.out.println("not updating");
    } */

  
  
   //odometry.update(getRobotYawInRotation2d(), frontLeftState, frontRightState, backLeftState, backRightState);

  //SmartDashboard.putNumber("fl vel", frontLeftModule.getDriveVelocity());
  }
}