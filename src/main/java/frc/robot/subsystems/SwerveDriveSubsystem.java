/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
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

  SwerveDriveKinematics kinematics;

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

    //assigns drive and angle motors to their respective swerve modules
    frontLeftModule = new SwerveModule(frontLeftDriveMotor, frontLeftAngleMotor);
    frontRightModule = new SwerveModule(frontRightDriveMotor, frontRightAngleMotor);
    backLeftModule = new SwerveModule(backLeftDriveMotor, backLeftAngleMotor);
    backRightModule = new SwerveModule(backRightDriveMotor, backRightAngleMotor);

    //assigns swerve modules to an array 
    //this makes doing repetitive actions, such as updating states, much more convienent 
    modules = new ArrayList<SwerveModule>();
        modules.add(frontLeftModule);
        modules.add(frontRightModule);
        modules.add(backLeftModule);
        modules.add(backRightModule);

    //forward = postive x, left = positive y
    Translation2d frontLeft = new Translation2d(kXDistanceFromCenter, kYDistanceFromCenter);
    Translation2d frontRight = new Translation2d(kXDistanceFromCenter, -kYDistanceFromCenter);
    Translation2d backLeft = new Translation2d(-kXDistanceFromCenter, kYDistanceFromCenter);
    Translation2d backRight = new Translation2d(-kXDistanceFromCenter, -kYDistanceFromCenter);

    kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * Drives the robot using given x, y, and angular stick inputs 
   * 
   * @param forwardStrafe Forward velocity
   * @param sidewaysStrafe Sideways velocity
   * @param rotate Angular velocity
   */
  public void drive(double forwardStrafe, double sidewaysStrafe, double rotate){
    //sets deadbands
    if(forwardStrafe <= kMotorNeutralDeadband && forwardStrafe >= -kMotorNeutralDeadband){
      forwardStrafe = 0;
    }
    if(sidewaysStrafe <= kMotorNeutralDeadband && sidewaysStrafe >= -kMotorNeutralDeadband){
      sidewaysStrafe = 0;
    }
    if(rotate <= kMotorNeutralDeadband && rotate >= -kMotorNeutralDeadband){
      rotate = 0;
    }
    //sets target angle and velocity based on stick input
    
    double xVelocity = forwardStrafe ;//* kMaxMPS;
    double yVelocity = sidewaysStrafe ;//* kMaxMPS;
    double rotateVelocity = rotate ;//* kMaxRPM;
    
    //converts input targets to individual module states
    ChassisSpeeds targetVelocity = new ChassisSpeeds(xVelocity, yVelocity, rotateVelocity);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(targetVelocity);

    //updates target velocity and angle for each swerve module
    for(SwerveModule module: modules){
      
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
    drive(0, 0, 0);
  }

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
      //new slope for motor output                           //repositions constant based on deadband
      output = (outMin / (kMotorNeutralDeadband + inMin)) * (input + kMotorNeutralDeadband);
    }
    //System.out.println("output = " + output);
    return output;
  }

  //DIAGNOSTIC 
  //motor test drive
  public void motorTest(SwerveModule module, double driveInput, double angleInput){
    module.testDriveMotor(driveInput);
    module.testAngleMotor(angleInput);
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
    //gives current encoder position for all 8 motors
    /*SmartDashboard.putNumber("frontLeftDriveEncoder", frontLeftModule.getDriveEncoder());
    SmartDashboard.putNumber("frontLeftAngleEncoder", frontLeftModule.getAngleEncoder());*/

    SmartDashboard.putNumber("driveVelocity", frontRightModule.getDriveVelocity());
    /*SmartDashboard.putNumber("front right angle", frontRightModule.getAngleEncoder());
    SmartDashboard.putNumber("back left drive", backLeftModule.getDriveEncoder());
    SmartDashboard.putNumber("back left angle", backLeftModule.getAngleEncoder());
    SmartDashboard.putNumber("back right drive", backRightModule.getDriveEncoder());
    SmartDashboard.putNumber("back right angle", backRightModule.getAngleEncoder());*/

    //gives target velocity and angles
    SmartDashboard.putNumber("targetVelocity", frontRightModule.getTargetVelocity());
    SmartDashboard.putNumber("targetAngle", frontRightModule.getTargetAngle());

    SmartDashboard.putNumber("drive output", frontRightModule.getDriveMotorOutput());
    SmartDashboard.putNumber("angle output", frontRightModule.getAngleMotorOutput());
  }
}