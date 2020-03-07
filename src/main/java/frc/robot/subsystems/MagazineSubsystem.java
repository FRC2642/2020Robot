/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.GeneralUtil.PIDProfile;
import frc.robot.util.library.simple.RightSight;

public class MagazineSubsystem extends SubsystemBase {
  
  CANSparkMax topMagBelt;
  CANSparkMax bottomMagBelt;

  CANEncoder magEncoder;
  CANPIDController magPID;
  
  Solenoid magPis;

  public RightSight rightSight = new RightSight(kRightSight);

  int ballCount = 0;
  boolean hasBallEntered = false;
  boolean hasBallCounted = false;

  boolean isMagEngagedAtIdle;

  public MagazineSubsystem() {
    //Magazine Neo Information
    topMagBelt = new CANSparkMax(ID_TOP_MAG_BELT_MOTOR, MotorType.kBrushless);
    topMagBelt.restoreFactoryDefaults();
    topMagBelt.setInverted(true);
    topMagBelt.setSmartCurrentLimit(kCurrentLimit);

    bottomMagBelt = new CANSparkMax(ID_BOTTOM_MAG_BELT_MOTOR, MotorType.kBrushless);
    bottomMagBelt.restoreFactoryDefaults();
    bottomMagBelt.setInverted(true);
    bottomMagBelt.setSmartCurrentLimit(kCurrentLimit);

    magPis = new Solenoid(kMagazinePistonPort);

    magPID = topMagBelt.getPIDController();
    magEncoder = topMagBelt.getEncoder();

    magPID.setFeedbackDevice(magEncoder);

    setPIDGains(magPID, PIDProfile.MAGAZINE);

    //boolean = whether to invert output being followed
    bottomMagBelt.follow(topMagBelt, true);

    isMagEngagedAtIdle = false;
  }



   /**
   * MAGAZINE STATES (belt vel + piston settings)
   */
  /** */

  public void setToIdleState(){
    magStop();
    magPis.set(isMagEngagedAtIdle);
  }

  public void setToShootingState(){
    magShoot();
    magPis.set(true);
  }

  public void setToShootingState(double beltVel){
    magShoot(beltVel);
    magPis.set(true);
  }

  public void setToEjectState(){
    magEject();
    magPis.set(false);
  }

  /**
   * BELT VELOCITY SETTERS
   */
  /** */

  public void magStop() {
    setBeltVelocity(0);
  }

  public void magLoad() {
    senseBall();
    setInverted(true);
    setBeltVelocity(kMagLoadSpeed);
  }

  public void magEject() {
    setInverted(false);
    setBeltVelocity(kMagEjectSpeed);
  }
  
  public void magShoot() {
    setInverted(true);
    setBeltVelocity(kMagDefaultShootSpeed);
  }
  
  public void magShoot(double vel){
    setInverted(true);
    setBeltVelocity(vel);
  }

  public void setBeltVelocity(double targetVelocity) {
    magPID.setReference(targetVelocity, ControlType.kVelocity);
  }

  /**
   * MAG PISTON SETTERS 
   */
  /** */

  public void setMagPis(boolean state){
    magPis.set(state);
  }

  public void toggleIdleState(){
    isMagEngagedAtIdle = !isMagEngagedAtIdle;
  }

  /**
   * BELT MOTOR SETTINGS AND DIAGNOSTICS
   */
  /** */

  public void setInverted(boolean inverted){
    topMagBelt.setInverted(inverted);
  }

  public double getVelocity(){
    return magEncoder.getVelocity();
  }

  /**
   * BALL INDEXER METHODS
   */

  Timer timer = new Timer();
  public void senseBall() {

    if(rightSight.get() == true) {
      hasBallEntered = true;
    } else {
      hasBallEntered = false;
    }

  if (hasBallEntered && !hasBallCounted) {
    ballCount++;
    hasBallCounted = true;
  }
  
    if (ballCount == 5) {
      timer.start();
      RobotContainer.auxController.setRumble(RumbleType.kLeftRumble, 1);
      RobotContainer.auxController.setRumble(RumbleType.kRightRumble, 1);
    }
    if (timer.get() > .5) {
      RobotContainer.auxController.setRumble(RumbleType.kLeftRumble, 0);
      RobotContainer.auxController.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  public int getBallCount() {
    return ballCount;
  }

  /**
   * STATE GETTERS FOR AUTO SHOOTING
   */
  /** */

  public boolean isMagReadyToShoot(){

    boolean isArmReady = RobotContainer.arm.isArmAtGoal();
    boolean areWheelsReady = RobotContainer.shooter.isAtTargetVelocity();

    if(isArmReady && areWheelsReady){
      return true;
    } else {
      return false;
    }
  }

  /**
   * TRIGGERS
   */
  /** */
  public boolean getRightTrigger() {
    double rt = RobotContainer.driveController.getTriggerAxis(Hand.kRight);
    return (rt > .5);
  }

  @Override
  public void periodic() {
  }
}