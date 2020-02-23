/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.util.GeneralUtil.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class MagazineSubsystem extends SubsystemBase {
  
  CANSparkMax magBeltMotor;
  CANEncoder magEncoder;
  CANPIDController magPID;
  public Solenoid magPis;
  Timer timer = new Timer();



  int ballCount = 0;
  boolean hasBallEntered = false;
  boolean hasBallCounted = false;

  public MagazineSubsystem() {
    //Magazine Neo Information
    magBeltMotor = new CANSparkMax(ID_MAG_BELT_MOTOR, MotorType.kBrushless);
    magBeltMotor.restoreFactoryDefaults();
    magBeltMotor.setInverted(true);
    magBeltMotor.setSmartCurrentLimit(kCurrentLimit);

    magPis = new Solenoid(kMagazinePistonPort);

    //Magazine PID Controller
    magPID = magBeltMotor.getPIDController();
    //Magazine Motor Encoder
    magEncoder = magBeltMotor.getEncoder();

    magPID.setFeedbackDevice(magEncoder);

    setPIDGains(magPID, PIDProfile.MAGAZINE);
    //Lifts Magazine belt on startup
    //magPis.set(true);

    //Sets sonar to constant pulse
  }

  //Magazine Conveyor 
  public void setBeltVelocity(double targetVelocity) {
    magPID.setReference(targetVelocity, ControlType.kVelocity);
  }
  //Magazine Belt Is Set To Load Speed
  public void magLoad() {
    setBeltVelocity(kMagLoadSpeed);
  }
  //Magazine Belt Is Set To Shoot Speed
  public void magShoot() {
    setBeltVelocity(kMagShootSpeed);
  }
  //Magazine Belt Is Set To Idle Speed
  public void magIdle() {
    setBeltVelocity(kMagIdleSpeed);
  }
  //Magazine Belt Is Set To Stop
  public void stop() {
    magPID.setReference(0, ControlType.kVelocity);
  }

  //Magazine "Left" and "Right" Belt Lift Pistons
  public void magDisengage(){
    stop();
    magPis.set(false);
  }
  public void magEngage(){
    magShoot();
    magPis.set(true);
  }

  public void runAtIdle(){
    magIdle();
    magPis.set(false);
  }

  double speed;
  public void test(double speed){
    this.speed = speed;
    magBeltMotor.set(speed);
  }

  public double getSpeed(){
    return speed;
  }

  //Ultrasonic Sonar Ball Counter
  public void senseBall() {

  


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
  @Override
  public void periodic() {
    SmartDashboard.putNumber("vel", magEncoder.getVelocity());

    SmartDashboard.putNumber("input", speed);
  }
}