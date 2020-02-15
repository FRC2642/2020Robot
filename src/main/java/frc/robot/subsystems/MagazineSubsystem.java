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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagazineSubsystem extends SubsystemBase {
  
  CANSparkMax magBeltMotor;
  CANEncoder magEncoder;
  CANPIDController magPID;
  public Solenoid magPis = new Solenoid(kMagazinePistonPort);
  Ultrasonic ultra = new Ultrasonic(kMagazineSonarOutput, kMagazineSonarInput);

  int ballCount = 0;
  boolean hasBallEntered = false;
  boolean hasBallCounted = false;

  public MagazineSubsystem() {
    //Magazine Neo Information
    magBeltMotor = new CANSparkMax(ID_MAG_BELT_MOTOR, MotorType.kBrushless);
    magBeltMotor.restoreFactoryDefaults();
    magBeltMotor.setInverted(false);
    magBeltMotor.setSmartCurrentLimit(kCurrentLimit);

    //Magazine PID Controller
    magPID = magBeltMotor.getPIDController();
    //Magazine Motor Encoder
    magEncoder = magBeltMotor.getEncoder();

    magPID.setFeedbackDevice(magEncoder);

    setPIDGains(magPID, PIDProfile.MAGAZINE);
    //Lifts Magazine belt on startup
    magPis.set(true);

    //Sets sonar to constant pulse
    ultra.setAutomaticMode(true);
  }

  //Magazine Conveyor 
  public void setBeltVelocity(double targetVelocity){
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
  public void stop(){
    magPID.setReference(0, ControlType.kVelocity);
  }

  //Magazine "Left" and "Right" Belt Lift Pistons
  public void magDisengage(){
    magPis.set(true);
  }
  public void magEngage(){
    magPis.set(false);
  }

  //Ultrasonic Sonar Ball Counter
  public void senseBall(){

    //Gets the sonar's range in inches
    double range = ultra.getRangeInches();

      if(range <= 6) {
        hasBallEntered = true;
      } else {
        hasBallEntered = false;
      }


    if (hasBallEntered && !hasBallCounted) {
      ballCount++;
      hasBallCounted = true;
    }
  }     
  public int getballCount() {
    return ballCount;
  }
  public void resetBallCount() {
    ballCount = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
