/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

public class MagazineSubsystem extends SubsystemBase {
  
  CANSparkMax magBeltMotor;
  CANEncoder magEncoder;
  CANPIDController magPID;

  public Solenoid magazineLeftPis = new Solenoid(Constants.kLeftMagazinePis);
  public Solenoid magazineRightPis = new Solenoid(Constants.kRightMagazinePis);

  Ultrasonic ultra = new Ultrasonic(Constants.kMagazineSonarOutput, Constants.kMagazineSonarInput);

  int ballCount = 0;
  boolean hasBallEntered = false;

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

  //Lifts Magazine belt on startup
  magazineLeftPis.set(true);
  magazineRightPis.set(true);

  //Sets sonar to constant pulse
  ultra.setAutomaticMode(true);
  }

  //Magazine Conveyor 
  public void setBeltVelocity(double targetVelocity){
    magPID.setReference(targetVelocity, ControlType.kVelocity);
  }

  public void magBeltOn(){
    setBeltVelocity(kMagBeltSpeed);
  }

  //Magazine "Left" and "Right" Belt Lift Pistons
  public void magDisengage(){
    magazineLeftPis.set(true);
    magazineRightPis.set(true);}
  public void magEngage(){
    magazineLeftPis.set(false);
    magazineRightPis.set(false);}

  //Ultrasonic Sonar Ball Counter
  public void senseBall(){

  //Gets the sonar's range in inches
  double range = ultra.getRangeInches();

  if(range <= 6) {
    hasBallEntered = true;
  } else {
    hasBallEntered = false;
  }
  if (hasBallEntered = true) {
    ballCount++;
  } else {
  }   }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
