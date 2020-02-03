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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class MagazineSubsystem extends SubsystemBase {
  //(Charles- remember to comment)
  CANSparkMax magBeltMotor;
  public Solenoid magazineLeftPis = new Solenoid(Constants.kLeftMagazinePis);
  public Solenoid magazineRightPis = new Solenoid(Constants.kRightMagazinePis);
  public TalonSRX magBelt = new TalonSRX(Constants.ID_MAG_BELT_MOTOR);

  public MagazineSubsystem() {
  magBeltMotor = new CANSparkMax(ID_MAG_BELT_MOTOR, MotorType.kBrushless);
  magBeltMotor.restoreFactoryDefaults();
  magBeltMotor.setInverted(false);
  magBeltMotor.setSmartCurrentLimit(kCurrentLimit);
  }

  //Magazine Conveyor 
  public void magBeltForward(){
    magBelt.set(ControlMode.PercentOutput,-0.6);
  }
  public void magBeltBackward(){
    magBelt.set(ControlMode.PercentOutput,0.2);
  }

  //Magazine "Left" and "Right" Belt Lift Pistons
  public void magDisengage(){
    magazineLeftPis.set(true);
    magazineRightPis.set(true);}
  public void magEngage(){
    magazineLeftPis.set(false);
    magazineRightPis.set(false);}

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
