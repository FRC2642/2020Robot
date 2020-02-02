/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class MagazineSubsystem extends SubsystemBase {
 
  public TalonSRX magazineBelt = new TalonSRX(Constants.ID_MAG_BELT_MOTOR);
  public Solenoid magazineLeftPis = new Solenoid(Constants.kLeftMagazinePis);
  public Solenoid magazineRightPis = new Solenoid(Constants.kRightMagazinePis);
  
  public MagazineSubsystem() {

  }
  //Magazine Conveyor 
  public void magBeltForward(){
    magazineBelt.set(ControlMode.PercentOutput,-0.6);}
  public void magBeltBackward(){
    magazineBelt.set(ControlMode.PercentOutput,0.2);}
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
