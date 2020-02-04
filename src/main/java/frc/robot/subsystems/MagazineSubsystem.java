/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MagazineSubsystem extends SubsystemBase {
 
  public TalonSRX magazineTilt = new TalonSRX(Constants.ID_MAG_TILT_MOTOR);
  public Solenoid magazineLeftPis = new Solenoid(Constants.kLeftMagazinePis);
  public Solenoid magazineRightPis = new Solenoid(Constants.kRightMagazinePis);




  public void Magazine() {
  

  }
  //Magazine Conveyor Motors
  public void magTiltForward(){
    magazineTilt.set(ControlMode.PercentOutput,-0.6);
  }
  public void magTiltBackward(){
    magazineTilt.set(ControlMode.PercentOutput,0.2);
  }
  
  //Magazine "Left" and "Right" Belt Lift Pistons
  public void magDisengage(){
    magazineLeftPis.set(true);
    magazineRightPis.set(true);
  }
  public void magEngage(){
    magazineLeftPis.set(false);
    magazineRightPis.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}