

package frc.robot.subsystems;

import static frc.robot.Constants.ID_SPINNER_MOTOR;
import static frc.robot.Constants.kCurrentLimit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ColorSpinnerSubsystem extends SubsystemBase {
  //declares motor and piston
  CANSparkMax colorSpinnerMotor;
  public Solenoid colorSpinerPiston;

  public TalonSRX spinnerMotor = new TalonSRX(Constants.ID_SPINNER_MOTOR);
//define motor and piston
  public ColorSpinnerSubsystem() {
  colorSpinnerMotor = new CANSparkMax(ID_SPINNER_MOTOR, MotorType.kBrushless);
  colorSpinerPiston = new Solenoid(kColorSpinnerPiston);
  colorSpinnerMotor.restoreFactoryDefaults();
  colorSpinnerMotor.setInverted(false);
  colorSpinnerMotor.setSmartCurrentLimit(kCurrentLimit);
  }
//Set speed for Color Spinner direction.
//spins colorspinner motor Counter Clockwise
  public void spinL() {
    spinnerMotor.set(ControlMode.PercentOutput,-0.4);
  }
//spins colorspinner motor Clockwise
  public void spinR(){
    spinnerMotor.set(ControlMode.PercentOutput,0.4);
  }
//stops motor
  public void stop(){
    spinnerMotor.set(ControlMode.PercentOutput,0.0);
  }
 //extends piston
  public void extend(){
    colorSpinerPiston.set(true);
  }
//retracts piston
  public void retract(){
    colorSpinerPiston.set(false);
  }

  public void spin(){
     
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

