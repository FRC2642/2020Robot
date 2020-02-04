

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ColorSpinnerSubsystem extends SubsystemBase {
  //(Charles- remember to comment)
  CANSparkMax colorSpinnerMotor;

  public TalonSRX spinnerMotor = new TalonSRX(Constants.ID_SPINNER_MOTOR);

  public ColorSpinnerSubsystem() {
  colorSpinnerMotor = new CANSparkMax(ID_SPINNER_MOTOR, MotorType.kBrushless);
  colorSpinnerMotor.restoreFactoryDefaults();
  colorSpinnerMotor.setInverted(false);
  colorSpinnerMotor.setSmartCurrentLimit(kCurrentLimit);
  }
//Set speed for Color Spinner direction.
//spins colorspinner motor Counter Clockwise
  public void spinL()
    spinnerMotor.set(ControlMode.PercentOutput,-0.4);
  }
//spins colorspinner motor Clockwise
  public void spinR(){
    spinnerMotor.set(ControlMode.PercentOutput,0.4);
  }

  public void stop(){
    spinnerMotor.set(ControlMode.PercentOutput,0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
