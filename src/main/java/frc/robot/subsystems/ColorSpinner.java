

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSpinner extends SubsystemBase {
  //(Charles- remember to comment)

  public TalonSRX spinnerMotor = new TalonSRX(Constants.ID_SPINNER_MOTOR);

  public ColorSpinner() {

  }
//Set speed for Color Spinner direction.
//spins colorspinner motor Counter Clockwise
  public void spinL(){
    spinnerMotor.set(ControlMode.PercentOutput,-0.6);
  }
//spins colorspinner motor Clockwise
  public void spinR(){
    spinnerMotor.set(ControlMode.PercentOutput,0.6);
  }

  public void stop(){
    spinnerMotor.set(ControlMode.PercentOutput,0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
