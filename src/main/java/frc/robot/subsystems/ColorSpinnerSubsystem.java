

package frc.robot.subsystems;

import static frc.robot.Constants.ID_SPINNER_MOTOR;
import static frc.robot.Constants.kColorSpinnerPistonPort;
import static frc.robot.Constants.kCurrentLimit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSpinnerSubsystem extends SubsystemBase {

  CANSparkMax colorSpinnerMotor;
  public Solenoid colorSpinerPiston;
  public ColorSensorV3 m_colorSensor;

  //creates final RGB values for colors
  final ColorMatch m_colorMatcher = new ColorMatch();
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  String colorString;
  ColorMatchResult match;
  Color detectedColor;
  double slowStopSpeed = 0.4;
  double slowingSpeed = 0.005; //change this for diffrent slowing of the motor after rotating
  

  public ColorSpinnerSubsystem() {
    colorSpinnerMotor = new CANSparkMax(ID_SPINNER_MOTOR, MotorType.kBrushless);
    colorSpinnerMotor.restoreFactoryDefaults();
    colorSpinnerMotor.setInverted(false);
    colorSpinnerMotor.setSmartCurrentLimit(kCurrentLimit);

    colorSpinerPiston = new Solenoid(kColorSpinnerPistonPort);

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
  }

//Set speed for Color Spinner direction.
//spins colorspinner motor Counter Clockwise
  public void spinL() {
    colorSpinnerMotor.set(-.4);
  }

//spins colorspinner motor Clockwise
  public void spinR(){
    colorSpinnerMotor.set(.4);
  }

//stops motor
  public void stop(){
    colorSpinnerMotor.set(0.0);
  }
//slow stop
  public void slowStop(){
    while (slowStopSpeed != 0);
    colorSpinnerMotor.set(slowStopSpeed);
    slowStopSpeed = slowStopSpeed - slowingSpeed;
  }

 //extends piston
  public void extend(){
    colorSpinerPiston.set(true);
  }

//retracts piston
  public void retract(){
    colorSpinerPiston.set(false);
  }

  public String detectColor(){
    
    detectedColor = m_colorSensor.getColor();
    match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    return colorString;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}

