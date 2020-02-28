
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ColorSpinnerSubsystem extends SubsystemBase {

  CANSparkMax colorSpinnerMotor;
  public Solenoid colorSpinnerPiston;
  public ColorSensorV3 m_colorSensor;
  public DigitalInput limitSwitch;

  //creates final RGB values for colors
  final ColorMatch m_colorMatcher = new ColorMatch();
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public String colorString;
  ColorMatchResult match;
  Color detectedColor;
  double slowStopSpeed = 0.4;
  double slowingSpeed = 0.005; //change this for diffrent slowing of the motor after rotating
  
  boolean hasCounted;
  char targetColor;

  public ColorSpinnerSubsystem() {
    colorSpinnerMotor = new CANSparkMax(ID_SPINNER_MOTOR, MotorType.kBrushless);
    colorSpinnerMotor.restoreFactoryDefaults();
    colorSpinnerMotor.setInverted(false);
    colorSpinnerMotor.setSmartCurrentLimit(kCurrentLimit);

    colorSpinnerPiston = new Solenoid(kColorSpinnerPistonPort);

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    limitSwitch = new DigitalInput(kColorSpinnerLimitSwitch);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  

    hasCounted = false;
  }


public static boolean Counter() {
  int counter = 0;
  int spins = 0;
  // The contents of this will execute 100 times
  for(spins = 1; spins <= 7; spins++) 
    
      // This increases your counter by 1
      counter++; 
      // Since your counter is declared outside of the loop, it is accessible here
      // so check its value
      if(counter <= 6) { 
        return true;
  
      }else{
        return false;
      }
  
      // At this point, the loop is over, so go to the next iteration
   
}

//Set speed for Color Spinner direction.
//spins colorspinner motor Counter Clockwise
  public void spinL() {
    colorSpinnerMotor.set(-.4);
  }

//slowly spins clockwise
  public void slowSpinR(){
    colorSpinnerMotor.set(.1);
  }
//slowly pins counter clokewise
  public void slowSpinL(){
    colorSpinnerMotor.set(-.1);
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
    colorSpinnerPiston.set(true);
  }

//retracts piston
  public void retract(){
    colorSpinnerPiston.set(false);
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

  public String getColorString(){
    return colorString;
  }

  public char getCurrentColorChar(){
    String color = detectColor();
    return color.charAt(0);
  }

  /**
   * counter for rotation control
   */
  /** */
  public void zeroCounter(){
    counter = 0;
  }

  public int counter = 0;
  public void counterUp(){
    counter++;
  }

  public int getCounter(){
    return counter;
  }

  public boolean getHasCounted(){
    return hasCounted;
  }

  public void setHasCounted(boolean hasCounted){
    this.hasCounted = hasCounted;
  }


  /**
   * position control
   */
  /** */
  public void setTargetColor(){
    char goal = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
    if(goal == 'Y'){
      targetColor = 'G';
    } else if (goal == 'G'){
      targetColor = 'Y';
    } else if(goal == 'B'){
      targetColor = 'R';
    } else if (goal == 'R'){
      targetColor = 'B';
    }
  }

  public boolean isAtColor(){
    char currentColor = getCurrentColorChar();
    return targetColor == currentColor;
  }

  public boolean getLDPad(){
    return RobotContainer.auxController.getPOV() == 270;
  }

  public boolean getRDPad(){
    return RobotContainer.auxController.getPOV() == 90;
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", detectColor());
    SmartDashboard.putNumber("targetColor", targetColor);
    SmartDashboard.putBoolean("atColor", isAtColor());
  }
}

