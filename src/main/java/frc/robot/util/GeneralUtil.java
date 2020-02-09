package frc.robot.util;

import static frc.robot.Constants.*;

import com.revrobotics.CANPIDController;

public class GeneralUtil {

     /**
   * Applies a deadband to raw joystick input
   * 
   * @param input raw joystick input
   * @return deadbanded joystick input
   */
  public static double deadband(double input){
    double outMax = 1.0;
    double outMin = -1.0;
    double inMax = 1.0;
    double inMin = -1.0; 

    double output = 0.0;
    
    if(input <= kMotorNeutralDeadband && input >= (-kMotorNeutralDeadband)){
      output = 0.0;
    }
    if(input >= kMotorNeutralDeadband){
                //new slope for motor output                 //repositions constant based on deadband
      output = (outMax / (inMax - kMotorNeutralDeadband)) * (input - kMotorNeutralDeadband);
    }
    if(input <= -kMotorNeutralDeadband){
               //new slope for motor output                  //repositions constant based on deadband
      output = (outMin / (kMotorNeutralDeadband + inMin)) * (input + kMotorNeutralDeadband);
    }
    
    return output;
  }

   /**
   * Sets PID gains for Spark max controllers
   * 
   * @param pid the PID controller to set values for
   * @param profile what gains profile to use
   */
  public static void setPIDGains(CANPIDController pid, PIDProfile profile){
    switch(profile) {
      case DRIVE: 
        pid.setFF(kDriveFF);
        pid.setP(kDriveP);
        pid.setI(kDriveI);
        pid.setD(kDriveD);
        break; 
      case ANGLE:
        pid.setFF(kAngleFF);
        pid.setP(kAngleP);
        pid.setI(kAngleI);
        pid.setD(kAngleD);  
        break;
      case MAGAZINE:
        pid.setFF(kMagFF);
        pid.setP(kMagP);
        pid.setI(kMagI);
        pid.setD(kMagD);
        break;
      case TILT:
        pid.setFF(kTiltFF);
        pid.setP(kTiltP);
        pid.setI(kTiltI);
        pid.setD(kTiltD);
      case SHOOTER:
        pid.setFF(kShooterFF);
        pid.setP(kShooterP);
        pid.setI(kShooterI);
        pid.setD(kShooterD);
        break;
      case CLIMB:
        pid.setFF(kClimbFF);
        pid.setP(kClimbP);
        pid.setI(kClimbI);
        pid.setD(kClimbD);
        break;
      
    }
    pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  public enum PIDProfile {
        DRIVE,
        ANGLE,
        MAGAZINE,
        TILT,
        SHOOTER,
        CLIMB;
  }
}

