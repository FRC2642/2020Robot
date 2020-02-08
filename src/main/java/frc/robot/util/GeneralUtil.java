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
   * Sets PID terms for swerve module controllers
   * 
   * @param pid the PID controller to set values for
   * @param drive is module a drive module (if false, angle module)
   */
  public static void setPIDTerms(CANPIDController pid, PIDProfile profile){
    switch(profile) {
      case DRIVE: 
        pid.setP(kDriveP);
        pid.setI(kDriveI);
        pid.setD(kDriveD);
        pid.setFF(kDriveFF);
        break; 
      case ANGLE:
        pid.setP(kAngleP);
        pid.setI(kAngleI);
        pid.setD(kAngleD);
        pid.setFF(kAngleFF);
        break;
      case MAGAZINE:
        pid.setP(kMagP);
        pid.setI(kMagI);
        pid.setD(kMagD);
        pid.setFF(kMagFF);
        break;
      case SHOOTER:
        pid.setP(kShooterP);
        pid.setI(kShooterI);
        pid.setD(kShooterD);
        pid.setFF(kShooterFF);
        break;
      case CLIMB:
        pid.setP(kClimbP);
        pid.setI(kClimbI);
        pid.setD(kClimbD);
        pid.setFF(kClimbFF);
        break;
      
    }
    pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  public enum PIDProfile {
        DRIVE,
        ANGLE,
        MAGAZINE,
        SHOOTER,
        CLIMB;
  }
}

