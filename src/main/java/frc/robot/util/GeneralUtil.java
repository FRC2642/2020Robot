package frc.robot.util;

import static frc.robot.Constants.kAngleD;
import static frc.robot.Constants.kAngleFF;
import static frc.robot.Constants.kAngleI;
import static frc.robot.Constants.kAngleP;
import static frc.robot.Constants.kDriveD;
import static frc.robot.Constants.kDriveFF;
import static frc.robot.Constants.kDriveI;
import static frc.robot.Constants.kDriveP;
import static frc.robot.Constants.kMagD;
import static frc.robot.Constants.kMagFF;
import static frc.robot.Constants.kMagI;
import static frc.robot.Constants.kMagP;
import static frc.robot.Constants.kMaxOutput;
import static frc.robot.Constants.kMinOutput;
import static frc.robot.Constants.kMotorNeutralDeadband;
import static frc.robot.Constants.kShooterD;
import static frc.robot.Constants.kShooterFF;
import static frc.robot.Constants.kShooterI;
import static frc.robot.Constants.kShooterP;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

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
      case SHOOTER:
        pid.setFF(kShooterFF);
        pid.setP(kShooterP);
        pid.setI(kShooterI);
        pid.setD(kShooterD);
        break;
    }
    pid.setOutputRange(kMinOutput, kMaxOutput);
  }

  public enum PIDProfile {
        DRIVE,
        ANGLE,
        MAGAZINE,
        SHOOTER,
  }

  public static Command generateAuto(){

      Command auto = new InstantCommand(
        () -> RobotContainer.magazine.setMagPis(true)
        ); 


      /* Command auto = new RunCommand(
        () -> RobotContainer.shooter.setShooterSpeed(kInitLineVelocity)
        ).alongWith()

        () -> RobotContainer.magazine.setToShootingState(), RobotContainer.magazine
      )
      .alongWith(
        RobotContainer.autoArmTo
        
      )
      .andThen( new InstantCommand(
        //drive::alignWheels
        () -> RobotContainer.drive.alignWheels()
          )
        ); */
       /*.andThen( new WaitCommand(.5)
      )
      .andThen( new RunCommand(
        () -> RobotContainer.drive.drive(-.3, 0, 0), RobotContainer.drive
          )
        )
      .withTimeout(1)
      .andThen( new RunCommand(
        () -> RobotContainer.drive.drive(0, 0, 0), RobotContainer.drive
          )
        ); */ 

      return auto; 
  }
}

