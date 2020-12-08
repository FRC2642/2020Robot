/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.armTilt;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ArmToSetPosition extends ProfiledPIDCommand {
  
  public static ArmSubsystem arm; 

  public ArmToSetPosition(double target, ArmSubsystem armSub) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            kTiltP, kTiltI, kTiltD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(kTiltMaxVel, kTiltMaxAccel)),
        // This should return the measurement
        () -> arm.getPot(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(target, 0),
        // This uses the output
        (output, setpoint) -> {
          arm.setTarget(target);
          arm.moveArm(output);
          // Use the output (and setpoint, if desired) here
        });

    arm = armSub;
    addRequirements(arm);


    //getController().setTolerance(.25);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal() || arm.getManualOverride();
  }
}
