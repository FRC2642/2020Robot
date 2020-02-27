/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.armTilt;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ArmToClimbPosition extends PIDCommand {
  
  public static ArmSubsystem arm;

  public ArmToClimbPosition(ArmSubsystem armSub) {
    super(
        // The controller that the command will use
        new PIDController(kTiltP, kTiltI, kTiltD),
        // This should return the measurement
        () -> arm.getPot(),
        // This should return the setpoint (can also be a constant)
        () -> kClimbPos,
        // This uses the output
        output -> {
          arm.moveArm(output);
          // Use the output here
        });
            
    arm = armSub;
    addRequirements(arm);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isManualOverride();
  }
}
