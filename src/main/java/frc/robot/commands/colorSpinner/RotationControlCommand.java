/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.colorSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSpinnerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RotationControlCommand extends CommandBase {

  ColorSpinnerSubsystem spinner;
  SwerveDriveSubsystem drive;

  String targetColor;
  boolean hasCounted;

  public RotationControlCommand(final ColorSpinnerSubsystem colorSub) {
    spinner = colorSub;
    addRequirements(spinner);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetColor = spinner.detectColor();
    spinner.zeroCounter();
    System.out.println(spinner.getCounter());
    hasCounted = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinner.spinL();

    //System.out.println(spinner.getCounter());

    if ((spinner.detectColor() == targetColor) && !spinner.getHasCounted()) {
      spinner.counterUp();
      spinner.setHasCounted(true);
    } else if (spinner.detectColor() != targetColor) {
      spinner.setHasCounted(false);
    }
    
    }
 
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    spinner.stop();
    spinner.zeroCounter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return (spinner.getCounter() >= 8); //if it has spun the desired amount of times it stop
  }
}
