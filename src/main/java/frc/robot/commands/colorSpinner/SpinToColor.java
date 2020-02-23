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
import edu.wpi.first.wpilibj.DriverStation;

public class SpinToColor extends CommandBase {

  ColorSpinnerSubsystem spinner;
  SwerveDriveSubsystem drive;
  char correctColor;
  String currentColor;
  String colorNeeded;

  public SpinToColor(final ColorSpinnerSubsystem colorSub) {
    spinner = colorSub;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentColor = spinner.detectColor();
    currentColor = currentColor.toUpperCase();

    if (colorNeeded == "RED" & currentColor == "YELLOW"){
      spinner.slowSpinL();
    } else if (colorNeeded == "GREEN" & currentColor == "RED"){
      spinner.slowSpinL();
    } else if (colorNeeded == "BLUE" & currentColor == "GREEN"){
      spinner.slowSpinL();
    } else if (colorNeeded == "YELLOW" & currentColor == "BLUE"){
      spinner.slowSpinL();

    } else{
      spinner.slowSpinR();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentColor == colorNeeded);
  }
}
