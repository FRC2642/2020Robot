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
  String correctColor;
  String currentColor;
  String colorNeeded;

  public SpinToColor(final ColorSpinnerSubsystem colorSub) {
    spinner = colorSub;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    correctColor = DriverStation.getInstance().getGameSpecificMessage();
    correctColor = correctColor.toUpperCase();
    if (correctColor == "RED"){
      colorNeeded = "BLUE";
    }
    if (correctColor == "GREEN"){
      colorNeeded = "YELLOW";
    }
    if (correctColor == "BLUE"){
      colorNeeded = "RED";
    }
    if (correctColor == "YELLOW"){
      colorNeeded = "GREEN";
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (correctColor == "RED" & currentColor == "YELLOW"){
      spinner.slowSpinL();
    }
    else if (correctColor == "GREEN" & currentColor == "RED"){
      spinner.slowSpinL();
    }
    else if (correctColor == "BLUE" & currentColor == "GREEN"){
      spinner.slowSpinL();
    }
    else if (correctColor == "YELLOW" & currentColor == "BLUE"){
      spinner.slowSpinL();
    }
    else{
      spinner.slowSpinR();
    }
    currentColor = spinner.detectColor();
    currentColor = currentColor.toUpperCase();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (correctColor == null){
      return false;
    }
    if (currentColor == colorNeeded){
      spinner.stop();
      return true;
    }
    else{
      return false;
    }
  }
}
