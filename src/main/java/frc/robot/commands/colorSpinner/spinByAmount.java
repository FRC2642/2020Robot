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

public class spinByAmount extends CommandBase {

  public boolean notDetected = true;
  ColorSpinnerSubsystem spinner;
  SwerveDriveSubsystem drive;
  public String Color;
  double spin; 
  public int counter = 0; 

  public spinByAmount(final ColorSpinnerSubsystem colorSub, final SwerveDriveSubsystem driveSub){
    spinner = colorSub;
    drive = driveSub;
    addRequirements(spinner, drive);
  }
  //creates a counter that detects how many times the wheel spins
  public boolean spinnerCounter(){ 
      counter++;
      if(counter >= 7)
        return false;
      else{
        return true;
      }
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Color = spinner.detectColor();
    notDetected = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinner.spinL();
    if (spinner.detectColor() == Color & !notDetected){
      spinnerCounter();
      notDetected = true;
    }
    else if(spinner.detectColor() != Color){
      notDetected = false;
    }
    }
 
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (counter >= 7){ //if it has spun the desired amount of times it stops
      spinner.slowStop();

      return true;
    }
    else{
      return false;
    }


    

  }
}
