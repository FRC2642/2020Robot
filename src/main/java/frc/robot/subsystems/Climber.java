/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
/**
 * Add your docs here.
 */
CANSparkMax climberMotor;
    public Climber() {
        climberMotor = new CANSparkMax(ID_CLIMBER_MOTOR, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        climberMotor.setInverted(false);
        climberMotor.setSmartCurrentLimit(kCurrentLimit);
    }
}