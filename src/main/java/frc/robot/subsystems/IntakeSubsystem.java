// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private static final CANSparkMax intakeLeft = new CANSparkMax(DriveConstants.kIntakeNeoLeftPort, MotorType.kBrushless);
  private static final CANSparkMax intakeRight = new CANSparkMax(DriveConstants.kIntakeNeoRightPort, MotorType.kBrushless);
  public IntakeSubsystem() {}
  
  public void setIntakeLeftVolts(double volts){
    intakeLeft.setVoltage(volts);
  }
  
  public void setIntakeRightVolts(double volts){
    intakeRight.setVoltage(volts);
  }

  public void setBothIntakeMotorVolts(double volts){
    intakeRight.setVoltage(volts);
    intakeLeft.setVoltage(volts);
  }

  public void zeroAllOutputs(){
    intakeLeft.set(0);
    intakeRight.set(0);
  }

  @Override
  public void periodic() {}
}
