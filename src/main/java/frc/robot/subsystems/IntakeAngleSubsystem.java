// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeAngleSubsystem extends SubsystemBase {


  private static final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(7);
  private static final Encoder intakeEncoder = new Encoder(DriveConstants.kIntakeEncoderPorts[0], DriveConstants.kIntakeEncoderPorts[1]);
  
  

  /** Creates a new IntakeAngleSubsystem. */
  public IntakeAngleSubsystem() {
    intakeEncoder.setDistancePerPulse(1/44.4);
  }

  public double getIntakeRate(){
    return  intakeEncoder.getRate();
  }

  public double getIntakeDistance(){
    return intakeEncoder.getDistance();
  }

  public void setIntakeVolts(double volts){
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Voltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("intake rate", getIntakeRate());
    SmartDashboard.putNumber("intake encoder", intakeEncoder.getDistance());
  }

}
