// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ChargingStationCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChargeCommand extends CommandBase {
  /** Creates a new ChargeCommand. */
  private Drivetrain m_drive;
  private PIDController pid = new PIDController(3.6, 0, 0);
  public ChargeCommand(Drivetrain drivetrain) {
    m_drive = drivetrain;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("is balanced", m_drive.isBalanced());
    if(!m_drive.isBalanced()){
      double value = -pid.calculate(m_drive.getPitch());
      m_drive.tankDriveVolts(value , value);
    }else{
      m_drive.tankDriveVolts(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
