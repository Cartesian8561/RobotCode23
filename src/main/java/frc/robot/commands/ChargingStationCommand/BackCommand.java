// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ChargingStationCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BackCommand extends CommandBase {
  /** Creates a new BackCommand. */
  private Drivetrain m_drive;
  private double startingDistance;
  private boolean finished = false;
  private double currentX;

  public BackCommand(Drivetrain m_drive) {
    this.m_drive = m_drive;
    addRequirements(this.m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    startingDistance = m_drive.getEstimatedPose().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = m_drive.getEstimatedPose().getX();
    m_drive.tankDriveVolts(-5.5, -5.5);
    if(startingDistance - currentX >= 50){
      finished = true;
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
