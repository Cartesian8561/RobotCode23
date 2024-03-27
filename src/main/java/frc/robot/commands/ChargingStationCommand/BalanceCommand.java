// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ChargingStationCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PitchConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
  /** Creates a new ChargingStationCommand. */
  private static Drivetrain m_drive;
  private PIDController pitchControler;
  private PIDController levelController;
  private boolean finished = false;
  private double startingEncoderDistance; 
  private boolean gate = false;


  private long startTime;
  private long currentTime;

  private double startingX;

  public BalanceCommand(Drivetrain drivetrain) {
    m_drive = drivetrain;
    pitchControler = new PIDController(PitchConstants.pitchkp, 0, PitchConstants.pitchkd);
    levelController = new PIDController(PitchConstants.levelkp, 0, 0);
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    startingEncoderDistance = m_drive.getLeftEncoderDistance();
    finished = false;
    gate = false;
    m_drive.setMaxOutput(0.5);
    startingX = m_drive.getEstimatedPose().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = System.currentTimeMillis();
    if(startTime - currentTime <= 100){
      m_drive.tankDriveVolts(5.5, 5.5);
    }else{
      if(m_drive.getEstimatedPose().getX() - startingX <= 0.5){
        //işte geri gitme şeysi
        new BackCommand(m_drive).schedule();
        startingX = m_drive.getEstimatedPose().getX();
        m_drive.getLeftEncoderDistance();
        startTime = System.currentTimeMillis();
      }
      else{
        if(!m_drive.isBalanced() && !gate){
          startingEncoderDistance = m_drive.getLeftEncoderDistance();
          gate = true;
        }
    
        double distanceOnPlatform = Math.abs(m_drive.getLeftEncoderDistance() - startingEncoderDistance);
    
        
        if(!m_drive.isBalanced() &&  distanceOnPlatform < PitchConstants.platformSize){
    
          double currentDistanceToCenter = (Math.abs(PitchConstants.platformSize/2 - distanceOnPlatform));
          double distanceCoefficient = currentDistanceToCenter * 2.0 / PitchConstants.platformSize;
    
          if(m_drive.getPitch() > 10.0){
    
            double command = -pitchControler.calculate(m_drive.getPitch(), 0) * distanceCoefficient;
            m_drive.tankDriveVolts(command, command);
          }else{
            double command = -levelController.calculate(m_drive.getPitch(), 0) * distanceCoefficient;
            m_drive.tankDriveVolts(command, command);
          }
        }else{
          m_drive.tankDriveVolts(0.0, 0.0);
        }
      }
    }
      


   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDriveVolts(0.0, 0.0);
    m_drive.setMaxOutput(1.0);
    gate = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
