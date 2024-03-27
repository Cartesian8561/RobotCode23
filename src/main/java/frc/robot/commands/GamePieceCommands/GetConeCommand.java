// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GamePieceCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class GetConeCommand extends CommandBase {
  /** Creates a new GetConeCommand. */

  private IntakeSubsystem intakeSub;
  private double previousCurrent = -1;
  private PowerDistribution pdp = new PowerDistribution();
  private boolean isFinished = false;
  public GetConeCommand(IntakeSubsystem intakeSubsystem) {
    intakeSub = intakeSubsystem;
    addRequirements(intakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("pdp ampere", pdp.getCurrent(12));//TODO GET THE CHANEL FOR NEO 550 MOTORS

    if(previousCurrent == -1){
      previousCurrent = pdp.getCurrent(0);
    }

    if(previousCurrent - pdp.getCurrent(0) < 0.5){
      intakeSub.setBothIntakeMotorVolts(2.5);
    }else{
      intakeSub.setBothIntakeMotorVolts(0.0);
      isFinished = true;
    }
    
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setBothIntakeMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}