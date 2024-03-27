// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeAngleSubsystem;

public class LowerOpenIntake extends CommandBase {

  private IntakeAngleSubsystem intakeSub;
  private double setPoint;
  private PIDController groundController = new PIDController(60, 0, 5.0);
  private boolean isFinished = false;
  private double stabilizer;


  /** Creates a new OpenIntake. */
  public LowerOpenIntake(IntakeAngleSubsystem intakeSubsystem, double setPoint, double stableVolts) {
    intakeSub = intakeSubsystem;
    this.setPoint = setPoint;
    this.stabilizer = stableVolts;
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
    if(Math.abs(setPoint - intakeSub.getIntakeDistance()) > 0.2){
       intakeSub.setIntakeVolts(-groundController.calculate(intakeSub.getIntakeDistance(), setPoint)); 
    }else{
      intakeSub.setIntakeVolts(stabilizer);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setIntakeVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
