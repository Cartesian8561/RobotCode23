// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class CloseArm extends CommandBase {

  private static final PIDController closePID = new PIDController(10, 2.11, 0.0);
  
  //private static final ArmFeedforward feedforward = new ArmFeedforward(0.84271, 0.98, 3.44);

  private ArmSubsystem armSubsystem;
  private double setPoint;
  private boolean isFinished = false;

  public CloseArm(ArmSubsystem armSubsystem, double setPoint) {
    this.setPoint = setPoint;
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);
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
    armSubsystem.updateEncoder();
    if(Math.abs(setPoint - ArmSubsystem.getArmAngle()) > 0.2){
        double command = -closePID.calculate(ArmSubsystem.getArmAngle(), setPoint);
        if(command > 5.0) {command = 5.0;}
        armSubsystem.setAngleMotorVolts(command);
      }else{
        isFinished = true;
        armSubsystem.zeroAllOutputs();
      }

      //olmazsa bunu dene
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.zeroAllOutputs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
