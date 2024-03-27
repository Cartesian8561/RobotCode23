// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArm extends CommandBase {
  /** Creates a new ExtendArm. */
  private ArmSubsystem armSub;
  private boolean isFinished = false;
  private double setPoint;
  private PIDController pid = new PIDController(1.3, 2.7, 0.0);//1.3, -2.7, -2.1

  public ExtendArm(ArmSubsystem armSubsystem, double setPoint) {
    this.setPoint = setPoint;
    armSub = armSubsystem;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(setPoint - armSub.getArmEncoderDistance()) > 2.0){
      armSub.setLengthMotorVolts(pid.calculate(armSub.getArmEncoderDistance(), setPoint));
    }else{
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.setLengthMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}