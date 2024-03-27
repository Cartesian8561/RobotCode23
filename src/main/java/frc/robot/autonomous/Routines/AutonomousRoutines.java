// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.Routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.CartesianRamseteClass;
import frc.robot.autonomous.Trajectories.AutonomousTrajectories;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutonomousRoutines {
    private static AutonomousTrajectories m_trajectories;
    private static CartesianRamseteClass m_ramsete;
    private Drivetrain m_drive;


    public AutonomousRoutines(AutonomousTrajectories trajectory, CartesianRamseteClass ramsete, Drivetrain drive){
        m_trajectories = trajectory;
        m_ramsete = ramsete;
        m_drive = drive;
    }

    public Command gamePiece3Command(){
        return m_ramsete.getCartesianRamseteCommand(m_trajectories.goToGamePiece3()).andThen(new InstantCommand(() -> m_drive.tankDriveVolts(0, 0), m_drive));
    }

    public Command leftGrid(){
        return m_ramsete.getCartesianRamseteCommand(m_trajectories.goToLeftConeGridLeft()).andThen(new InstantCommand(() -> m_drive.tankDriveVolts(0, 0), m_drive));
    }
}
