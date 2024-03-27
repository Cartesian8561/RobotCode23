
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.Trajectories;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * A class where all the trajectories are stored
 */
public class AutonomousTrajectories {

    // Create a voltage constraint to ensure we don't accelerate too fast
    private final DifferentialDriveVoltageConstraint autoVoltageConstraint;
/*  var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);*/


    // Create config for trajectory
    private final TrajectoryConfig configForward;
    private final TrajectoryConfig configBackward;
    private final Drivetrain m_robotDrive;

    public AutonomousTrajectories(Drivetrain m_subsystem){
        autoVoltageConstraint  = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

        configForward  = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        configBackward  = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);
        m_robotDrive = m_subsystem;
    }

    public Trajectory goToGamePiece3(){
        return TrajectoryGenerator.generateTrajectory(FieldConstants.allianceFlip(new Pose2d(2.898, 4.459, new Rotation2d(1.204))), List.of(), FieldConstants.allianceFlip(new Pose2d(6.788, 4.588, new Rotation2d())), configForward);
    }

    public Trajectory goToLeftConeGridLeft(){
        return TrajectoryGenerator.generateTrajectory(FieldConstants.allianceFlip(new Pose2d(6.745, 4.996, new Rotation2d())), List.of(), FieldConstants.allianceFlip(new Pose2d(1.662, 5.050,new Rotation2d())), configBackward);
    }
}
