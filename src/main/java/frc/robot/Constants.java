// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants for Driving */
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor3Port = 3;
    public static final int kRightMotor4Port = 4;

    public static final int kIntakeAngleMotorPort = 7;
    public static final int kArmAngleMotorPort = 10;
    public static final int kIntakeNeoLeftPort = 11;
    public static final int kIntakeNeoRightPort = 12;

    public static final int[] kLeftEncoderPorts = new int[] {3, 2};
    public static final int[] kRightEncoderPorts = new int[] {1, 0};
    public static final int[] kIntakeEncoderPorts = new int[] {4, 5}; //44.4 pulse per revolution
    
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    public static final double kTrackwidthMeters = 0.52;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // TODO KV 36 DIYOR KESIN YANLIS CALISMICAK
    public static final double ksVolts = 0.84271; // 0.9896
    public static final double kvVoltSecondsPerMeter = 3.1946; // 36.233
    public static final double kaVoltSecondsSquaredPerMeter = 0.64206; // 8.8353

    // TODO Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.45401; //0.19399

    public static final int limitSwitchPort = 6;
  }

  /** Controller port constants */
  public static final class OIConstants {
    public static final int kRightJoystick = 2;
    public static final int kLeftJoystick = 0;
    public static final int kBlueXboxStick = 1;
    public static final int kGrayXboxStick = 3;
  }

  /** Constants for Autonomous */
  public static final class AutoConstants {
    //TODO Example values only - DO NOT USE FOR THE REAL ROBOT
    public static final double kMaxSpeedMetersPerSecond = 0.5;//3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;//6.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  /** Constants for Vision calculations */
  public static final class VisionConstants{
    /** Pose of the camera on the robot */
    public static final Transform3d robotToCam = new Transform3d(
        new Translation3d(-0.34, 0.175, 0.445),
        new Rotation3d(0, 0, Math.PI)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final String cameraName = "Microsoft_LifeCam_HD-3000";
  }

  public static final class PitchConstants{
    public static final double platformSize = 122.0;
    public static final double pitchkp = 0.008;
    public static final double levelkp = 0.002;
    public static final double pitchkd = 0.1;
    
  }

  public static final class LimitConstants{

    public static final double rotDis = Math.PI/2.7 * 1/2;
    public static final double positiveAnglePoint = 4.3; //4.6
    public static final double negativeAnglePoint = -31.2; //0.2

    public static final double positiveDistancePoint = -0.5;
    public static final double negativeDistancePoint = -28.0;
  }

  public static final class IntakeConstants{

  }


//TODO BUNLARI BULACAKSIN
  public static final class ArmPoseConstants{
    public static final double armGridPose = 3.9; //3.7
    public static final double armMidPose = 3.8; 
    public static final double armGroundPose = -0.15;//0.6   0.01
    public static final double armSubstationPose = 0;
    public static final double armStartingPose = 2.4;

  }

  public static final class IntakePoseConstants{
    public static final double intakeGroundCubePose = -1.2; //0.9
    public static final double intakeGroundConePose = -1.4;
    public static final double intakeMidCubePose = -0.1; 
    public static final double intakeMidConePose = 0; 
    public static final double intakeGridCubePose = -0.2;//-0.14
    public static final double intakeGridConePose = 0;
    public static final double intakeSubstationCubePose = 0;
    public static final double intakeSubstationConePose = 0;
    public static final double intakeStartingPosition = -0.5;
  }

  public static final class IntakeStabilizeConstants{
    public static final double intakeGridConeVolt = 1.5;
  }

  public static final class AutoPoseConstants{
    public static final double cubeHighGrid = 1.75;
    public static final double intakeCubeHighGrid = -0.2;
  }

}
