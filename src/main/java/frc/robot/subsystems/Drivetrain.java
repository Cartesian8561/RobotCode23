// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.vision.apriltagPoseEstimator;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushed);
  private final CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushed);
  private final CANSparkMax rightMotor3 = new CANSparkMax(DriveConstants.kRightMotor3Port, MotorType.kBrushed);
  private final CANSparkMax rightMotor4 = new CANSparkMax(DriveConstants.kRightMotor4Port, MotorType.kBrushed);
  private int hasVision = 0;
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(leftMotor1, leftMotor2);


  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(rightMotor3, rightMotor4);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder



  private final Encoder m_leftEncoder = new Encoder(
    DriveConstants.kLeftEncoderPorts[0], 
    DriveConstants.kLeftEncoderPorts[1], 
    DriveConstants.kLeftEncoderReversed); 
   // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
    DriveConstants.kRightEncoderPorts[0],
    DriveConstants.kRightEncoderPorts[1], 
    DriveConstants.kRightEncoderReversed);

  //getDistance() --> getPosition()
  //getRate() --> getVelocity()

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  //PID
  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

  //FeedForward
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);

  public apriltagPoseEstimator apriltagPoseEstimator;

  private final DifferentialDrivePoseEstimator m_poseEstimator =
  new DifferentialDrivePoseEstimator(
          DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d());






  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead

    
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightMotor3.setInverted(true);
    m_rightMotors.setInverted(true);
    m_leftMotors.setInverted(true);

    apriltagPoseEstimator = new apriltagPoseEstimator();
    m_rightEncoder.reset();
    m_leftEncoder.reset();
    m_gyro.reset();

    
    
    resetEncoders();
  }



  
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
            m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
            m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftMotors.setVoltage(leftOutput + leftFeedforward);
    m_rightMotors.setVoltage(rightOutput + rightFeedforward);
}

public void resetVision(){
  hasVision = 0;
}
  @Override
  public void periodic() {
    updateOdometry();
    // Update the odometry in the periodic block

    

    SmartDashboard.putNumber("estimated rotation2d radians", m_poseEstimator.getEstimatedPosition().getRotation().getRadians());
    SmartDashboard.putNumber("gyro rotation2d radians", m_gyro.getRotation2d().getRadians());
    SmartDashboard.putNumber("robot X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("robot Y", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("right Encoder position", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("left Encoder position", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("left velocity", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("right velocity", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("pitch", getPitch());

  }

    /**
   * Updates the DifferentialDrivePoseEstimator position and if there is an apriltag seen, adds vision measurement to the Kalman Filters
   */
  
   public void updateOdometry() {
    m_poseEstimator.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    Optional<EstimatedRobotPose> result = apriltagPoseEstimator.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
    if(result != null && hasVision != 250){
      if (result.isPresent()) {
        hasVision++;
        EstimatedRobotPose camPose = result.get();
        m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
    }else{
      apriltagPoseEstimator.setPoseEstimator();
    }
    if(hasVision > 250) hasVision = 250;
}


  public void setMotor3Volts(double volts){
    leftMotor1.setVoltage(volts);
    rightMotor3.setVoltage(-volts);
    m_drive.feed();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    //resetEncoders();
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void tankOrArcadeDrive(double fwd, double rot, double leftY, double rightY, boolean isTank, int reverser){
    if(isTank){
      if(reverser == 1){
        m_drive.tankDrive(rightY, leftY);
      }else{
       m_drive.tankDrive(-leftY, -rightY);       
      }
    }else{
      m_drive.arcadeDrive(reverser * fwd, -rot);
    }
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    /*if(leftVolts > 0){
      m_leftMotors.setVoltage(leftVolts + 1.5);
      m_rightMotors.setVoltage(rightVolts);
      m_drive.feed();
    }
    else if(leftVolts < 0){
      m_leftMotors.setVoltage(leftVolts - 1.5);
      m_rightMotors.setVoltage(rightVolts);
      m_drive.feed();
    }else{
      */
      m_leftMotors.setVoltage(leftVolts);
      m_rightMotors.setVoltage(rightVolts);
      m_drive.feed();
    //}
    }


  /** Resets the drive encoders to currently read a position of 0. */

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void resetGyro(){
    m_gyro.reset();
  }


  public double getLeftEncoderDistance(){
    return m_leftEncoder.getDistance();
  }

  public double getPitch(){
    return m_gyro.getRoll();
  }

  public boolean isBalanced(){
    if(Math.abs(getPitch()) < 0.5){
      return true;
    }else{
      return false;
    }
  }

  public SimpleMotorFeedforward getFeedforward(){
    return m_feedforward;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }  
}
