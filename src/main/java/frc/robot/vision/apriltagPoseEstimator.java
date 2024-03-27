// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CameraSubsystem;

/** Creates a new apriltagPoseEstimator. */
public class apriltagPoseEstimator extends SubsystemBase {

  public PhotonCamera apriltagCamera;
  public PhotonPoseEstimator robotPoseEstimator;


  public apriltagPoseEstimator() {
    AprilTagFieldLayout ApriltagFieldLayout = new AprilTagFieldLayout(FieldConstants.getApriltagPoses(), FieldConstants.fieldLength, FieldConstants.fieldWidth);
    if(CameraSubsystem.hasTargets()){
        apriltagCamera = CameraSubsystem.getCamera();
        var result = apriltagCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
          robotPoseEstimator = new PhotonPoseEstimator(ApriltagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, apriltagCamera, VisionConstants.robotToCam);
      }
    }



    /* var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(apriltagCamera, VisionConstants.robotToCam));
    robotPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, camList);*/
  }


  public void setPoseEstimator(){
    AprilTagFieldLayout ApriltagFieldLayout = new AprilTagFieldLayout(FieldConstants.getApriltagPoses(), FieldConstants.fieldLength, FieldConstants.fieldWidth);
    if(CameraSubsystem.hasTargets()){
        apriltagCamera = CameraSubsystem.getCamera();
        var result = apriltagCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
          robotPoseEstimator = new PhotonPoseEstimator(ApriltagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, apriltagCamera, VisionConstants.robotToCam);
      }
    }
  }


/**
 * Estimates the Robot pose using {@link PhotonPoseEstimator}
 * @param prevEstimatedRobotPose
 * @return The estimated position of the robot in {@link Pose2d}
 */

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if(robotPoseEstimator != null){
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return robotPoseEstimator.update();
    }else{
      setPoseEstimator();
      return null;
    }
  }

}
