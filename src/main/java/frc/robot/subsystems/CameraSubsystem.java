// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class CameraSubsystem extends SubsystemBase {

  private static PhotonCamera camera;
  /** Sets up the Photonvision cameras */
  public CameraSubsystem() {
    camera = new PhotonCamera(VisionConstants.cameraName);
  }

  /** Sets up the Photonvision cameras and prints out any errors */
  public void setCamera(){
    try{
      camera = new PhotonCamera(VisionConstants.cameraName);
    }catch(Exception e){
      System.out.println(e.getCause());
    }

  }

  /**
   * @return The PhotonVision Camera
   */
  public static PhotonCamera getCamera(){
    return camera;
  }

  /**
   * @return True if the camera sees any targets.
   */
  public static boolean hasTargets(){
    if(camera != null){
      var results = camera.getLatestResult();
      return results.hasTargets();
    }else{
      return false;
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
