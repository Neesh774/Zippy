// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private PhotonCamera camera;

  public Vision() {
    this.camera = new PhotonCamera("photonvision");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Number Vision Targets", getResult().getTargets().size());
  }

  public PhotonPipelineResult getResult() {
    return camera.getLatestResult();
  }
}
