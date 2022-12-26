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
import frc.robot.commands.turnToTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private PhotonCamera camera;

  public Vision() {
    this.camera = new PhotonCamera("Global_Shutter_Camera");
  }

  @Override
  public void periodic() {
    PhotonPipelineResult res = getResult();
    SmartDashboard.putNumber("Number Vision Targets", res.getTargets().size());
    SmartDashboard.putBoolean("Has Targets", res.hasTargets());
  }

  public PhotonPipelineResult getResult() {
    return camera.getLatestResult();
  }

  public Runnable turnToTag(SwerveSubsystem swerveSubsystem) {
    PhotonPipelineResult res = getResult();
    if (res.hasTargets()) {
      return () -> new turnToTarget(swerveSubsystem,
          res.getBestTarget().getYaw());
    } else {
      return () -> {
      };
    }
  }
}
