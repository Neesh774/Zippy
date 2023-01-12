// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnToTag extends PIDCommand {
  /** Creates a new trackAprilTag. */
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  public turnToTag(SwerveSubsystem swerveSubsystem, Vision visionSubsystem) {
    // pid command that tries to turn the angle between our heading and the april
    // tag to 0
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0),
        // This should return the measurement
        () -> visionSubsystem.getResult().hasTargets() ? visionSubsystem.getResult().getBestTarget().getYaw() : 0,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        turningSpeed -> {
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
          SlewRateLimiter turningLimiter = new SlewRateLimiter(
              DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
          turningSpeed = turningLimiter.calculate(turningSpeed)
              * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
          System.out.println("Turning Speed: " + turningSpeed);
          ChassisSpeeds chassisSpeeds;
          chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          swerveSubsystem.setModuleStates(moduleStates);
        });
    this.m_controller.setTolerance(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Is finished");
    return this.m_controller.atSetpoint();
  }
}
