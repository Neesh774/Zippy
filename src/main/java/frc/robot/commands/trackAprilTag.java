// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class trackAprilTag extends PIDCommand {
  /** Creates a new trackAprilTag. */
  public trackAprilTag(SwerveSubsystem swerveSubsystem, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0),
        // This should return the measurement
        () -> swerveSubsystem.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        turningSpeed -> {
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
          SlewRateLimiter turningLimiter = new SlewRateLimiter(
              DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
          turningSpeed = turningLimiter.calculate(turningSpeed)
              * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
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
    return this.m_controller.atSetpoint();
  }
}
