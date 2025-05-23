// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import wmironpatriots.subsystems.Swerve.module.ModuleHardware.LoggableState;

public class Module {
  private final ModuleHardware hardware;

  public LoggableState loggableState =
      new LoggableState(false, 0, 0, 0, 0, 0, false, 0, 0, 0, 0, 0, 0, false, 0);

  public Module(ModuleHardware hardware) {
    this.hardware = hardware;
  }

  /** Periodic Swerve Module logic */
  public void periodic() {
    loggableState = hardware.getLoggableState();
  }

  /**
   * Set angle and speed setpoints of the module
   *
   * @param setpointState {@link SwerveModuleState} representing the desired angle and speed of the
   *     module
   * @return {@link SwerveModuleState} representing the applied optimized setpoints
   */
  public SwerveModuleState setSetpoints(SwerveModuleState setpointState) {
    // Minimize the change in heading
    setpointState.optimize(getRotation2d());

    // Decrease drive speed based on distance to angle setpoint (reduces thread wear)
    setpointState.speedMetersPerSecond *= setpointState.angle.minus(getRotation2d()).getCos();

    hardware.setDriveSetpointSpeed(setpointState.speedMetersPerSecond);
    hardware.setPivotSetpointPose(setpointState.angle.getRotations());

    return setpointState;
  }

  /** Stop all module movement */
  public void stop() {
    hardware.stop();
  }

  /**
   * @return {@link Rotation2d} representing the angle of the module
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(loggableState.cancoderRevs());
  }

  /**
   * @return {@link SwerveModuleState} representing the measured position and speed of the module
   */
  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(loggableState.driveSetpointMps(), getRotation2d());
  }

  /**
   * @return {@link SwerveModulePosition} representing the measured field pose of the module
   */
  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(loggableState.drivePose(), getRotation2d());
  }
}
