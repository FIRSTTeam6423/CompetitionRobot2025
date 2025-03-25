// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.LoggedSubsystemComponent;
import monologue.Annotations.Log;

public abstract class Module extends LoggedSubsystemComponent {
  // * LOGGED VALUES
  @Log protected double pivotPoseRevs;
  @Log protected double cancoderPoseRevs;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotCurrentAmps;
  @Log protected double drivePoseMeters;
  @Log protected double driveVelMPS;
  @Log protected double driveAppliedVolts;
  @Log protected double driveCurrentAmps;
  @Log protected double driveTorqueAmps;

  /**
   * Runs module to {@link SwerveModuleState} setpoint
   *
   * @param setpoint desired {@link SwerveModuleState}
   * @return Optimized setpoint state for logging
   */
  public SwerveModuleState runModuleSetpoint(SwerveModuleState setpoint) {
    // Cosine compensation; Decreases carpet wear and drift
    setpoint.optimize(getRotation2d());
    setpoint.speedMetersPerSecond *= Math.cos(setpoint.angle.minus(getRotation2d()).getRadians());

    setPivotPose(setpoint.angle.getRotations());
    setDriveVel(setpoint.speedMetersPerSecond);

    return setpoint;
  }

  /**
   * @return module angle as {@link Rotation2d}
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(pivotPoseRevs);
  }

  /**
   * @return current {@link SwerveModuleState}
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveVelMPS, getRotation2d());
  }

  /**
   * @return current {@link SwerveModulePosition}
   */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(drivePoseMeters, getRotation2d());
  }

  // * HARDWARE METHODS
  /**
   * Run module with specified drive amps at specified angle for module characterization
   *
   * @param pivotPose Desired module angle {@link Rotation2d}
   * @param driveAmps Desired drive output in amps
   */
  public void runCharacterizationAmps(Rotation2d pivotPose, double driveAmps) {
    setPivotPose(pivotPose.getRotations());
    setDriveCurrent(driveAmps, true);
  }

  /**
   * Run module with specified drive and pivot amps for module characterization
   *
   * @param pivotAmps Desired pivot output in amps
   * @param driveAmps Desired drive output in amps
   */
  public void runCharacterizationAmps(double pivotAmps, double driveAmps) {
    setPivotCurrent(pivotAmps);
    setDriveCurrent(driveAmps, true);
  }

  protected abstract void setPivotCurrent(double amps);

  protected abstract void setDriveCurrent(double amps, boolean focEnabled);

  protected abstract void setPivotPose(double poseRevs);

  protected abstract void setDriveVel(double velMPS);

  /** Stops pivot & drive motor input */
  public abstract void stopMotors();

  protected abstract void enableCoastMode(boolean enabled);
}
