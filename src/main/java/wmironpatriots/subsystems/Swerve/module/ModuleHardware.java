// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.module;

/** Generalized hardware methods for a swerve-module's hardware */
public interface ModuleHardware {
  /**
   * Represents a set of measured values from module hardware
   *
   * @param pivotIsOk Is pivot motor measuring correctly?
   * @param pivotRevs Position measurement of pivot motor in revolutions (relative encoder)
   * @param pivotSetpointPoseRevs Position setpoint of pivot motor in revolutions
   * @param pivotAppliedVolts Output voltage of pivot motor in volts
   * @param pivotStatorAmps Stator current of pivot motor in amps
   * @param pivotTorqueAmps Torque output of pivot motor in amps
   * @param driveIsOk Is drive motor measuring correctly?
   * @param driveMps Speed measurement of the drive motor in Meters/Second
   * @param drivePose Distance driven in meters
   * @param driveSetpointMps Speed setpoint of drive motor in Meters/Second
   * @param driveAppliedVolts Output voltage of drive motor in volts
   * @param driveStatorAmps Stator current of drive motor in amps
   * @param driveTorqueAmps Torque output of drive motor in amps
   * @param cancoderIsOk Is CANcoder measuring correctly?
   * @param cancoderRevs Position measurement of pivot motor in revolutions
   */
  public static record LoggableState(
      boolean pivotIsOk,
      double pivotRevs,
      double pivotSetpointPoseRevs,
      double pivotAppliedVolts,
      double pivotStatorAmps,
      double pivotTorqueAmps,
      boolean driveIsOk,
      double drivePose,
      double driveMps,
      double driveSetpointMps,
      double driveAppliedVolts,
      double driveStatorAmps,
      double driveTorqueAmps,
      boolean cancoderIsOk,
      double cancoderRevs) {}

  /**
   * @return {@link LoggableState} representing the latest hardware measurements
   */
  public LoggableState getLoggableState();

  /**
   * Set pivot motor setpoint voltage
   *
   * @param volts Desired voltage
   */
  public void setPivotAppliedVolts(double volts);

  /**
   * Set drive motor setpoint voltage
   *
   * @param volts Desired voltage
   */
  public void setDriveAppliedVolts(double volts);

  /**
   * Set pivot motor setpoint position
   *
   * @param poseRevs Desired position in revs
   */
  public void setPivotSetpointPose(double poseRevs);

  /**
   * Set drive motor setpoint speed
   *
   * @param speedMps Desired speed in Meters/Second
   */
  public void setDriveSetpointSpeed(double speedMps);

  /** Stop all module input */
  public void stop();

  /** Enable or disable module coasting for easier movement */
  public void coastingEnabled(boolean enabled);
}