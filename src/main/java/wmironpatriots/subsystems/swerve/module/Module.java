// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations.Log;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystemComponent;

public abstract class Module extends LoggedSubsystemComponent {
  // * CONSTANTS
  // mech constants
  public static final double PIVOT_REDUCTION = 0.0;
  public static final double DRIVE_REDUCTION = 0.0;
  public static final double WHEEL_RADIUS_METERS = 2.7;

  public static record ModuleConfig(
      int pivotID,
      int driveID,
      int cancoderID,
      double cancoderOffsetRevs,
      boolean cancoderFlipped) {}

  public static TalonFXConfiguration getPivotConf() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    return conf;
  }

  public static TalonFXConfiguration getDriveConf() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    return conf;
  }

  public static CANcoderConfiguration getCancoderConf() {
    CANcoderConfiguration conf = new CANcoderConfiguration();
    return conf;
  }

  // * LOGGED VALUES
  @Log public double pivotPoseRevs;
  @Log public double pivotAppliedVolts;
  @Log public double pivotCurrentAmps;
  @Log public double pivotTorqueAmps;
  @Log public double drivePoseMeters;
  @Log public double driveVelMPS;
  @Log public double driveAppliedVolts;
  @Log public double driveCurrentAmps;
  @Log public double driveTorqueAmps;

  private SwerveModuleState prevState = new SwerveModuleState();
  private double prevTime;

  /**
   * Optimizes and runs desired swerve module state
   *
   * @param setpoint desired state
   * @return optimized state
   */
  public SwerveModuleState runModuleSetpoint(SwerveModuleState setpoint) {
    // Cosine compensation; Decreases carpet wear and drift
    setpoint.optimize(getRotation2d());
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(getRotation2d()).getCos();

    setPivotPose(setpoint.angle.getRotations());
    setDriveVel(
        setpoint.speedMetersPerSecond,
        (setpoint.speedMetersPerSecond - prevState.speedMetersPerSecond)
            / (Timer.getFPGATimestamp() - prevTime));

    prevState = setpoint;
    prevTime = Timer.getFPGATimestamp();

    return setpoint;
  }

  /** Stop module */
  public void stopModule() {
    stopMotors();
  }

  /**
   * Get current module state
   *
   * @return SwerveModuleState based on the current module state
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState();
  }

  /**
   * Get current module field pose
   *
   * @return SwerveModulePosition based on current module position
   */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition();
  }

  /**
   * Gets module angle as Rotation2d
   *
   * @return Rotation2d of current angle
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(pivotPoseRevs);
  }

  // * HARDWARE METHODS
  protected abstract void setPivotVolts(double volts);

  protected abstract void setPivotPose(double poseRevs);

  protected abstract void setDriveVolts(double volts);

  protected abstract void setDriveVel(double velMPS, double accelMPSSqrd);

  protected abstract void stopMotors();

  protected abstract void enableCoastMode(boolean enabled);
}
