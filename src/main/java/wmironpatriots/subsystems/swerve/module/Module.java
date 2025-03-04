// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import wmironpatriots.util.mechanismUtil.IronComponent;

public abstract class Module extends IronComponent {
  /** MODULE CONSTANTS */
  public static final double PIVOT_REDUCTION = 150 / 7;

  public static final double DRIVE_REDUCTION = 6.21;
  public static final double WHEEL_RADIUS_METERS = 0.0508;

  // configs
  public static final TalonFXConfiguration PIVOT_CONFIG = getPivotConfig();
  public static final TalonFXConfiguration DRIVE_CONFIG = getDriveConfig();
  public static final CANcoderConfiguration CANCODER_CONFIG = getCANcoderConfig();

  // static methods
  protected static TalonFXConfiguration getPivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = PIVOT_REDUCTION;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Slot0.kP = 20.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.68275;
    config.Slot0.kV = 0.42962962963;
    config.Slot0.kA = 0.031543;

    config.MotionMagic.MotionMagicCruiseVelocity = (5500 / 60) / PIVOT_REDUCTION;
    config.MotionMagic.MotionMagicAcceleration = (5500 / 60) / (PIVOT_REDUCTION * 0.005);
    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  protected static TalonFXConfiguration getDriveConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;
    // TODO TUNE VALUES
    config.Slot0.kA = 5.0;
    config.Slot0.kS = 10.0;
    config.Slot0.kP = 300.0;
    config.Slot0.kD = 0.0;

    return config;
  }

  protected static CANcoderConfiguration getCANcoderConfig() {
    return new CANcoderConfiguration();
  }

  public static record ModuleConfig(
      int index,
      int pivotID,
      int driveID,
      int cancoderID,
      double cancoderOffsetRads,
      boolean pivotInverted) {}
  ;

  /** LOGGED VALUES */
  @Log public boolean pivotOk = false;

  @Log public boolean driveOk = false;

  @Log public double pivotABSPoseRads = 0.0;
  @Log public double pivotPoseRads = 0.0;
  @Log public double pivotVelRadsPerSec = 0.0;
  @Log public double pivotAppliedVolts = 0.0;
  @Log public double pivotSupplyCurrent = 0.0;
  @Log public double pivotTorqueCurrent = 0.0;

  @Log public double drivePoseMeters = 0.0;
  @Log public double driveVelMPS = 0.0;
  @Log public double driveAppliedVolts = 0.0;
  @Log public double driveSupplyCurrent = 0.0;
  @Log public double driveTorqueCurrent = 0.0;

  protected final ModuleConfig config;

  protected Module(ModuleConfig config) {
    this.config = config;
  }

  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    Rotation2d currentAngle = getCurrentAngle();
    state.optimize(currentAngle);
    state.speedMetersPerSecond *= Math.cos(state.angle.minus(currentAngle).getRadians());

    runPivotPose(state.angle.getRadians());
    runDriveVel(state.speedMetersPerSecond, 0.0);
    return state;
  }

  public SwerveModuleState runOpenloopSetpoint(SwerveModuleState state, boolean focEnabled) {
    Rotation2d currentAngle = getCurrentAngle();
    state.optimize(currentAngle);
    state.speedMetersPerSecond *= Math.cos(state.angle.minus(currentAngle).getRadians());

    runPivotPose(state.angle.getRadians());
    runDriveVolts(state.speedMetersPerSecond, focEnabled);
    return state;
  }

  public void stop() {
    stopMotors();
  }

  public void moduleCoastingEnabled(boolean enabled) {
    motorCoastingEnabled(enabled);
  }

  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRadians(pivotPoseRads);
  }

  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(drivePoseMeters, getCurrentAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveVelMPS, getCurrentAngle());
  }

  /** HARDWARE METHODS */
  protected abstract void runPivotVolts(double volts);

  protected abstract void runPivotPose(double poseRads);

  protected void runDriveVolts(double volts) {
    runDriveVolts(volts, true);
  }

  protected abstract void runDriveVolts(double volts, boolean focEnabled);

  protected abstract void runDriveVel(double velMPS, double ff);

  protected abstract void stopMotors();

  protected abstract void motorCoastingEnabled(boolean enabled);
}
