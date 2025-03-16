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
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations.Log;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystemComponent;

public abstract class Module extends LoggedSubsystemComponent {
  // * CONSTANTS
  // mech constants
  public static final double PIVOT_REDUCTION = 21.428571428571427;
  public static final double DRIVE_REDUCTION = 6.122448979591837;
  public static final double WHEEL_RADIUS_METERS = 0.049784;

  public static record ModuleConfig(
      int index,
      int pivotID,
      int driveID,
      int cancoderID,
      double cancoderOffsetRevs,
      boolean pivotInverted) {}

  public static CANcoderConfiguration getCancoderConf(double cancoderOffsetRevs) {
    CANcoderConfiguration conf = new CANcoderConfiguration();
    conf.MagnetSensor.MagnetOffset = cancoderOffsetRevs;

    return conf;
  }

  public static TalonFXConfiguration getPivotConf(int cancoderID, boolean inverted) {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    conf.CurrentLimits.StatorCurrentLimit = 40.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;

    conf.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    conf.TorqueCurrent.PeakReverseTorqueCurrent = 40.0;
    conf.TorqueCurrent.TorqueNeutralDeadband = 0.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    conf.Feedback.FeedbackRemoteSensorID = cancoderID;
    conf.Feedback.SensorToMechanismRatio = PIVOT_REDUCTION;
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    // conf.Feedback.SensorToMechanismRatio = 1.0;

    conf.Slot0.kP = 430.0;
    conf.Slot0.kD = 50.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 10;
    conf.Slot0.kS = 0.014;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.ClosedLoopGeneral.ContinuousWrap = true;

    return conf;
  }

  public static TalonFXConfiguration getDriveConf() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    conf.CurrentLimits.StatorCurrentLimit = 120.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;

    conf.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    conf.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    conf.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION / (WHEEL_RADIUS_METERS * 2 * Math.PI);
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    conf.Slot0.kP = 10000.0;
    conf.Slot0.kD = 0.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 0.0;
    conf.Slot0.kS = 5.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.ClosedLoopGeneral.ContinuousWrap = true;

    return conf;
  }

  // * LOGGED VALUES
  @Log public double pivotPoseRevs;
  @Log public double pivotAppliedVolts;
  @Log public double pivotCurrentAmps;
  @Log public double drivePoseMeters;
  @Log public double driveVelMPS;
  @Log public double driveAppliedVolts;
  @Log public double driveCurrentAmps;
  @Log public double driveTorqueAmps;

  @Log public double[] odoPivotPoseRevsQueue = new double[] {};
  @Log public double[] odoDrivePoseMetersQueue = new double[] {};

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
    setDriveVel(setpoint.speedMetersPerSecond);

    prevState = getModuleState();
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
    return new SwerveModuleState(driveVelMPS, getRotation2d());
  }

  /**
   * Get current module field pose
   *
   * @return SwerveModulePosition based on current module position
   */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(drivePoseMeters, getRotation2d());
  }

  /**
   * Get all module poses since last tick
   *
   * @return {@link SwerveModulePosition} array of all module poses since last tick
   */
  public SwerveModulePosition[] getModulePoses() {
    int minOdometryPositions =
        Math.min(odoDrivePoseMetersQueue.length, odoPivotPoseRevsQueue.length);
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              odoDrivePoseMetersQueue[i], Rotation2d.fromRotations(odoPivotPoseRevsQueue[i]));
    }
    return positions;
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
  /**
   * Run drive motor voltage for characterization
   *
   * @param pivotPose Pivot angle
   * @param driveVolts Drive motor input
   */
  public void runCharacterizationVolts(Rotation2d pivotPose, double driveVolts) {
    setPivotPose(pivotPose.getRotations());
    setDriveVolts(driveVolts, true);
  }

  /**
   * Run drive & pivot motor voltage for characterization
   *
   * @param pivotPose Pivot motor input
   * @param driveVolts Drive motor input
   */
  public void runCharacterizationVolts(double pivotVolts, double driveVolts) {
    setPivotVolts(pivotVolts);
    setDriveVolts(driveVolts, true);
  }

  protected abstract void setPivotVolts(double volts);

  protected abstract void setPivotPose(double poseRevs);

  protected abstract void setDriveVolts(double volts, boolean focEnabled);

  protected abstract void setDriveVel(double velMPS);

  protected abstract void stopMotors();

  protected abstract void enableCoastMode(boolean enabled);
}
