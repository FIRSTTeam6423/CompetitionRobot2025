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
      int pivotID, int driveID, int cancoderID, double cancoderOffsetRevs, boolean pivotInverted) {}

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

    conf.CurrentLimits.SupplyCurrentLimit = 20.0;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    conf.Feedback.FeedbackRemoteSensorID = cancoderID;
    conf.Feedback.RotorToSensorRatio = PIVOT_REDUCTION;
    conf.Feedback.SensorToMechanismRatio = 1.0;

    conf.Slot0.kP = 30.0;
    conf.Slot0.kD = 0.5;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 2.66;
    conf.Slot0.kS = 0.1;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.ClosedLoopGeneral.ContinuousWrap = true;

    return conf;
  }

  public static TalonFXConfiguration getDriveConf() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    conf.CurrentLimits.StatorCurrentLimit = 50.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimit = 120.0;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    conf.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;

    conf.Slot0.kP = 0.1;
    conf.Slot0.kD = 0.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 0.124;
    conf.Slot0.kS = 0.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.ClosedLoopGeneral.ContinuousWrap = true;

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

  protected abstract void setDriveVolts(double volts, boolean focEnabled);

  protected abstract void setDriveVel(double velMPS, double accelMPSSqrd);

  protected abstract void stopMotors();

  protected abstract void enableCoastMode(boolean enabled);
}
