// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;

public class CompBotSwerveConfigs extends SwerveConfig {

  // Kinematics Constants
  // ! fix
  @Override
  public double getMaxLinearSpeedMetersPerSec() {
    // https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/3
    return Units.rotationsToRadians(6000 / 60) / getDriveReduction() * getWheelRadiusInches();
  }

  @Override
  public double getMaxLinearAccelMetersPerSecSqrd() {
    return Units.feetToMeters(16.0);
  }

  // Robot characteristics
  @Override
  public double getRobotMassKg() {
    return Units.lbsToKilograms(121);
  }

  @Override
  public double getRobotWidthMeters() {
    return Units.inchesToMeters(28.500000);
  }

  @Override
  public double getRobotLengthMeters() {
    return Units.inchesToMeters(28.500000);
  }

  @Override
  public double getTrackWidthYMeters() {
    return Units.inchesToMeters(18.404182);
  }

  @Override
  public double getTrackWidthXMeters() {
    return Units.inchesToMeters(18.404182);
  }

  @Override
  public double getBumperWidthInches() {
    return Units.inchesToMeters(34.500000);
  }

  @Override
  public double getBumperLengthMeters() {
    return Units.inchesToMeters(34.500000);
  }

  @Override
  public Translation2d[] getModuleLocs() {
    return new Translation2d[] {
      new Translation2d(0.381, 0.381),
      new Translation2d(0.381, -0.381),
      new Translation2d(-0.381, 0.381),
      new Translation2d(-0.381, -0.381)
    };
  }

  // Module characteristics
  @Override
  public double getPivotReduction() {
    return 150 / 7;
  }

  @Override
  public double getDriveReduction() {
    return 6.21;
  }

  @Override
  public double getWheelRadiusInches() {
    return 2;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    var configs =
        new ModuleConfig[] {
          new ModuleConfig(
              1,
              1,
              2,
              0,
              Rotation2d.fromRadians(0),
              true,
              getPivotConfigTalonFX(),
              getDriveConfigTalonFX(),
              getCANcoderConfig()),
          new ModuleConfig(
              2,
              3,
              4,
              1,
              Rotation2d.fromRadians(0),
              true,
              getPivotConfigTalonFX(),
              getDriveConfigTalonFX(),
              getCANcoderConfig()),
          new ModuleConfig(
              3,
              5,
              6,
              2,
              Rotation2d.fromRadians(0),
              true,
              getPivotConfigTalonFX(),
              getDriveConfigTalonFX(),
              getCANcoderConfig()),
          new ModuleConfig(
              4,
              7,
              8,
              3,
              Rotation2d.fromRadians(0),
              true,
              getPivotConfigTalonFX(),
              getDriveConfigTalonFX(),
              getCANcoderConfig())
        };

    Arrays.stream(configs)
        .forEach(
            (c) -> c.kWheelRadiusMeters = Units.inchesToMeters(getWheelRadiusInches())); // Shut up

    return configs;
  }

  // Gains
  @Override
  public PIDController getRotationalFeedback() {
    return new PIDController(1.0, 0.0, 0.0);
  }

  @Override
  public PIDController getTranslationFeedback() {
    return new PIDController(1.0, 0.0, 0.0);
  }

  // CTRe Configs
  @Override
  public CANcoderConfiguration getCANcoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    return config;
  }

  @Override
  public TalonFXConfiguration getPivotConfigTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Audio.BeepOnBoot = true; // boop

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = getPivotCurrentLimitAmps();
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Torque
    config.TorqueCurrent.PeakForwardTorqueCurrent = getPivotCurrentLimitAmps();
    config.TorqueCurrent.PeakReverseTorqueCurrent = -getPivotCurrentLimitAmps();

    // Feedback config
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = getPivotReduction();
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.ClosedLoopGeneral.ContinuousWrap = true; // Takes the shortest path

    // Gains
    config.Slot0.kP = 500.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.68275;

    config.Slot0.kV = 0.43;
    config.Slot0.kA = 0.0;
    config.Slot0.kS = 0.27;

    config.MotionMagic.MotionMagicCruiseVelocity = 1;
    config.MotionMagic.MotionMagicAcceleration = 1;
    config.MotionMagic.MotionMagicJerk = 1;

    return config;
  }

  @Override
  public TalonFXConfiguration getDriveConfigTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 1.0;
    return config;
  }

  // REV Configs
  @Override
  public AlternateEncoderConfig getPivotABSEncoderConfig() {
    return new AlternateEncoderConfig();
  }

  @Override
  public SparkMaxConfig getPivotConfigSparkMax() {
    return new SparkMaxConfig();
  }

  @Override
  public SparkMaxConfig getDriveConfigSparkMax() {
    return new SparkMaxConfig();
  }

  // Gyro
  @Override
  public GyroType getGyroType() {
    return GyroType.PIGEON;
  }

  @Override
  public int getGyroID() {
    return 0;
  }
}
