// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;

public class DevBotSwerveConfigs extends SwerveConfig {
  // Kinematics Constants
  // ! fix
  @Override
  public double getMaxLinearSpeedMetersPerSec() {
    // https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/3
    return Units.rotationsToRadians(6000 / 60) / getDriveReduction() * getWheelRadiusInches();
  }

  @Override
  public double getMaxLinearAccelMetersPerSecSqrd() {
    return Units.feetToMeters(8.0);
  }

  // Robot characteristics
  @Override
  public double getRobotMassKg() {
    return 0.0;
  }

  @Override
  public double getRobotWidthMeters() {
    return 0.0;
  }

  @Override
  public double getRobotLengthMeters() {
    return 0.0;
  }

  @Override
  public double getTrackWidthYMeters() {
    return 0.0;
  }

  @Override
  public double getTrackWidthXMeters() {
    return 0.0;
  }

  @Override
  public double getBumperWidthInches() {
    return 0.0;
  }

  @Override
  public double getBumperLengthMeters() {
    return 0.0;
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
    return 0.0;
  }

  @Override
  public double getDriveReduction() {
    return 0.0;
  }

  @Override
  public double getWheelRadiusInches() {
    return 0.0;
  }

  @Override
  public ModuleConfig[] getModuleConfigs() {
    return new ModuleConfig[] {
      new ModuleConfig(
          1,
          1,
          2,
          0,
          Rotation2d.fromRadians(0),
          true,
          getPivotConfigSparkMax(),
          getDriveConfigSparkMax()),
      new ModuleConfig(
          2,
          3,
          4,
          1,
          Rotation2d.fromRadians(0),
          true,
          getPivotConfigSparkMax(),
          getDriveConfigSparkMax()),
      new ModuleConfig(
          3,
          5,
          6,
          2,
          Rotation2d.fromRadians(0),
          true,
          getPivotConfigSparkMax(),
          getDriveConfigSparkMax()),
      new ModuleConfig(
          4,
          7,
          8,
          3,
          Rotation2d.fromRadians(0),
          true,
          getPivotConfigSparkMax(),
          getDriveConfigSparkMax()),
    };
  }

  // Gains
  @Override
  public PIDController getRotationalFeedback() {
    return new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public PIDController getTranslationFeedback() {
    return new PIDController(0.0, 0.0, 0.0);
  }

  // CTRe Configs
  @Override
  public CANcoderConfiguration getCANcoderConfig() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCANcoderConfig'");
  }

  @Override
  public TalonFXConfiguration getPivotConfigTalonFX() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPivotConfigTalonFX'");
  }

  @Override
  public TalonFXConfiguration getDriveConfigTalonFX() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveConfigTalonFX'");
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
    return GyroType.NAVX;
  }

  @Override
  public int getGyroID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroID'");
  }
}
