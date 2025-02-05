// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig.moduleType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;

/** A class */
public class Constants {

  // * Select which robot to initalize
  private static final RobotType m_robotType = RobotType.SIMULATED;

  /*
   * Robot types:
   * SIMULATED - Inits all subsystems with simulated hardware
   * COMPBOT - Inits all subsystems with real hardware
   * DEVBOT - Inits drive and vision subsystem (for use on practice DT)
   */
  public static enum RobotType {
    SIMULATED,
    COMPBOT,
    DEVBOT
  }

  /*
   * Robot deploy modes:
   * Simulation - Deploy to sim
   * Real - Deploy to comp or dev
   * Replay - AKit replay mode
   */
  public static enum DeployMode {
    SIMULATION,
    REAL,
    REPLAY
  }

  /** Get selected robot type */
  public static RobotType getRobot() {
    if (Robot.isReal() && m_robotType == RobotType.SIMULATED) {
      new Alert("Simulated robot type selected; Defaulting to devbot", AlertType.kError);
      return RobotType.DEVBOT;
    }
    return m_robotType;
  }

  /** Gets current deploy mode {@link DeployMode} */
  public static DeployMode getDeployMode() {
    return switch (m_robotType) {
      case COMPBOT, DEVBOT -> Robot.isReal() ? DeployMode.REAL : DeployMode.REPLAY;
      case SIMULATED -> Robot.isReal() ? DeployMode.REAL : DeployMode.SIMULATION;
    };
  }

  // * SUBSYSTEM CONSTANTS
  public class KDriveConstants {    
    // Swerve Configs
    public static final SwerveConfig kCompBotConfig =
        new SwerveConfig(
            new ModuleConfig[] {
              new ModuleConfig(1, moduleType.TALONFX, 1, 2, 0, Rotation2d.fromRadians(0), true, getPivotConfig(), getDriveConfig(), getCANcoderConfig()),
              new ModuleConfig(2, moduleType.TALONFX, 3, 4, 1, Rotation2d.fromRadians(0), true, getPivotConfig(), getDriveConfig(), getCANcoderConfig()),
              new ModuleConfig(3, moduleType.TALONFX, 5, 6, 2, Rotation2d.fromRadians(0), true, getPivotConfig(), getDriveConfig(), getCANcoderConfig()),
              new ModuleConfig(4, moduleType.TALONFX, 7, 8, 3, Rotation2d.fromRadians(0), true, getPivotConfig(), getDriveConfig(), getCANcoderConfig())
            },
            new Translation2d[] {
              new Translation2d(0.381, 0.381),
              new Translation2d(0.381, -0.381),
              new Translation2d(-0.381, 0.381),
              new Translation2d(-0.381, -0.381)
            },
            23.47,
            23.47,
            2,
            2,
            56, // !
            Units.feetToMeters(16),
            Units.feetToMeters(16),
            100);

    public static final SwerveConfig kDevBotConfig =
        new SwerveConfig(
            new ModuleConfig[] {
              new ModuleConfig(1, moduleType.SPARKMAX, 1, 2, 0, Rotation2d.fromRadians(0), true),
              new ModuleConfig(2, moduleType.SPARKMAX, 3, 4, 1, Rotation2d.fromRadians(0), true),
              new ModuleConfig(3, moduleType.SPARKMAX, 5, 6, 2, Rotation2d.fromRadians(0), true),
              new ModuleConfig(4, moduleType.SPARKMAX, 7, 8, 3, Rotation2d.fromRadians(0), true)
            },
            new Translation2d[] {
              new Translation2d(0.381, 0.381),
              new Translation2d(0.381, -0.381),
              new Translation2d(-0.381, 0.381),
              new Translation2d(-0.381, -0.381)
            },
            23.47,
            23.47,
            2,
            2,
            56, // !
            Units.feetToMeters(16),
            Units.feetToMeters(16),
            100);

    // Swerve Constants
    public static final double kDriveBaseWidth = Units.inchesToMeters(25.0);
    public static final double kDriveBaseLength = Units.inchesToMeters(25.0);
    public static final double kDriveBaseRadius =
        Math.hypot(kDriveBaseWidth / 2.0, kDriveBaseLength / 2.0);

    public static final double kMaxLinearSpeed = Units.feetToMeters(16);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;

    public static final double kSwerveRotationalP = 0.0;
    public static final double kSwerveRotationalI = 0.0;
    public static final double kSwerveRotationalD = 0.0;

    // Global Module Constants (Same for all modules)
    public static enum DriveControlMode {
      OPENLOOP,
      CLOSEDLOOP
    }

    public static final double kPivotReduction = 5 / 1; // ! these values are wrong; check later
    public static final double kDriveReduction = 5 / 1;

    public static final double kVoltageCompensation = 12.0;
    public static final int kPivotCurrentLimitAmps = 40;
    public static final int kDriveCurrentLimitAmps = 40;

    public static final double kWheelRadius = Units.inchesToMeters(4);

    public static final double kPivotP = 1;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;

    public static final double kDriveP = 1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    private static TalonFXConfiguration getPivotConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.Audio.BeepOnBoot = true; // boop

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      config.CurrentLimits.StatorCurrentLimit = kPivotCurrentLimitAmps;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      // Torque
      config.TorqueCurrent.PeakForwardTorqueCurrent = kPivotCurrentLimitAmps;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -kPivotCurrentLimitAmps;
      
      // Feedback config
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      config.Feedback.RotorToSensorRatio = kPivotReduction;
      config.Feedback.SensorToMechanismRatio = 1.0;
      config.Feedback.FeedbackRotorOffset = 0.0;
      config.ClosedLoopGeneral.ContinuousWrap = true; // Takes the shortest path

      // Gains
      config.Slot0.kP = 0.0;
      config.Slot0.kI = 0.0;
      config.Slot0.kD = 0.0;

      config.Slot0.kV = 0.0;
      config.Slot0.kA = 0.0;
      config.Slot0.kS = 0.0;

      config.MotionMagic.MotionMagicCruiseVelocity = 0;
      config.MotionMagic.MotionMagicAcceleration = 0;
      config.MotionMagic.MotionMagicJerk = 0;

      return config;
    }

    private static TalonFXConfiguration getDriveConfig() {}

    private static CANcoderConfiguration getCANcoderConfig() {}
  }
}
