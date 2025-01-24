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
import org.frc6423.frc2025.subsystems.swerve.module.ModuleIO;
import org.frc6423.frc2025.subsystems.swerve.module.ModuleIOSpark;

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
    public static final Translation2d[] kDevModuleLocs = new Translation2d[] {};

    public static final Translation2d[] kCompModuleLocs = new Translation2d[] {};

    // Devbot module configs
    public static final ModuleConfig[] kDevModuleConfigs =
        new ModuleConfig[] {
          new ModuleConfig(RobotType.DEVBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.DEVBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.DEVBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.DEVBOT, 0, 0, 0, new Rotation2d(), false)
        };

    // Comp module configs
    public static final ModuleConfig[] kCompModuleConfigs =
        new ModuleConfig[] {
          new ModuleConfig(RobotType.COMPBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.COMPBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.COMPBOT, 0, 0, 0, new Rotation2d(), false),
          new ModuleConfig(RobotType.COMPBOT, 0, 0, 0, new Rotation2d(), false)
        };

    // Dev IOs
    public static final ModuleIO[] kDevModuleIOs =
        new ModuleIO[] {
          new ModuleIOSpark(kDevModuleConfigs[0]),
          new ModuleIOSpark(kDevModuleConfigs[0]),
          new ModuleIOSpark(kDevModuleConfigs[0]),
          new ModuleIOSpark(kDevModuleConfigs[0])
        };

    public static final double kPivotReduction = 5 / 1; // ! these values are wrong; check later
    public static final double kDriveReduction = 5 / 1;

    public static final double kWheelRadius = Units.inchesToMeters(4);

    public static final double kPivotP = 0;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;

    public static final double kDriveP = 0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    public record ModuleConfig(
        RobotType type,
        int pivotID,
        int driveID,
        int pivotABSID,
        Rotation2d pivotOffset,
        boolean inverted) {}
  }
}
