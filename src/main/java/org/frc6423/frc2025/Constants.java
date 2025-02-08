// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/** A class */
public class Constants {

  public static final double kTickSpeed = 0.02;

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
  @SuppressWarnings("resource")
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
}
