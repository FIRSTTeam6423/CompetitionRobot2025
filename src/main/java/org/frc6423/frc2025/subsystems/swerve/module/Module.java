// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import static org.frc6423.frc2025.Constants.KDriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs;

  private SwerveModuleState m_goalState;

  private final Alert m_pivotConnectionAlert;
  private final Alert m_driveConnectionAlert;

  public Module(ModuleIO io, int index) {
    this.index = index;
    this.io = io;
    this.inputs = new ModuleIOInputsAutoLogged();

    m_pivotConnectionAlert = new Alert(index + " pivot disconnected", AlertType.kWarning);
    m_driveConnectionAlert = new Alert(index + " drive disconnected", AlertType.kWarning);
  }

  /** Update Module */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);

    m_pivotConnectionAlert.set(inputs.pivotEnabled);
    m_driveConnectionAlert.set(inputs.driveEnabled);
  }

  /** Set swerve state goal */
  public void setDesiredSate(SwerveModuleState goalState) {
    m_goalState = goalState;
    Rotation2d currentAngle = getCurrentState().angle;

    m_goalState.optimize(currentAngle);
    // Decreases speed based on how far the module angle is from goal
    m_goalState.cosineScale(currentAngle);

    io.setPivotAngle(m_goalState.angle);
    io.setDriveVelocity(m_goalState.speedMetersPerSecond); // / kDriveReduction);
  }

  /** Enable or Disable module coast */
  public void setCoastMode(boolean enabled) {
    io.setPivotCoastMode(enabled);
    io.setDriveCoastMode(enabled);
  }

  /** Set all Module motor input to 0 */
  public void stopModuleInputs() {
    io.stop();
  }

  /** Get current goal state */
  public SwerveModuleState getDesiredState() {
    return m_goalState;
  }

  /** Get current swerve module state {@link SwerveModuleState} */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(inputs.driveVelRadsPerSec * kWheelRadius, inputs.pivotABSPose);
  }

  /** Get current swerve module pose {@link SwerveModulePosition} */
  public SwerveModulePosition getCurrentPose() {
    return new SwerveModulePosition(inputs.drivePose, inputs.pivotPose);
  }
}
