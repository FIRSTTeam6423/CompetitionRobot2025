// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.frc6423.frc2025.Constants.KDriveConstants.ModuleConfig;

public class ModuleIOSim implements ModuleIO {

  public ModuleIOSim(ModuleConfig config) {}

  @Override
  public void updateInputs(ModuleIOInputs inputs) {}

  @Override
  public void periodic() {}

  @Override
  public void setPivotVolts(double volts) {}

  @Override
  public void setDriveVolts(double volts) {}

  @Override
  public void setPivotAngle(Rotation2d angle) {}

  @Override
  public void setDriveVelocity(double velMetersPerSec) {}

  @Override
  public void setPivotCoastMode(boolean enabled) {}

  @Override
  public void setDriveCoastMode(boolean enabled) {}

  @Override
  public void stop() {}
}
