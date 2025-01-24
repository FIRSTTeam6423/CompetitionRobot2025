// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import org.frc6423.frc2025.subsystems.swerve.module.Module;
import org.frc6423.frc2025.subsystems.swerve.module.ModuleIO;

public class SwerveSubsystem extends SubsystemBase {

  private final Module[] m_modules;

  public SwerveSubsystem(ModuleIO[] ios) {
    m_modules = new Module[ios.length];

    Arrays.stream(ios).forEach((io) -> m_modules[0] = new Module(io, 0));
  }

  @Override
  public void periodic() {}
}
