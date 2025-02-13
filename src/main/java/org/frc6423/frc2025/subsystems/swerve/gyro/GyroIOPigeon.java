// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
  private final Pigeon2 m_gyro;

  public GyroIOPigeon(int id) {
    m_gyro = new Pigeon2(0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.orientation = m_gyro.getRotation3d();
    inputs.omegaRadsPerSec = m_gyro.getAccelerationZ().getValueAsDouble();
  }
}
