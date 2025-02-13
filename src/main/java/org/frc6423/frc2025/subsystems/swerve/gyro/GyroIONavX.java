// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.orientation = m_gyro.getRotation3d();
    inputs.omegaRadsPerSec = Units.degreesToRadians(m_gyro.getRate());
  }
}
