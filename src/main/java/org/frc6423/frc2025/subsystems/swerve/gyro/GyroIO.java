// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public class GyroIOInputs {
    public Rotation3d orientation = new Rotation3d();
    public double omegaRadsPerSec = 0.0;
  }

  public void updateInputs(GyroIOInputs inputs);
}
