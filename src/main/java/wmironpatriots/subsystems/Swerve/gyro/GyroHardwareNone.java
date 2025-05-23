// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.gyro;

import edu.wpi.first.math.geometry.Rotation3d;

public class GyroHardwareNone implements GyroHardware {
  @Override
  public Rotation3d getRotation3d() {
    return Rotation3d.kZero;
  }
}
