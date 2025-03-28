// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.LoggedSubsystemComponent;
import monologue.Annotations.Log;

public abstract class Gyro extends LoggedSubsystemComponent {
  @Log protected double headingDegrees;

  public abstract Rotation2d getRotation2d();
}
