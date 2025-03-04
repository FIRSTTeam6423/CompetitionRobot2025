// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Annotations.Log;
import wmironpatriots.util.mechanismUtil.IronComponent;

public abstract class Gyro extends IronComponent {
  /** CONSTANTS */
  public static final int GYRO_ID = 0;

  /** LOGGED VALUES */
  @Log public double yawRads;

  @Log public double pitchRads;
  @Log public double rollRads;
  @Log public double yawVelRadsPerSec;
  @Log public double pitchVelRadsPerSec;
  @Log public double rollVelRadsPerSec;

  public Gyro() {}

  /**
   * @return {@link Rotation2d} of yaw
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(yawRads);
  }
}
