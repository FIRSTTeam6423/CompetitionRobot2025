// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.chute;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Chute extends SubsystemBase {
  public static final double INTAKE_SPEED = -0.13;
  public static final double OUTAKE_SPEED = 0.13;

  public Command runChuteSpeedCmmd(double speed) {
    return this.run(() -> {});
  }
}
