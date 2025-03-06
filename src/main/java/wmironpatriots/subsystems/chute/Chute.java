// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.chute;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.util.mechanismUtil.LoggedSubsystem;

public abstract class Chute implements LoggedSubsystem {
  @Log protected double chuteAppliedVolts;
  @Log protected double chuteSpeedRPM;
  @Log protected double chuteSupplyCurrent;

  public static final double INTAKE_SPEED = -0.13;

  /** Checks to see if current is spiking for chute motors */
  public boolean hasCoral() {
    return chuteSupplyCurrent > 10.0;
  }

  /** Runs chute at specific speed */
  public Command runChuteSpeedCmmd(double speed) {
    return this.run(() -> {});
  }
}
