// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.chute;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.utils.LoggedSubsystem;

public abstract class Chute implements LoggedSubsystem {
  @Log protected double appliedVolts;
  @Log protected double speedRPM;
  @Log protected double currentAmps;

  public static final double SPEED_INTAKING = -0.1;
  public static final double SPEED_OUTAKING = 0.1;
  public static final double STUCK_CURRENT = 15.0;

  /** Runs chute at specific speed */
  public Command runChuteSpeedCmmd(double speed) {
    return this.run(() -> {});
  }

  /** Checks to see if current is spiking for chute motors */
  public boolean hasCoral() {
    return currentAmps > 10.0;
  }

  /** Checks if chute is jammed */
  public boolean isStuck() {
    return currentAmps > 10.0;
  }
}
