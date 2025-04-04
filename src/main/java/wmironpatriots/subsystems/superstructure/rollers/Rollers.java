// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Rollers extends SubsystemBase {
  // * CONSTANTS
  public static final double SPEED_INTAKING = 0.0;
  public static final double SPEED_OUTAKING = 0.0;
  public static final double SPEED_SCORING = 0.0;

  /**
   * Runs rollers at specified speed
   *
   * @param speed desired speed
   */
  public Command runRollerSpeed(double speed) {
    return this.run(() -> setMotorSpeed(speed));
  }

  // * HARDWARE METHODS
  protected abstract void setMotorVolts(double volts);

  protected abstract void setMotorSpeed(double speed);

  protected abstract void stopMotors();

  public abstract void close();
}
