// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.tail.roller;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.utils.LoggedSubsystem;

public abstract class Roller implements LoggedSubsystem {
  // * CONSTANTS
  public static final double SPEED_INTAKING = 2;
  public static final double SPEED_OUTAKING = -1;
  public static final double SPEED_SCORING = 2.5;

  // * LOGGED VALUES
  @Log public double poseRevs;
  @Log public double velRPM;

  /**
   * Runs tail rollers until coral is in scoring pose
   *
   * @return coral indexing command
   */
  public Command indexCoralCmmd() {
    return runRollerDistanceCmmd(2);
  }

  /**
   * Runs rollers at specified speed
   *
   * @param speed in revs
   * @return run roller speed command
   */
  public Command runRollerSpeedCmmd(double speed) {
    return this.run(() -> setRollerSpeed(speed));
  }

  /**
   * Runs rollers for a specified distance
   *
   * @param distanceRevs distance in revs
   * @return run roller distance command
   */
  public Command runRollerDistanceCmmd(double distanceRevs) {
    double prevPose = distanceRevs;
    return this.run(() -> setRollerVolts(1)).until(() -> this.poseRevs - prevPose == distanceRevs);
  }

  // * HARDWARE METHODS
  protected abstract void setRollerVolts(double volts);

  protected abstract void setRollerSpeed(double speeds);

  protected abstract void stopRollers();
}
