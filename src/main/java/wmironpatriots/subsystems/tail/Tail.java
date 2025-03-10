// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project
package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.utils.LoggedSubsystem;

public abstract class Tail implements LoggedSubsystem {
  // * CONSTANTS
  // Mech constants
  public static final double MASS_KG = 0.0;
  public static final double RANGE = 10.1;

  // Poses
  public static final double POSE_STOWED = 0;
  public static final double POSE_MAX = 10;
  public static final double POSE_MIN = 8.2;

  public static final double POSE_L1 = 5.56;
  public static final double POSE_L2 = (20 * 5 / 36); // 5;
  public static final double POSE_L3 = 3.95; // (20 * 5 / 36); // 5.56
  public static final double POSE_L4 = (30 * 5 / 36); // 4;

  public static final double POSE_ALGAE_HIGH = (70 * 5 / 36); // uses formula
  public static final double POSE_ALGAE_LOW = (70 * 5 / 36); // uses formula

  // Roller speeds
  public static final double SPEED_INTAKING = 2;
  public static final double SPEED_OUTAKING = -1;
  public static final double SPEED_SCORING = 2.5;

  // * LOGGED VALUES
  @Log protected double poseRevs;
  @Log protected double targetPoseRevs;
  @Log protected double velRPM;
  @Log protected double appliedVolts;
  @Log protected double currentAmps;
  @Log protected boolean isZeroed;

  /**
   * Runs Tail up until current spikes to find zero
   *
   * @return Tail zeroing command
   */
  public Command runCurrentZeroingCmmd() {
    return this.run(
            () -> {
              setPivotVolts(-1);
              targetPoseRevs = 0.0;
            })
        .until(() -> currentAmps > 20.0)
        .finallyDo(
            (i) -> {
              stopMotors();
              setEncoderPose(0.0);
              isZeroed = true;
            });
  }

  /**
   * Runs tail to specified setpoint pose
   *
   * @param desiredPoseRevs setpoint pose
   * @return Set tail setpoint command
   */
  public Command runPoseCmmd(double desiredPoseRevs) {
    return this.run(
        () -> {
          targetPoseRevs = desiredPoseRevs;
          setPivotPose(desiredPoseRevs);
        });
  }

  /**
   * Runs tail rollers with specified speed
   *
   * @param speed setpoint speed
   * @return Set roller speed command
   */
  public Command runRollerSpeed(double speed) {
    return this.run(() -> setRollerVolts(speed));
  }

  /**
   * Sets all tail motor input to zero
   *
   * @return Stop elevator command
   */
  public Command stopTailCmmd() {
    return this.run(() -> stopMotors());
  }

  /**
   * Sets tail coasting to enabled or disabled
   * 
   * @param enabled true for coasting false for brake
   * @return Set elevator idle mode command
   */
  public Command setCoasting(boolean enabled) {
    return this.run(() -> motorCoasting(enabled));
  }

  // * HARDWARE METHODS
  /** Runs pivot motors with specified volts */
  protected abstract void setPivotVolts(double volts);

  /** Runs pivot motors to specified pose in revs */
  protected abstract void setPivotPose(double poseRevs);

  /** Sets motor rotary encoder pose in revs */
  protected abstract void setEncoderPose(double poseRevs);

  /** Runs roller motors with specified volts */
  protected abstract void setRollerVolts(double volts);

  /** Sets all motor input to 0 */
  protected abstract void stopMotors();

  /** Enable motor coasting for easier movement */
  protected abstract void motorCoasting(boolean enabled);
}
