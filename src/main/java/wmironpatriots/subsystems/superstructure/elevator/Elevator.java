// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Elevator implements Logged, Subsystem {
  // * CONSTANTS
  // Mech constants
  public static final double MASS_KG = 5.6 + 1.8;
  public static final double RANGE = 1.218;

  // Poses
  public static final double POSE_STOWED = 0.0;
  public static final double POSE_COLLISION =
      4.18; // Position where top of tail will collide with top of first stage when stowed
  public static final double POSE_INTAKE = 0.0;
  public static final double POSE_ALGAE_L = 5.4;
  public static final double POSE_ALGAE_H = 9.7;
  public static final double POSE_L1 = 0.0;
  public static final double POSE_L2 = (5.5 / 0.87835 / 3.14159 / 2 * 3);
  public static final double POSE_L3 = 7.3; // (13.5 / 0.87835 / 3.14159 / 2 * 3);
  public static final double POSE_L4 = 12.75;

  // * LOGGED VALUES
  @Log public double poseRevs;
  @Log public double targetPoseRevs;
  @Log public double velRPM;
  @Log public double appliedVolts;
  @Log public double currentAmps;
  @Log public boolean isZeroed = false;

  /**
   * Runs elevator down until current spikes to find zero
   *
   * @return Elevator zeroing command
   */
  public Command runCurrentZeroingCmmd() {
    return this.run(() -> setMotorVolts(-1.0))
        .until(() -> currentAmps > 20.0)
        .finallyDo(
            (i) -> {
              stopMotors();
              setEncoderPose(0.0);
              System.out.println("Elevator zeroed");
              isZeroed = true;
            });
  }

  /**
   * Runs elevator to specified setpoint pose
   *
   * @param desiredPoseRevs setpoint pose
   * @return Set elevator setpoint command
   */
  public Command runPoseCmmd(double desiredPoseRevs) {
    return this.run(
        () -> {
          targetPoseRevs = desiredPoseRevs;
          setMotorPose(desiredPoseRevs);
        });
  }

  /**
   * Sets all elevator motor input to zero
   *
   * @return Stop elevator command
   */
  public Command stopElevatorCmmd() {
    return this.run(() -> stopMotors());
  }

  /**
   * Sets elevator coasting to enabled or disabled
   *
   * @param enabled true for coasting false for brake
   * @return Set elevator idle mode command
   */
  public Command setCoasting(boolean enabled) {
    return this.run(() -> motorCoasting(enabled));
  }

  /**
   * Checks if elevator pose is in a 0.5 rev range from setpoint
   *
   * @return true if in range false if not
   */
  public boolean nearSetpoint() {
    return Math.abs(targetPoseRevs - poseRevs) > 0.5;
  }

  // * HARDWARE METHODS
  /** Runs elevator motors with specified volts */
  protected abstract void setMotorVolts(double volts);

  /** Runs elevator motors to specified pose in revs */
  protected abstract void setMotorPose(double poseRevs);

  /** Sets motor rotary encoder pose in revs */
  protected abstract void setEncoderPose(double poseRevs);

  /** Sets all motor input to 0 */
  protected abstract void stopMotors();

  /** Enable motor coasting for easier movement */
  protected abstract void motorCoasting(boolean enabled);
}
