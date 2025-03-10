// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.utils.LoggedSubsystem;

public abstract class Elevator implements LoggedSubsystem {
  // * CONSTANTS
  // Mech constants
  public static final double MASS_KG = 5.6 + 1.8;
  public static final double RANGE = 1.218;

  // Poses
  public static final double IDLE_POSE = 0.0;
  public static final double INTAKE_POSE = 0.0;
  public static final double ALGAE_L_POSE = 5.4;
  public static final double ALGAE_H_POSE = 9.7;
  public static final double L1_POSE = 0.0;
  public static final double L2_POSE = (5.5 / 0.87835 / 3.14159 / 2 * 3);
  public static final double L3_POSE = 7.3; // (13.5 / 0.87835 / 3.14159 / 2 * 3);
  public static final double L4_POSE = 12.75;

  // * LOGGED VALUES
  @Log protected double poseRevs;
  @Log protected double targetPoseRevs;
  @Log protected double velRPM;
  @Log protected double appliedVolts;
  @Log protected double currentAmps;
  @Log protected boolean isZeroed;

  @Override
  public void periodic() {
  }

  /**
   * Runs elevator to specified setpoint pose
   * 
   * @param desiredPoseRevs setpoint pose
   * @return Set elevator setpoint command
   */
  public Command runPoseCmmd(double desiredPoseRevs) {
    return this.run(() -> {
      targetPoseRevs = desiredPoseRevs;
      runMotorPose(desiredPoseRevs);
    });
  }

  /**
   * Runs elevator down until current spikes to find zero
   * 
   * @return Elevator zeroing command
   */
  public Command runCurrentZeroingCmmd() {
    return this.run(() ->  {
      runMotorVolts(-0.8);
      targetPoseRevs = 0.0;
    }).until(() -> currentAmps > 20.0)
    .finallyDo((i) -> {
      stopMotors();
      setEncoderPose(0.0);
      isZeroed = true;
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

  // * HARDWARE METHODS
  /**
   * Runs elevator motors with specified amount of voltage
   *
   * @param volts desired voltage
   */
  protected abstract void runMotorVolts(double volts);

  /**
   * Runs elevator motors to a specified position in revs
   *
   * @param poseRevs motor position in revs
   */
  protected abstract void runMotorPose(double poseRevs);

  /**
   * Sets the position of the elevator's rotary encoder
   *
   * @param poseRevs encoder pose in revs
   */
  protected abstract void setEncoderPose(double poseRevs);

  /** Sets all motor input to 0 */
  protected abstract void stopMotors();

  /**
   * Enable or disable coast mode to move elevator easier
   *
   * @param enabled False to set motors in break mode; True to set motors in coast mode
   */
  protected abstract void motorCoasting(boolean enabled);
}
