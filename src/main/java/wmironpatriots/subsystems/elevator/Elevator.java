// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Elevator extends SubsystemBase implements Logged {
  /** ELEVATOR CONSTANTS */
  // mech constants
  public static final double MASS_KG = 5.6 + 1.8; // Carriage + 1 stage

  public static final double REDUCTION = 3;
  public static final double SPOOL_RADIUS_INCHES = 0.878350;
  public static final double RANGE_ROTS = 1.218;

  // Poses
  public static final double IDLE = 0.0;
  public static final double POSE_INTAKING = 0.0;
  public static final double POSE_ALGAE_H = 0.0; // Remove algae high
  public static final double POSE_ALGAE_L = 0.0; // Remove algae low
  public static final double POSE_MAX_CARRIAGE_STAGE_ONE = 1.39903980829;

  public static final double POSE_L1 = 0.0;
  public static final double POSE_L2 = 3.16;
  public static final double POSE_L3 = 10.81;
  public static final double POSE_L4 = 24;

  // Visualizer constants
  public static final double MAX_ELEVATOR_HEIGHT_INCHES = 57;
  public static final double STAGE_ZERO_HEIGHT_INCHES = 32;
  public static final double STAGE_ONE_HEIGHT_INCHES = 24;

  /** LOGGED VALUES */
  @Log protected boolean parentOk = false;

  @Log protected boolean childOk = false;

  @Log protected double setpointPoseRots;
  @Log protected double poseRevs;
  @Log protected double velRPM;
  @Log protected boolean isZeroed = false;

  @Log protected double parentPoseRevs;
  @Log protected double parentVelRPM;
  @Log protected double parentAppliedVolts;
  @Log protected double parentSupplyCurrentAmps;
  @Log protected double parentTorqueCurrentAmps;
  @Log protected double parentTempCelsius;

  @Log protected double childPoseRevs;
  @Log protected double childVelRPM;
  @Log protected double childAppliedVolts;
  @Log protected double childSupplyCurrentAmps;
  @Log protected double childTorqueCurrentAmps;
  @Log protected double childTempCelsius;

  /** Run target position in revs from current zeroed pose */
  public Command setTargetPoseCmmd(double pose) {
    return this.run(
        () -> {
          setpointPoseRots = pose;
          runMotorPose(pose);
          ;
        });
  }

  /** Zero elevator encoder to current pose */
  public Command zeroPoseCmmd() {
    return this.run(
        () -> {
          setEncoderPose(0);
          isZeroed = true;
        });
  }

  /** Runs elevator down until current spikes above threshold */
  public Command runPoseZeroingCmmd() {
    return this.run(() -> runMotorVolts(-1.0))
        .until(() -> parentSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              stopMotors();
              resetPose();
              isZeroed = true;
            });
  }

  /** Stop all elevator motor input */
  public Command stopMotorInputCmmd() {
    return this.runOnce(() -> runMotorVolts(0.0));
  }

  /** Enable coast mode to move elevator easier */
  public Command enableMotorCoastingCmmd(boolean enabled) {
    return this.runOnce(() -> motorCoastingEnabled(enabled));
  }

  /** Set elevator pose to 0.0 revs */
  private void resetPose() {
    setEncoderPose(0.0);
  }

  /** Returns pose in revs */
  public double getPose() {
    return poseRevs;
  }

  /** Gets velocity in rotations per minute */
  public double getVelocity() {
    return velRPM;
  }

  /** Gets displacement from setpoint pose */
  public double getSetpointDisplacement() {
    return setpointPoseRots - poseRevs;
  }

  /** Checks if elevator has been zeroed */
  public boolean isZeroed() {
    return isZeroed;
  }

  /** Checks if elevator is around a specific range of the setpoint */
  public boolean inSetpointRange() {
    return Math.abs(setpointPoseRots - parentPoseRevs) < 0.05; // TODO tweak range if needed
  }

  /** HARDWARE METHODS */
  /** Run elevator motor with voltage request */
  protected abstract void runMotorVolts(double volts);

  /** Run elevator motor with position request */
  protected abstract void runMotorPose(double poseRevs);

  /** Set elevator encoder position in rotations */
  protected abstract void setEncoderPose(double poseMeters);

  /** Stop motor input */
  protected abstract void stopMotors();

  /** Enable or disable motor coasting */
  protected abstract void motorCoastingEnabled(boolean enabled);
}
