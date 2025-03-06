// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.util.mechanismUtil.IronSubsystem;

public abstract class Tail implements IronSubsystem {
  /** CONSTANTS */
  // mech constants
  // TODO check values in CAD
  public static final double GEAR_REDUCTION = 50;

  public static final double MASS_KG = 0.0;
  public static final double LENGTH_INCHES = 10.0;
  public static final double JKG_METERS_SQRD = 3.0;
  public static final double CURRENT_LIMIT = 40.0;

  // Poses
  public static final double POSE_MIN_REVS = -10.4;
  public static final double POSE_MAX_REVS = 0.1;
  public static final double POSE_MOVE_REVS = -5;

  public static final double POSE_L1 = -4;
  public static final double POSE_L2 = -4;
  public static final double POSE_L3 = -4;
  public static final double POSE_L4 = -4;

  // Roller speeds
  public static final double INTAKING_SPEEDS = 1;
  public static final double OUTTAKING_SPEEDS = -1;
  public static final double OUTPUTTING_SPEEDS = 2;

  public static final double PIVOT_P = 1.5;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0;

  /** LOGGED VALUES */
  @Log protected boolean isZeroed = false;

  @Log protected boolean pivotMotorOk = false;
  @Log protected boolean rollerMotorOk = false;

  @Log protected boolean beamTriggered = false;

  @Log protected double pivotSetpointRevs;
  @Log protected double pivotPoseRevs;
  @Log protected double pivotVelRPM;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotSupplyCurrentAmps;

  @Log protected double rollerVelRPM;
  @Log protected double rollerAppliedVolts;
  @Log protected double rollerSupplyCurrentAmps;

  /** Runs target position in radians from current zeroed pose */
  public Command setTargetPoseCmmd(double poseRes) {
    return this.run(
        () -> {
          pivotSetpointRevs = poseRes;
          runPivotSetpoint(poseRes);
        });
  }

  

  /** Runs rollers at specific speed */
  public Command setRollerSpeedCmmd(double speed) {
    return this.run(
        () -> {
          runRollerSpeed(speed);
        });
  }

  /** Zeroes tail pivot at current pose */
  public Command zeroPoseCmmd() {
    return this.run(
        () -> {
          isZeroed = true;
          // setEncoderPose(POSE_MAX_REVS);
        });
  }

  /** Runs pivot backwards until current spikes above threshold */
  public Command runPoseZeroingCmmd() {
    return this.run(() -> runPivotVolts(-1))
        .until(() -> pivotSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              runPivotVolts(0.0);
              setEncoderPose(POSE_MAX_REVS);
              System.out.println("Tail zeroed");
              isZeroed = true;
            });
  }

  /** Stop all elevator motor input */
  public Command stopMotorInputCmmd() {
    return this.runOnce(() -> runPivotVolts(0.0));
  }

  /** Enable coast mode to move elevator easier */
  public Command enableMotorCoastingCmmd(boolean enabled) {
    return this.runOnce(() -> pivotCoastingEnabled(enabled));
  }

  /** Returns pivot pose in radians */
  public double getPose() {
    return pivotPoseRevs;
  }

  /** Checks if elevator has been zeroed */
  public boolean isZeroed() {
    return isZeroed;
  }

  /** Checks if pivot pose is +- PI/8 rads from specified pose */
  public boolean inSetpointRange() {
    return Math.abs(pivotSetpointRevs - pivotPoseRevs) < 0.2;
  }

  /** Checks if both tail beambreaks are triggered */
  public boolean hasCoral() {
    return beamTriggered;
  }

  /** HARDWARE METHODS */
  /** Run pivot motor with voltage request */
  protected abstract void runPivotVolts(double volts);

  /** Run pivot motor with pose request */
  protected abstract void runPivotSetpoint(double setpointRevs);

  /** Run roller motor with speed request */
  protected abstract void runRollerSpeed(double speed);

  /** Reset encoder to specific pose in rads */
  protected abstract void setEncoderPose(double poseRevs);

  /** Zeros pivot at current pose */
  private void resetEncoderPose() {
    setEncoderPose(0.0);
  }

  /** Stop pivot motor */
  protected abstract void stopPivot();

  /** Stop roller motor */
  protected abstract void stopRollers();

  /** Set tail to coast mode for easier movement */
  protected abstract void pivotCoastingEnabled(boolean enabled);
}
