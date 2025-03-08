// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import monologue.Annotations.Log;
import wmironpatriots.util.mechanismUtil.LoggedSubsystem;

public abstract class Tail implements LoggedSubsystem {
  /** CONSTANTS */
  // mech constants
  // TODO check values in CAD
  public static final double GEAR_REDUCTION = 50;

  public static final double MASS_KG = 0.0;
  public static final double LENGTH_INCHES = 10.0;
  public static final double JKG_METERS_SQRD = 3.0;
  public static final double CURRENT_LIMIT = 40.0;

  // Poses
  public static final double POSE_IN_ANGLE = 0;
  public static final double POSE_OUT_ANGLE = 10;
  public static final double POSE_MOVE_ANGLE = 8.2;

  public static final double POSE_L1 = 5.56;
  public static final double POSE_L2 = (20 * 5 / 36); // 5;
  public static final double POSE_L3 = 3.95; // (20 * 5 / 36); // 5.56
  public static final double POSE_L4 = (30 * 5 / 36); // 4;

  public static final double POSE_ALGAE_HIGH = (70 * 5 / 36); // uses formula
  public static final double POSE_ALGAE_LOW = (70 * 5 / 36); // uses formula

  // Roller speeds
  public static final double INTAKING_SPEEDS = 2;
  public static final double OUTTAKING_SPEEDS = -1;
  public static final double OUTPUTTING_SPEEDS = 2.5;

  public static final double PIVOT_P = 0.4;
  public static final double PIVOT_I = 0.1;
  public static final double PIVOT_D = 0.0;

  public static final double POSE_MIN_REVS = 0;

  /** LOGGED VALUES */
  @Log protected boolean isZeroed = false;

  @Log protected boolean pivotMotorOk = false;
  @Log protected boolean rollerMotorOk = false;

  @Log public boolean beamTripped = true;

  @Log protected double pivotSetpointRevs;
  @Log protected double pivotPoseRevs;
  @Log protected double pivotVelRPM;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotSupplyCurrentAmps;

  @Log protected double rollerVelRPM;
  @Log protected double rollerAppliedVolts;
  @Log protected double rollerSupplyCurrentAmps;

  Timer timer = new Timer();

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

  public Command setRollerTimecmmd(double speed, double time) {
    return this.run(
        () -> {
          runRollerSpeed(speed);
          timer.reset();
          timer.start();
          if (timer.hasElapsed(1)) {
            runRollerSpeed(0);
          }
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
    return this.run(() -> runPivotVolts(-2))
        .until(() -> pivotSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              runPivotVolts(0.0);
              setEncoderPose(0);
              System.out.println("Tail zeroed");
              isZeroed = true;
            });
  }

  public Command setRollerPositionCommand(double revs) {
    return this.run(
        () -> {
          setRollerPosition(revs);
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
    return Math.abs(pivotSetpointRevs - pivotPoseRevs) < .6;
  }

  public boolean pastCollisionPose() {
    return pivotPoseRevs > 3.19;
  }

  /** Checks if both tail beambreaks are triggered */
  public boolean hasCoral() {
    return beamTripped;
  }

  /** HARDWARE METHODS */
  /** Run pivot motor with voltage request */
  protected abstract void runPivotVolts(double volts);

  /** Run pivot motor with pose request */
  protected abstract void runPivotSetpoint(double setpointRevs);

  /** Run roller motor with speed request */
  protected abstract void runRollerSpeed(double speed);

  protected abstract void setRollerPosition(double revs);

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
