// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class Intake extends SubsystemBase implements Logged {
  /** INTAKE CONSTANTS */
  // mech constants
  public static final double REDUCTION = 25;
  public static final double OFFSET_RADS = 0.0; // TODO MEASURE ABS OFFSET

  // Poses
  public static final double POSE_STOWED = 0.0;
  public static final double POSE_INTAKING = Math.PI * 2;

  /** LOGGED VALUES */
  @Log protected boolean pivotOk = false;
  @Log protected boolean rollerOk = false;

  @Log protected double pivotSetpointRads;
  @Log protected double pivotPoseABSRads;
  @Log protected double pivotPoseRads;
  @Log protected double pivotVelRadsPerSec;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotSupplyCurrent;
  @Log protected double pivotTorqueCurrent;
  @Log protected double pivotTempCelsius;

  /** VARIABLES */

  public Command setTargetPoseCommand(double pose) {
    return this.run(
        () -> {
          pivotSetpointRads = pose;
          runPivotPose(pose);
        });
  }

  /** Enable coast mode to move the intake easier */
  public Command intakeCoasting(boolean enabled) {
    return this.runOnce(() -> pivotCoastingEnabled(enabled));
  }

  /** HARDWARE METHODS */
  /** Run pivot motor with position request */
  protected abstract void runPivotPose(double poseRads);

  /** Enable or disable motor coasting` */
  protected abstract void pivotCoastingEnabled(boolean booleanEnable);
}
