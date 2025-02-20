// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

/** Elevator subsytem for raising and lower tail subsystem for scoring */
public abstract class Elevator extends SubsystemBase {
  /** ELEVATOR CONSTANTS */
  // mech constants
  public static final double MASS_KG = 5.6 + 1.8; // Carriage + 1 stage

  public static final double REDUCTION = 3;
  public static final double SPOOL_RADIUS_M = 0.878350;
  public static final double RANGE_ROTS = 1.218;

  // Non-scoring poses
  public static final double POSE_ALGAE_H = 0.0; // Remove algae high
  public static final double POSE_ALGAE_L = 0.0; // Remove algae low

  // Scoring poses
  // TODO CONVERT POSES TO RADIANS
  public static final double POSE_L2 = 3.16;
  public static final double POSE_L3 = 10.81;
  public static final double POSE_L4 = 24;

  /** LOGGED VALUES */
  @Log protected boolean parentOk = false;

  @Log protected boolean childOk = false;

  @Log protected double setpointPoseRots;
  @Log protected double poseRots;
  @Log protected double velRPM;
  @Log protected boolean isZeroed = false;

  @Log protected double parentPoseRots;
  @Log protected double parentVelRPM;
  @Log protected double parentAppliedVolts;
  @Log protected double parentSupplyCurrentAmps;
  @Log protected double parentTorqueCurrentAmps;
  @Log protected double parentTempCelsius;

  @Log protected double childPoseRots;
  @Log protected double childVelRPM;
  @Log protected double childAppliedVolts;
  @Log protected double childSupplyCurrentAmps;
  @Log protected double childTorqueCurrentAmps;
  @Log protected double childTempCelsius;

  /** VARIABLES */
  private final PositionVoltage reqMotorPose =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final VoltageOut reqMotorVolt =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  /** Run target position in rotations from current zeroed pose */
  public Command runTargetPoseCommand(double pose) {
    return this.run(
        () -> {
          setpointPoseRots = pose;
          runMotorControl(reqMotorPose.withPosition(pose).withEnableFOC(true));
        });
  }

  /** Zero elevator encoders at current pose */
  public Command zeroPoseCommand() {
    return this.run(
        () -> {
          setEncoderPose(0);
          isZeroed = true;
        });
  }

  /** Runs elevator down until current spikes above threshold */
  public Command runPoseZeroingCommand() {
    return this.run(() -> runMotorControl(reqMotorVolt.withOutput(-1.0)))
        .until(() -> parentSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              System.out.println("Zeroed");
              stopMotors();
              resetPose();
              isZeroed = true;
            });
  }

  /** Enable coast mode to move elevator easier */
  public Command elevatorCoasting(boolean enabled) {
    return this.runOnce(() -> motorCoasting(enabled));
  }

  /** Set elevator pose to 0.0 rotations */
  private void resetPose() {
    setEncoderPose(0.0);
  }

  /** Checks if elevator is around a specific range of the setpoint */
  public boolean inSetpointRange() {
    return Math.abs(setpointPoseRots - parentPoseRots) < 0.05; // TODO tweak range if needed
  }

  /** HARDWARE METHODS */
  /** Run elevator motor with control request */
  protected abstract void runMotorControl(ControlRequest request);

  /** Set elevator encoder position in rotations */
  protected abstract void setEncoderPose(double poseMeters);

  /** Stop motor input */
  protected abstract void stopMotors();

  /** Enable or disable motor coasting */
  protected abstract void motorCoasting(boolean enabled);
}
