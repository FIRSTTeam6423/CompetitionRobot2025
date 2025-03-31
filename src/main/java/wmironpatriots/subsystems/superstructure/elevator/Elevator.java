// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public abstract class Elevator extends SubsystemBase {
  // * CONSTANTS
  public static final double POSE_L1_REVS = 0.0;
  public static final double POSE_L2_REVS = 0.0;
  public static final double POSE_L3_REVS = 0.0;
  public static final double POSE_L4_REVS = 0.0;
  public static final double POSE_ALGAE_H_REVS = 0.0;
  public static final double POSE_ALGAE_L_REVS = 0.0;

  private boolean isZeroed;
  private double setpointPose;

  protected final ElevatorIOInputsAutoLogged inputs;

  protected Elevator() {
    isZeroed = false;

    inputs = new ElevatorIOInputsAutoLogged();
  }

  /** Runs elevator down until current spikes above threshold */
  public Command runCurrentZeroingCmd() {
    if (isZeroed) return this.runOnce(() -> {});
    return this.run(() -> setMotorCurrent(-1.0))
        .until(() -> inputs.data.parentCurrentAmps > 20.0)
        .finallyDo(
            (i) -> {
              stopMotors();
              setEncoderPose(0.0);
              System.out.println("ELEVATOR ZEROED");
              isZeroed = true;
            });
  }

  /**
   * Runs elevator to specified pose
   *
   * @param poseRevs desired pose in revs
   */
  public Command runPoseCmd(double poseRevs) {
    return this.run(
        () -> {
          setpointPose = poseRevs;
          setMotorPose(poseRevs);
        });
  }

  /**
   * Checks if elevator pose is around setpoint pose
   *
   * @return true if pose is ±0.5 from setpoint
   */
  public boolean nearSetpoint() {
    return Math.abs(setpointPose - inputs.data.poseRevs) > 0.5;
  }

  /**
   * @return true if elevator is zeroed properly
   */
  public boolean isInitalized() {
    return isZeroed;
  }

  // * HARDWARE METHODS
  protected abstract void setMotorCurrent(double amps);

  protected abstract void setMotorPose(double poseRevs);

  /** Stops all elevator motor output */
  public abstract void stopMotors();

  protected abstract void setEncoderPose(double poseRevs);

  protected abstract void enableCoastMode(boolean enabled);

  // * LOGGING
  @AutoLog
  public static class ElevatorIOInputs {
    public ElevatorData data = new ElevatorData(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  public record ElevatorData(
      double poseRevs,
      double parentCurrentAmps,
      double parentTorqueAmps,
      double parentAppliedVolts,
      double parentTempCelsius,
      double childCurrentAmps,
      double childTorqueAmps,
      double childAppliedVolts,
      double childTempCelsius) {}
}
