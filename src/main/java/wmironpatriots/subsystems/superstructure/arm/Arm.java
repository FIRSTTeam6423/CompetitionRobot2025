// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.arm;

import lib.LoggedSubsystem;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class Arm implements LoggedSubsystem {
  private boolean isZeroed;

  protected final ArmIOInputsAutoLogged inputs;

  /**
   * Arm factory method
   *
   * @return Arm IO based on robot
   */
  public static Arm createArm() {}

  protected Arm() {
    isZeroed = false;

    inputs = new ArmIOInputsAutoLogged();
  }

  /** Runs arm inwards until current spikes above threshold */
  public Command runCurrentZeroingCmd() {
    if (isZeroed) return this.runOnce(() -> {});
    return this.run(() -> setMotorCurrent(-1.0))
        .until(() -> inputs.data.pivotCurrentAmps > 20.0)
        .finallyDo((i) -> {
          stopMotors();
          setEncoderPose(0.0);
          System.out.println("ARM ZEROED");
          isZeroed = true;
        });
  }

  // * HARDWARE METHODS
  protected abstract void setMotorCurrent(double amps);

  protected abstract void setMotorPose(double poseRevs);

  /** Stops arm pivot motor output */
  public abstract void stopMotors();

  protected abstract void setEncoderPose(double poseRevs);

  protected abstract void enableCoastMode(boolean enabled);

  // * LOGGING
  @AutoLog
  public static class ArmIOInputs {}

  public record ArmData() {}
}
