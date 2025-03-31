// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLog;

public abstract class Arm extends SubsystemBase {
  private boolean isZeroed;

  protected final ArmIOInputsAutoLogged inputs;

  protected Arm() {
    isZeroed = false;

    inputs = new ArmIOInputsAutoLogged();
  }

  /** Runs arm inwards until current spikes above threshold */
  public Command runCurrentZeroingCmd() {
    if (isZeroed) return this.runOnce(() -> {});
    return this.run(() -> setMotorCurrent(-1.0))
        .until(() -> inputs.data.currentAmps > 20.0)
        .finallyDo(
            (i) -> {
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
  public static class ArmIOInputs {
    public ArmData data = new ArmData(0, 0, 0, 0, 0, false);
  }

  public record ArmData(double poseRevs, double currentAmps, double torqueAmps, double appliedVolts, double tempCelsius, boolean beamTriggered) {}
}
