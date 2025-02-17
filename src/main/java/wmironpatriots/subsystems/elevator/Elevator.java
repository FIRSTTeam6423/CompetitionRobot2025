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
  public static final double kMassKg = 5.6 + 1.8; // Carriage + 1 stage

  public static final double kReduction = 4;
  public static final double kSpoolRadiusMeters = 0.878350;
  public static final double kRangeMeters = 1.218;

  // Non-scoring poses
  public static final double kLowerAlgaeRemovePoseMeters = 0.0;
  public static final double kHigherAlgaeRemovePoseMeters = 0.0;

  // Scoring poses
  public static final double kL1PoseMeters = 0.0;
  public static final double kL2PoseMeters = 3.16;
  public static final double kL3PoseMeters = 10.81;
  public static final double kL4PoseMeters = 24;

  /** LOGGED VALUES */
  @Log protected boolean LMotorEnabled = false;

  @Log protected boolean RMotorEnabled = false;

  @Log protected double poseMeters;
  @Log protected double velMPS;
  @Log protected boolean isZeroed = false;

  @Log protected double LMotorPoseRads;
  @Log protected double LVelRadsPerSec;
  @Log protected double LMotorAppliedVolts;
  @Log protected double LMotorSupplyCurrentAmps;
  @Log protected double LMotorTorqueCurrentAmps;
  @Log protected double LMotorTempCelsius;

  @Log protected double RMotorPoseRads;
  @Log protected double RVelRadsPerSec;
  @Log protected double RMotorAppliedVolts;
  @Log protected double RMotorSupplyCurrentAmps;
  @Log protected double RMotorTorqueCurrentAmps;
  @Log protected double RMotorTempCelsius;

  /** Variables */
  private final PositionVoltage m_motorPoseOutReq = new PositionVoltage(0.0).withEnableFOC(true);

  private final VoltageOut m_motorVoltOutReq = new VoltageOut(0.0).withEnableFOC(true);

  /** Run target position meters from current zeroed pose */
  public Command runTargetPoseCommand(double poseMeters) {
    return this.run(
        () -> {
          runMotorControl(m_motorPoseOutReq.withPosition(poseMeters).withEnableFOC(true));
        });
  }

  /** Zero elevator encoders at current pose */
  public Command zeroPoseCommand() {
    return this.run(
        () -> {
          setEncoderPose(0.0);
          isZeroed = true;
        });
  }

  /** Runs elevator down until current spikes above threshold */
  public Command runPoseZeroingCommand() {
    return this.run(() -> runMotorControl(m_motorVoltOutReq.withOutput(0.0).withEnableFOC(true)))
        .until(() -> LMotorSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              stopMotors();
              resetPose();
              isZeroed = true;
            });
  }

  /** Enable coast mode to move elevator easier */
  public Command elevatorCoasting(boolean enabled) {
    return this.runOnce(() -> motorCoasting(enabled));
  }

  /** Set elevator pose to 0.0 meters */
  private void resetPose() {
    setEncoderPose(0.0);
  }

  // Hardware methods
  /** Run elevator motor with control request */
  protected abstract void runMotorControl(ControlRequest request);

  /** Set elevator encoder position in meters */
  protected abstract void setEncoderPose(double poseMeters);

  /** Stop motor input */
  protected abstract void stopMotors();

  /** Enable or disable motor coasting */
  protected abstract void motorCoasting(boolean enabled);
}
