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

  public static final double kReduction = 3;
  public static final double kSpoolRadiusMeters = 0.878350;
  public static final double kRangeMeters = 1.218;

  // Non-scoring poses
  public static final double kLowerAlgaeRemovePose = 0.0;
  public static final double kHigherAlgaeRemovePose = 0.0;

  // Scoring poses
  public static final double kIdlePose = 0.0;
  public static final double kL2Pose = 3.16;
  public static final double kL3Pose = 10.81;
  public static final double kL4Pose = 24;

  /** LOGGED VALUES */
  @Log protected boolean LMotorOk = false;
  @Log protected boolean RMotorOk = false;

  @Log protected double setpointPose;
  @Log protected double pose;
  @Log protected double velRPM;
  @Log protected boolean isZeroed = false;

  @Log protected double LMotorPose;
  @Log protected double LMotorVelRPM;
  @Log protected double LMotorAppliedVolts;
  @Log protected double LMotorSupplyCurrentAmps;
  @Log protected double LMotorTorqueCurrentAmps;
  @Log protected double LMotorTempCelsius;

  @Log protected double RMotorPose;
  @Log protected double RMotorVelRPM;
  @Log protected double RMotorAppliedVolts;
  @Log protected double RMotorSupplyCurrentAmps;
  @Log protected double RMotorTorqueCurrentAmps;
  @Log protected double RMotorTempCelsius;

  /** VARIABLES */
  private final PositionVoltage m_motorPoseOutReq =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final VoltageOut m_motorVoltOutReq =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  /** Run target position in rotations from current zeroed pose */
  public Command runTargetPoseCommand(double pose) {
    return this.run(
        () -> {
          setpointPose = pose;
          runMotorControl(m_motorPoseOutReq.withPosition(pose).withEnableFOC(true));
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
    return this.run(() -> runMotorControl(m_motorVoltOutReq.withOutput(-1.0)))
        .until(() -> LMotorSupplyCurrentAmps > 20.0)
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
    return Math.abs(setpointPose - LMotorPose) < 0.05; // TODO tweak range if needed
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
