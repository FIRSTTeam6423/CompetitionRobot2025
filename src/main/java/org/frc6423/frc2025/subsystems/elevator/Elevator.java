// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.frc6423.frc2025.Robot;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** ELEVATOR CONSTANTS */
  // mech constants
  public static final double kReduction = 1 / 2;

  public static final double kSpoolRadiusMeters = 0.878350;
  public static final double kRangeMeters = 0.0;

  // Non-scoring poses
  public static final double kChuteIntakingPoseMeters = 0.0;
  public static final double kLowerAlgaeRemovePoseMeters = 0.0;
  public static final double kHigherAlgaeRemovePoseMeters = 0.0;

  // Scoring poses
  public static final double kL1PoseMeters = 0.0;
  public static final double kL2PoseMeters = 0.0;
  public static final double kL3PoseMeters = 0.0;
  public static final double kL4PoseMeters = 0.0;

  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs;

  private double m_setpointMeters;
  private boolean m_zeroed;

  public Elevator() {
    m_io = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOComp();
    m_inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    Logger.recordOutput("Elevator/poseMeters", getPoseMeters());
    Logger.recordOutput("Elevator/zeroed", m_zeroed);
  }

  /** Sets elevator goal setpoint */
  public Command runPoseSetpoint(double poseMeters) {
    return this.run(
        () -> {
          m_setpointMeters = poseMeters;
          m_io.runTargetPose(m_setpointMeters);

          Logger.recordOutput("Elevator/setpointMeters", m_setpointMeters); // Logs setpoint to NT
        });
  }

  /** Sets elevator goal setpoint (meters) */
  public Command runPoseSetpoint(DoubleSupplier poseSupplier) {
    return runPoseSetpoint(poseSupplier.getAsDouble());
  }

  /** Zero elevator encoder at current position */
  public Command runPoseZeroingCommand() {
    return this.run(() -> m_io.runMotorVolts(-2.0, false))
        .until(() -> m_inputs.LMotorSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              m_io.stop();
              m_io.resetPose();
              m_zeroed = true;
            });
  }

  /** Run elevator down until current spikes */
  public Command runCurrentPoseZeroingCommand() {
    return this.run(
        () -> {
          m_io.resetPose();
          m_zeroed = true;
        });
  }

  /** Enable coasting to make manual movement easier */
  public Command elevatorCoasting(boolean enabled) {
    return this.run(() -> m_io.motorCoasting(enabled));
  }

  // GETTERS

  /** Returns elevator goal pose in meters */
  public double getSetpointMeters() {
    return m_setpointMeters;
  }

  /** Returns pose in meters from last encoder zero */
  public double getPoseMeters() {
    return m_inputs.poseMeters;
  }
}
