// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.elevator;

import static org.frc6423.frc2025.Constants.kTickSpeed;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim m_elevatorSim;

  private final ProfiledPIDController m_elevatorFeedback;
  private final ElevatorFeedforward m_elevatorFeedforward;

  private double m_appliedVolts;

  public ElevatorIOSim() {
    // Create physics sim
    m_elevatorSim =
        new ElevatorSim( // ! These are BS constants
            DCMotor.getKrakenX60Foc(1),
            ElevatorSubsystem.kReduction,
            Units.kilogramsToLbs(ElevatorSubsystem.kMassKg),
            ElevatorSubsystem.kSpoolRadiusMeters,
            0.0,
            ElevatorSubsystem.kRangeMeters,
            true,
            0.0);

    // Gains
    m_elevatorFeedback = new ProfiledPIDController(50.0, 0.0, 0.0, new Constraints(0.0, 0.0));
    m_elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    m_elevatorSim.update(kTickSpeed);

    inputs.LMotorAppliedVolts = m_appliedVolts;
    inputs.LMotorSupplyCurrentAmps = m_elevatorSim.getCurrentDrawAmps();

    inputs.poseMeters = m_elevatorSim.getPositionMeters();
    inputs.velMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
  }

  @Override
  public void runMotorVolts(double voltage, boolean focEnabled) {
    m_appliedVolts = voltage;
    m_elevatorSim.setInputVoltage(voltage);

    Logger.recordOutput("Elevator/simulatedVolts", voltage);
  }

  @Override
  public void runTargetPose(double poseMeters) {
    runMotorVolts(
        m_elevatorFeedback.calculate(m_elevatorSim.getPositionMeters(), poseMeters)
            + m_elevatorFeedforward.calculate(m_elevatorFeedback.getSetpoint().velocity),
        true);

    Logger.recordOutput("Elevator/setpointPose", poseMeters);
  }

  @Override
  public void resetPose(double poseMeters) {}

  @Override
  public void motorCoasting(boolean enabled) {}
}
