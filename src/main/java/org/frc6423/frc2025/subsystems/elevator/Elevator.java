// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc6423.frc2025.Robot;

public class Elevator extends SubsystemBase {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs;

  public Elevator() {
    m_io = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOComp();
    m_inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
  }

  /** Zero elevator encoder at current position */
  public Command runPoseZeroingCommand() {
    return this.run(() -> m_io.runMotorVolts(-2.0, false))
        .until(() -> m_inputs.LMotorSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              m_io.stop();
              m_io.resetPose();
            });
  }

  /** Run elevator down until current spikes */
  public Command runCurrentPoseZeroingCommand() {
    return this.run(() -> m_io.resetPose());
  }
}
