// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import org.frc6423.frc2025.subsystems.elevator.Elevator;
import org.frc6423.frc2025.subsystems.wrist.Wrist;

public class Superstructure {
  public static enum StructState {
    IDLE,
    INTAKE_CHUTE,
    INTAKE_GROUND,
    L1_SETUP,
    L2_SETUP,
    L3_SETUP,
    L4_SETUP,
    REEF_SCORE,
    CORAL_OUTTAKE,
    ALGAE_HIGHER,
    ALGAE_LOWER,
    PROCESSOR_SETUP,
    PROCESSOR_SCORE,
    ALGAE_OUTTAKE
  }

  private final Elevator m_elevator;
  private final Wrist m_wrist;

  private StructState m_state;
  private final HashMap<StructState, Trigger> m_stateTriggers;

  public Superstructure(Elevator elevator, Wrist wrist) {
    m_elevator = new Elevator();
    m_wrist = new Wrist();

    m_state = StructState.IDLE;
    m_stateTriggers = new HashMap<StructState, Trigger>();

    for (StructState state : StructState.values()) {
      m_stateTriggers.put(state, new Trigger(() -> m_state == state));
    }

    m_stateTriggers.get(StructState.INTAKE_CHUTE).whileTrue(m_elevator.runPoseSetpoint(0.0));
  }
}
