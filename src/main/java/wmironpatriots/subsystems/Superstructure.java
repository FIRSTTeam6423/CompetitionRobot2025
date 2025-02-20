// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import wmironpatriots.subsystems.elevator.Elevator;

/** Cursed superstructure ahh frfr */
public class Superstructure {
  public static enum Requests {
    INTAKE_GROUND,
    INTAKE_CHUTE,
    REEF_SCORE,
    PROCESSOR_SCORE,
    OUTTAKE_CORAL,
    OUTTAKE_ALGAE,
    REMOVE_ALGAE_H,
    REMOVE_ALGAE_L,
  }

  public static enum State {
    IDLE,
    INTAKING_GROUND,
    INTAKING_CHUTE,
    L1_SCORING,
    L2_SCORING,
    L3_SCORING,
    L4_SCORING,
    PROCESSOR_SCORE
  }

  private final Map<State, Trigger> stateMap = new HashMap<State, Trigger>();
  private State currentState = State.IDLE;

  private boolean hasCoral = false;
  private boolean hasAlgae = false;

  public Superstructure(Elevator elevator, Map<Requests, Trigger> requestMap) {
    for (var state : State.values()) {
      stateMap.put(state, new Trigger(() -> this.currentState == state));
    }

    /** REQUEST TRIGGERS */
    requestMap
        .get(Requests.INTAKE_CHUTE)
        .and(() -> !hasCoral)
        .onTrue(setCurrentStateCommand(State.INTAKING_CHUTE));

    requestMap
        .get(Requests.INTAKE_GROUND)
        .and(() -> !hasAlgae)
        .onTrue(setCurrentStateCommand(State.INTAKING_GROUND));

    requestMap
        .get(Requests.REEF_SCORE)
        .and(() -> hasCoral)
        .onTrue(
            setCurrentStateCommand(
                State.L4_SCORING)); // TODO take operator selected target into consideration

    requestMap
        .get(Requests.PROCESSOR_SCORE)
        .and(() -> hasAlgae)
        .onTrue(setCurrentStateCommand(State.PROCESSOR_SCORE)); // Processor logic
  }

  /** Set current state */
  public Command setCurrentStateCommand(State desiredState) {
    return Commands.run(
        () -> {
          currentState = desiredState;
        });
  }
}
