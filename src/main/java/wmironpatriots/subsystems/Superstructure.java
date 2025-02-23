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
import wmironpatriots.subsystems.tail.Tail;

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
    CANCEL
  }

  public static enum State {
    IDLE,
    INTAKING_GROUND,
    INTAKING_CHUTE,
    L1_SETUP,
    L2_SETUP,
    L3_SETUP,
    L4_SETUP,
    REEF_SCORE,
    PROCESSOR_SCORE
  }

  private final Map<State, Trigger> stateMap = new HashMap<State, Trigger>();
  private State currentState = State.IDLE;

  private boolean hasCoral = false;
  private boolean hasAlgae = false;

  public Superstructure(Elevator elevator, Tail tail, Map<Requests, Trigger> requestMap) {
    // Checks for null triggers in requestMap
    for (var request : Requests.values()) {
      if (requestMap.get(request) == null) {
        requestMap.put(request, new Trigger(() -> false));
      }
    }
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
                State.L4_SETUP)); // TODO take operator selected target into consideration

    requestMap
        .get(Requests.PROCESSOR_SCORE)
        .and(() -> hasAlgae)
        .onTrue(setCurrentStateCommand(State.PROCESSOR_SCORE)); // Processor logic

    /** STATE TRIGGER */
    stateMap
        .get(State.IDLE)
        .whileTrue(tail.setTargetPoseCommand(Tail.POSE_OUT_RADS))
        .whileTrue(elevator.setTargetPoseCommand(0.0));

    // Coral manipulation
    stateMap
        .get(State.INTAKING_CHUTE)
        .whileTrue(elevator.setTargetPoseCommand(Elevator.POSE_INTAKING))
        .and(() -> isTailSafe(elevator, tail))
        .whileTrue(tail.setTargetPoseCommand(Tail.POSE_IN_RADS))
        .and(() -> elevator.inSetpointRange())
        .whileTrue(tail.runRollersCommand(Tail.INTAKING_SPEEDS)) // TODO run chute
        .and(() -> tail.hasCoral(true))
        .onTrue(setCoralStatus(true))
        .onTrue(setCurrentStateCommand(State.IDLE));

    stateMap
        .get(State.L1_SETUP)
        .whileTrue(elevator.setTargetPoseCommand(Elevator.POSE_L1))
        .and(() -> isTailSafe(elevator, tail))
        .whileTrue(tail.setTargetPoseCommand(Tail.POSE_IN_RADS))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L2_SETUP)
        .whileTrue(elevator.setTargetPoseCommand(Elevator.POSE_L2))
        .and(() -> isTailSafe(elevator, tail))
        .whileTrue(tail.setTargetPoseCommand(Tail.POSE_IN_RADS))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L3_SETUP)
        .whileTrue(elevator.setTargetPoseCommand(Elevator.POSE_L3))
        .and(() -> isTailSafe(elevator, tail))
        .whileTrue(tail.setTargetPoseCommand(Tail.POSE_IN_RADS))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L4_SETUP)
        .whileTrue(elevator.setTargetPoseCommand(Elevator.POSE_L4))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.REEF_SCORE)
        .whileTrue(tail.runRollersCommand(Tail.OUTTAKING_SPEEDS))
        .and(() -> !tail.hasCoral(false))
        .onTrue(setCoralStatus(false))
        .onTrue(setCurrentStateCommand(State.IDLE));
  }

  /** Checks to see if tail will hit top of carriage when stowed */
  private boolean isTailSafe(Elevator elevator, Tail tail) {
    double vel = elevator.getVelocity();
    if (vel > 0 && elevator.getPose() < Elevator.POSE_MAX_CARRIAGE_STAGE_ONE) {
      return false;
    } else if (vel < 0 && elevator.getPose() > Elevator.POSE_MAX_CARRIAGE_STAGE_ONE) {
      return false;
    }
    return true;
  }

  private Command setCoralStatus(boolean hasCoral) {
    return Commands.runOnce(() -> this.hasCoral = hasCoral);
  }

  private Command setAlgaeStatus(boolean hasAlgae) {
    return Commands.runOnce(() -> this.hasAlgae = hasAlgae);
  }

  /** Set current state */
  private Command setCurrentStateCommand(State desiredState) {
    return Commands.runOnce(
        () -> {
          System.out.println("new state " + desiredState);
          currentState = desiredState;
        });
  }
}
