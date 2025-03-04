// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import wmironpatriots.Constants.BranchTarget;
import wmironpatriots.Constants.LevelTarget;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.tail.Tail;

/** Cursed superstructure ahh frfr */
public class Superstructure {
  public static enum Requests {
    INTAKE_CHUTE,
    REEF_SCORE,
    OUTTAKE_CORAL,
    OUTTAKE_ALGAE,
    REMOVE_ALGAE_H,
    REMOVE_ALGAE_L,
    CANCEL
  }

  public static enum State {
    IDLE,
    INTAKING_CHUTE,
    L1_SETUP,
    L2_SETUP,
    L3_SETUP,
    L4_SETUP,
    ALGAE_H,
    ALGAE_L,
    REEF_SCORE,
  }

  private final Map<State, Trigger> stateMap = new HashMap<State, Trigger>();
  private State previousState = State.IDLE;
  private State currentState = State.IDLE;
  private final Supplier<BranchTarget> branchSupplier;
  private final Supplier<LevelTarget> levelSupplier;

  private final Timer stateTimer;

  private boolean hasCoral = false;

  public Superstructure(
      Swerve swerve,
      Elevator elevator,
      Tail tail,
      Chute chute,
      Map<Requests, Trigger> requestMap,
      Supplier<BranchTarget> branchSupplier,
      Supplier<LevelTarget> levelSupplier) {
    // Checks for null triggers in requestMap
    for (var request : Requests.values()) {
      if (requestMap.get(request) == null) {
        requestMap.put(request, new Trigger(() -> false));
      }
    }
    for (var state : State.values()) {
      stateMap.put(state, new Trigger(() -> this.currentState == state));
    }
    this.branchSupplier = branchSupplier;
    this.levelSupplier = levelSupplier;

    stateTimer = new Timer();

    /** REQUEST TRIGGERS */
    requestMap
        .get(Requests.INTAKE_CHUTE)
        .and(() -> !hasCoral && !stateTimer.isRunning())
        .onTrue(setCurrentStateCommand(State.INTAKING_CHUTE));

    requestMap
        .get(Requests.REEF_SCORE)
        .and(() -> hasCoral && !stateTimer.isRunning() && levelSupplier.get() == LevelTarget.L1)
        .onTrue(setCurrentStateCommand(State.L1_SETUP));

    requestMap
        .get(Requests.REEF_SCORE)
        .and(() -> hasCoral && !stateTimer.isRunning() && levelSupplier.get() == LevelTarget.L2)
        .onTrue(setCurrentStateCommand(State.L2_SETUP));

    requestMap
        .get(Requests.REEF_SCORE)
        .and(() -> hasCoral && !stateTimer.isRunning() && levelSupplier.get() == LevelTarget.L3)
        .onTrue(setCurrentStateCommand(State.L3_SETUP));

    requestMap
        .get(Requests.REEF_SCORE)
        .and(() -> hasCoral && !stateTimer.isRunning() && levelSupplier.get() == LevelTarget.L4)
        .onTrue(setCurrentStateCommand(State.L4_SETUP));

    /** STATE TRIGGER */
    stateMap.get(State.IDLE);

    // Coral manipulation
    stateMap
        .get(State.INTAKING_CHUTE)
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS)
                .until(() -> isTailSafe(elevator, tail))
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS)))
        .whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_INTAKING))
        .and(() -> elevator.inSetpointRange())
        .whileTrue(tail.setRollerSpeedCmmd(Tail.INTAKING_SPEEDS))
        .whileTrue(chute.runChuteSpeedCmmd(Chute.INTAKE_SPEED))
        .and(() -> tail.hasCoral())
        .onTrue(setCoralStatus(true))
        .onTrue(setCurrentStateCommand(State.IDLE));

    stateMap
        .get(State.L1_SETUP)
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS)
                .until(() -> isTailSafe(elevator, tail))
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS)))
        .whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_L1))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L2_SETUP)
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS)
                .until(() -> isTailSafe(elevator, tail))
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS)))
        .whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_L2))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L3_SETUP)
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS)
                .until(() -> isTailSafe(elevator, tail))
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS)))
        .whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_L3))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.L4_SETUP)
        .whileTrue(tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS))
        .whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_L4))
        .and(() -> elevator.inSetpointRange())
        .onTrue(setCurrentStateCommand(State.REEF_SCORE));

    stateMap
        .get(State.REEF_SCORE)
        .whileTrue(elevator.setTargetPoseCmmd(levelSupplier.get().elevatorPoseRevs))
        .whileTrue(tail.setTargetPoseCmmd(levelSupplier.get().tailPoseRads))
        .whileTrue(tail.setRollerSpeedCmmd(Tail.OUTTAKING_SPEEDS))
        .and(() -> !tail.hasCoral())
        .onTrue(setCoralStatus(false))
        .onTrue(setCurrentStateCommand(State.IDLE));
  }

  /** Checks to see if tail will hit top of carriage when stowed */
  public static boolean isTailSafe(Elevator elevator, Tail tail) {
    double vel = elevator.getVelocity();
    vel = MathUtil.applyDeadband(vel, 0.2); // deadbands speed

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

  /** Set current state */
  private Command setCurrentStateCommand(State desiredState) {
    return Commands.runOnce(
        () -> {
          previousState = currentState;
          System.out.println("new state " + desiredState);
          currentState = desiredState;

          if (desiredState == State.IDLE) stateTimer.reset();
          else if (previousState != State.IDLE) stateTimer.restart();
          else stateTimer.start();
        });
  }
}
