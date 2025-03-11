// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.tail.Tail;

public class Superstructure {
  public static enum ReefBranch {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
  };

  public static enum ReefLevel {
    L1(Elevator.POSE_L1, Tail.POSE_L1),
    L2(Elevator.POSE_L2, Tail.POSE_L2),
    L3(Elevator.POSE_L3, Tail.POSE_L3),
    L4(Elevator.POSE_L4, Tail.POSE_L4);
    private final double elevatorPose, tailPose;

    private ReefLevel(double elevatorPose, double tailPose) {
      this.elevatorPose = elevatorPose;
      this.tailPose = tailPose;
    }
  };

  private final Elevator elevator;
  private final Tail tail;
  private final Chute chute;

  private boolean hasCoral;

  public Superstructure(Elevator elevator, Tail tail, Chute chute) {
    this.elevator = elevator;
    this.tail = tail;
    this.chute = chute;

    hasCoral = tail.hasCoral();
  }

  public Command defaultElevatorCmmd() {
    return Commands.sequence(
        elevator.runCurrentZeroingCmmd().onlyIf(() -> !elevator.isZeroed),
        elevator.runPoseCmmd(2).until(() -> elevator.underSetpoint()),
        elevator.stopElevatorCmmd());
  }

  public Command defaultTailCmmd() {
    return Commands.sequence(
        tail.runRollerSpeed(0).until(() -> true), // cooked
        tail.runCurrentZeroingCmmd().onlyIf(() -> !tail.isZeroed),
        new WaitUntilCommand(() -> elevator.getPose() <= 8.2),
        tail.runPoseCmmd(Tail.POSE_STOWED).until(() -> tail.nearSetpoint()));
  }

  public Command defaultChuteCmmd() {
    return chute.runChuteSpeedCmmd(0.0);
  }

  /** Intakes and indexes coral automatically */
  public Command intakeCoral() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_INTAKING)
        .alongWith(tail.runRollerSpeed(Tail.SPEED_INTAKING))
        .until(() -> tail.hasCoral())
        .andThen(tail.indexCoral());
  }

  public Command outtakeCoral() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_OUTAKING)
        .alongWith(tail.runRollerSpeed(Tail.SPEED_OUTAKING));
  }

  /** Scores to input level */
  public Command score(ReefLevel level) {
    return tail.runPoseCmmd(Tail.POSE_MAX)
        .until(() -> elevator.getPose() >= 8.2)
        .andThen(tail.runPoseCmmd(level.tailPose))
        .alongWith(
            elevator
                .runPoseCmmd(level.elevatorPose)
                .onlyIf(() -> tail.getPose() > 4.5)
                .repeatedly());
  }
}
