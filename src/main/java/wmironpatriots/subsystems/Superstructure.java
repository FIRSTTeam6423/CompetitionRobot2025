// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.tail.Tail;

public class Superstructure {
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
        elevator.runPoseCmmd(Elevator.POSE_STOWED).until(() -> elevator.nearSetpoint()),
        elevator.stopElevatorCmmd());
  }

  public Command defaultTailCmmd() {
    return Commands.sequence(
        tail.runCurrentZeroingCmmd().onlyIf(() -> !tail.isZeroed).alongWith(tail.vectorCoral()),
        tail.runPoseCmmd(Tail.POSE_STOWED),
        tail.stopTailCmmd());
  }

  public Command defaultChuteCmmd() {
    return chute.runChuteSpeedCmmd(0.0);
  }

  public Command intakeCoral() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_INTAKING)
        .alongWith(tail.runRollerSpeed(Tail.SPEED_INTAKING))
        .until(() -> tail.hasCoral())
        .andThen(tail.vectorCoral());
  }
}
