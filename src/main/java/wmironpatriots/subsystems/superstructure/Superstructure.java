// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import wmironpatriots.subsystems.superstructure.chute.Chute;
import wmironpatriots.subsystems.superstructure.chute.ChuteIOComp;
import wmironpatriots.subsystems.superstructure.elevator.Elevator;
import wmironpatriots.subsystems.superstructure.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.superstructure.tail.Tail;
import wmironpatriots.subsystems.superstructure.tail.TailIOComp;
import wmironpatriots.subsystems.superstructure.tail.roller.Roller;
import wmironpatriots.subsystems.superstructure.tail.roller.RollerIOComp;

public class Superstructure {
  private final Elevator elevator;
  private final Tail tail;
  private final Roller roller;
  private final Chute chute;

  public Superstructure() {
    // * INIT SUBSYSTEMS
    elevator = new ElevatorIOComp();
    tail = new TailIOComp();
    roller = new RollerIOComp();
    chute = new ChuteIOComp();

    elevator.setDefaultCommand(defaultElevatorCmmd());
    tail.setDefaultCommand(defaultTailCmmd());
    roller.setDefaultCommand(defaultRollerCmmd());
    chute.setDefaultCommand(defaultChuteCmmd());
  }

  // * DEFAULT COMMANDS
  /** Zeros elevator if it hasn't been zeroed since startup and runs elevator to neutral pose */
  public Command defaultElevatorCmmd() {
    return Commands.sequence(
        elevator.runCurrentZeroingCmmd().onlyIf(() -> !elevator.isZeroed),
        elevator.runPoseCmmd(2).until(() -> elevator.poseRevs <= elevator.targetPoseRevs),
        elevator.stopElevatorCmmd());
  }

  /** Zeros tail if it hasn't been zeroed since startup and runs tail to neutral pose */
  public Command defaultTailCmmd() {
    return Commands.sequence(
        tail.runCurrentZeroingCmmd().onlyIf(() -> !tail.isZeroed),
        new WaitUntilCommand(() -> elevator.poseRevs <= Elevator.POSE_COLLISION),
        tail.runPoseCmmd(Tail.POSE_STOWED).until(() -> tail.nearSetpoint()));
  }

  /** Turns off tail rollers */
  public Command defaultRollerCmmd() {
    return roller.runRollerSpeedCmmd(0.0);
  }

  /** Turns off chute rollers */
  public Command defaultChuteCmmd() {
    return chute.runChuteSpeedCmmd(0.0);
  }

  // * CORAL MANIPULATION
  /** Intakes and indexes coral */
  public Command intakeCoralCmmd() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_INTAKING)
        .alongWith(roller.runRollerSpeedCmmd(Roller.SPEED_INTAKING))
        .until(() -> tail.hasCoral())
        .andThen(roller.indexCoralCmmd())
        .onlyIf(() -> !tail.hasCoral());
  }

  /** Unjams coral in intake */
  public Command outtakeCoralCmmd() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_OUTAKING)
        .alongWith(roller.runRollerSpeedCmmd(Roller.SPEED_OUTAKING))
        .onlyIf(() -> tail.hasCoral());
  }

  /** Scores to input level */
  public Command scoreCoralCmmd(ReefLevel level) {
    return Commands.parallel(
            tail.runPoseCmmd(Tail.POSE_MAX)
                .until(() -> tailSafe())
                .andThen(tail.runPoseCmmd(level.tailPose)),
            elevator
                .runPoseCmmd(level.elevatorPose)
                .onlyWhile(() -> tail.poseRevs > Tail.POSE_SAFTEY),
            roller
                .runRollerSpeedCmmd(Roller.SPEED_SCORING)
                .onlyWhile(() -> tail.nearSetpoint() && elevator.nearSetpoint()))
        .onlyIf(() -> tail.hasCoral());
  }

  /** Checks if tail is safe from collision */
  private boolean tailSafe() {
    var displacement = tail.targetPoseRevs - tail.poseRevs;
    return displacement > 0
        ? elevator.poseRevs >= Elevator.POSE_COLLISION
        : elevator.poseRevs <= Elevator.POSE_COLLISION;
  }

  // * STATIC SCORE TARGET ENUMS
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
}
