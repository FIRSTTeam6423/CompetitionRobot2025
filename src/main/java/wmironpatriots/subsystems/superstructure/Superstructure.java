// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import wmironpatriots.subsystems.superstructure.chute.Chute;
import wmironpatriots.subsystems.superstructure.elevator.Elevator;
import wmironpatriots.subsystems.superstructure.tail.Tail;
import wmironpatriots.subsystems.superstructure.tail.roller.Roller;
import wmironpatriots.subsystems.swerve.Swerve;

public class Superstructure {
  // * CONSTANTS
  private final Swerve swerve;
  private final Elevator elevator;
  private final Tail tail;
  private final Roller roller;
  private final Chute chute;

  public final Trigger disabledTrigger;

  public Superstructure(Swerve swerve, Elevator elevator, Tail tail, Roller roller, Chute chute) {
    // * INIT SUBSYSTEMS
    this.swerve = swerve;
    this.elevator = elevator;
    this.tail = tail;
    this.roller = roller;
    this.chute = chute;

    elevator.setDefaultCommand(defaultElevatorCmmd());
    tail.setDefaultCommand(defaultTailCmmd());
    roller.setDefaultCommand(defaultRollerCmmd());
    chute.setDefaultCommand(defaultChuteCmmd());

    disabledTrigger = new Trigger(() -> DriverStation.isDisabled());
    Supplier<Pose2d> robotPoseSupplier = () -> swerve.getPose();

    // Turn off brake mode when disabled
    // disabledTrigger.onTrue(tail.setCoasting(true).alongWith(elevator.setCoasting(true)));
    // disabledTrigger.onFalse(tail.setCoasting(false).alongWith(elevator.setCoasting(false)));
  }

  // * DEFAULT COMMANDS
  /** Zeros elevator if it hasn't been zeroed since startup and runs elevator to neutral pose */
  public Command defaultElevatorCmmd() {
    return Commands.sequence(
        elevator.runCurrentZeroingCmmd().onlyIf(() -> !elevator.isZeroed),
        elevator.runPoseCmmd(2).until(() -> elevator.poseRevs - 0.2 <= elevator.targetPoseRevs),
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
  /** Automated intaking sequence; Will try to unjam if jammed */
  public Command runIntakeRoutineCmmd() {
    return Commands.sequence(
            intakeCoralCmmd(),
            outtakeCoralCmmd().andThen(intakeCoralCmmd()).onlyIf(() -> chute.isStuck()))
        .onlyIf(() -> !tail.hasCoral());
  }

  /** Intakes and then indexes coral if it isn't jammed in chute */
  public Command intakeCoralCmmd() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_INTAKING)
        .alongWith(roller.runRollerSpeedCmmd(Roller.SPEED_INTAKING))
        .until(() -> tail.hasCoral())
        .andThen(roller.indexCoralCmmd().onlyIf(() -> tail.hasCoral()));
  }

  /** Unjams coral in intake */
  public Command outtakeCoralCmmd() {
    return chute
        .runChuteSpeedCmmd(Chute.SPEED_OUTAKING)
        .alongWith(roller.runRollerSpeedCmmd(Roller.SPEED_OUTAKING));
  }

  /** Scores to input level */
  public Command scoreCoralCmmd(ReefLevel level) {
    return Commands.parallel(
        tail.runPoseCmmd(Tail.POSE_SAFTEY)
            .until(() -> tail.nearSetpoint())
            .andThen(tail.runPoseCmmd(level.tailPose)),
        elevator.runPoseCmmd(level.elevatorPose).onlyWhile(() -> tail.nearSetpoint()));
  }

  // * ALGAE MANIPULATION
  /** Puts tail out for intaking algae */
  public Command intakeAlgaeCmmd() {
    return Commands.parallel(
            tail.setCoasting(true)
                .andThen(tail.runPoseCmmd(Tail.POSE_MAX).until(() -> roller.hasAlgae()))
                .andThen(tail.setCoasting(false)),
            roller.runRollerSpeedCmmd(Roller.SPEED_OUTAKING).until(() -> roller.hasAlgae()))
        .onlyIf(() -> !tail.hasCoral());
  }

  public Command score() {
    return roller.runRollerSpeedCmmd(Roller.SPEED_SCORING);
  }

  /** Runs roller to score algae */
  public Command scoreAlgaeCmmd() {
    return roller.runRollerSpeedCmmd(Roller.SPEED_SCORING);
  }

  // * SAFETY CONSTRAINTS
  /** Checks if tail is safe from collision */
  private boolean tailSafe() {
    if (elevator.targetPoseRevs < Elevator.POSE_COLLISION) return true;
    var displacement = tail.targetPoseRevs - tail.poseRevs;
    return displacement > 0
        ? elevator.poseRevs >= Elevator.POSE_COLLISION
        : elevator.poseRevs <= Elevator.POSE_COLLISION;
  }

  /** Checks if tail is deployed */
  private boolean tailDeployed() {
    return tail.poseRevs > Tail.POSE_SAFTEY;
  }

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
