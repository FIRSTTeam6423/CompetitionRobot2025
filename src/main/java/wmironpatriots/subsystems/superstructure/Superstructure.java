// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.superstructure.arm.Arm;
import wmironpatriots.subsystems.superstructure.arm.ArmIOComp;
import wmironpatriots.subsystems.superstructure.elevator.Elevator;
import wmironpatriots.subsystems.superstructure.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.superstructure.rollers.Rollers;
import wmironpatriots.subsystems.superstructure.rollers.RollersIOComp;

public class Superstructure {
  private final Elevator elevator;
  private final Arm arm;
  private final Rollers rollers;
  
  public static Superstructure create() {
    if (Robot.isReal()) {
      return new Superstructure(
          new ElevatorIOComp(), new ArmIOComp(), new RollersIOComp());
    } else {
      return new Superstructure(
          new ElevatorIOComp(), new ArmIOComp(), new RollersIOComp());
    }
  }

  protected Superstructure(Elevator elevatorIO, Arm armIO, Rollers rollersIO) {
    elevator = elevatorIO;
    arm = armIO;
    rollers = rollersIO;

    elevator.setDefaultCommand(elevatorDefaultCmd());
    arm.setDefaultCommand(armDefaultCmd());
  }

  // * SUBSYSTEM DEFAULT CMDS
  private Command elevatorDefaultCmd() {
    return Commands.sequence(
        elevator.runCurrentZeroingCmd().onlyIf(() -> !elevator.isInitalized()),
        elevator
            .runPoseCmd(1)
            .until(elevator::nearSetpoint)
            .finallyDo(() -> elevator.stopMotors()));
  }

  private Command armDefaultCmd() {
    return Commands.sequence(
        arm.runCurrentZeroingCmd().onlyIf(() -> !elevator.isInitalized()),
        arm.runPoseCmd(0.0).until(arm::nearSetpoint).finallyDo(() -> arm.stopMotors()));
  }

  /**
   * Scores coral to desired reef level
   * 
   * @param level desired reef level as {@link ScoreTarget}
   * @return
   */
  public Command scoreCoralCmd(ScoreTarget level) {
    return prepAndScoreCoralCmd(level, () -> true);
  }

  /**
   * Sets up superstructure in scoring position, 
   * then waits until {@link BooleanSupplier} returns true
   * before scoring
   * 
   * @param level desired reef level as {@link ScoreTarget}
   * @param scoreCondition {@link BooleanSupplier} that triggers scoring
   */
  public Command prepAndScoreCoralCmd(ScoreTarget level, BooleanSupplier scoreCondition) {
    return Commands.parallel(
        arm.runPoseCmd(level.armPose),
        Commands.waitUntil(arm::nearSetpoint).andThen(elevator.runPoseCmd(level.elevatorPose)),
        Commands.sequence(
            Commands.waitUntil(() -> arm.nearSetpoint() && elevator.nearSetpoint()),
            Commands.waitUntil(scoreCondition),
            rollers.runRollerSpeed(Rollers.SPEED_SCORING).until(() -> !arm.hasCoral())));
  }

  public static enum ScoreTarget {
    L1(Elevator.POSE_L1_REVS, 0.0),
    L2(Elevator.POSE_L2_REVS, 0.0),
    L3(Elevator.POSE_L3_REVS, 0.0),
    L4(Elevator.POSE_L4_REVS, 0.0);

    double elevatorPose;
    double armPose;

    private ScoreTarget(double elevatorPose, double armPose) {
      this.elevatorPose = elevatorPose;
      this.armPose = armPose;
    }
  }
}
