// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.deviceUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import wmironpatriots.Constants.BranchTarget;
import wmironpatriots.Constants.LevelTarget;

public class OperatorController {
  private final CommandXboxController controller;
  private LevelTarget levelTarget;
  private BranchTarget branchTarget;

  public OperatorController(int port) {
    levelTarget = LevelTarget.L1;
    branchTarget = BranchTarget.A;

    controller = new CommandXboxController(port);

    controller
        .povUp()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.A));

    controller
        .povUp()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.B));

    controller
        .povUpRight()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.C));

    controller
        .povUpRight()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.D));

    controller
        .povRight()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.E));

    controller
        .povRight()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.F));

    controller
        .povDownRight()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.G));

    controller
        .povDownRight()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.H));

    controller
        .povDown()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.I));

    controller
        .povDown()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.J));

    controller
        .povDownLeft()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.K));

    controller
        .povDownLeft()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.L));

    controller
        .povLeft()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.M));

    controller
        .povLeft()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.N));

    controller
        .povUpLeft()
        .and(() -> controller.leftBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.O));

    controller
        .povUpLeft()
        .and(() -> controller.rightBumper().getAsBoolean())
        .onTrue(setBranchTargetCmmd(BranchTarget.P));

    controller.a().onTrue(setLevelTargetCmmd(LevelTarget.L1));

    controller.x().onTrue(setLevelTargetCmmd(LevelTarget.L2));

    controller.b().onTrue(setLevelTargetCmmd(LevelTarget.L3));

    controller.y().onTrue(setLevelTargetCmmd(LevelTarget.L4));
  }

  private Command setBranchTargetCmmd(BranchTarget target) {
    return Commands.runOnce(() -> branchTarget = target);
  }

  private Command setLevelTargetCmmd(LevelTarget target) {
    return Commands.runOnce(() -> levelTarget = target);
  }

  public BranchTarget getBranchTarget() {
    return this.branchTarget;
  }

  public LevelTarget getLevelTarget() {
    return this.levelTarget;
  }
}
