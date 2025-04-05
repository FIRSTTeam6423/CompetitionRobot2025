// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.deviceUtil;

import wmironpatriots.commands.swerve.AlignAndScore.AlignTarget;
import wmironpatriots.subsystems.superstructure.Superstructure.ScoreTarget;

/** Class for handling operator dashboard inputs */
public class OperatorDashboard {
  private AlignTarget alignTarget = AlignTarget.A;
  private ScoreTarget scoreTarget = ScoreTarget.L1;

  public AlignTarget getAlignTarget() {
    return alignTarget;
  }

  public ScoreTarget getScoreTarget() {
    return scoreTarget;
  }

  public void poll() {}
}
