package wmironpatriots.util.deviceUtil;

import wmironpatriots.commands.swerve.AlignAndScore.AlignTarget;
import wmironpatriots.subsystems.superstructure.Superstructure.ScoreTarget;

/**
 * Class for handling operator dashboard inputs
 */
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
