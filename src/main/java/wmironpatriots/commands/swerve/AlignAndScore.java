package wmironpatriots.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ScoreTarget;
import wmironpatriots.subsystems.swerve.Swerve;

public class AlignAndScore extends Command {
  public AlignAndScore(
    Swerve swerve, 
    Superstructure superstructure, 
    Supplier<ScoreTarget> scoreTargetSupplier, 
    Supplier<AlignTarget> alignTargetSupplier,
    BooleanSupplier canScoreSupplier) {
  }

  public static enum AlignTarget {
    A,
    B,
    C
  }
}
