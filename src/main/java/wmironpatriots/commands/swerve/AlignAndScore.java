package wmironpatriots.commands.swerve;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ScoreTarget;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.SwerveConstants;

public class AlignAndScore extends Command {
  private Swerve swerve;
  private Superstructure superstructure;

  private Supplier<ScoreTarget> scoreTargetSupplier;
  private Supplier<AlignTarget> alignTargetSupplier;

  private final BooleanSupplier canScoreSupplier;
  
  private final DoubleSupplier xVelSupplier, yVelSupplier, omegaVelSupplier;

  private double initialDisplacement = 0.0;

  public AlignAndScore(
    Swerve swerve, 
    Superstructure superstructure, 
    Supplier<ScoreTarget> scoreTargetSupplier, 
    Supplier<AlignTarget> alignTargetSupplier,
    BooleanSupplier canScoreSupplier,
    DoubleSupplier xVelocitySupplier,
    DoubleSupplier yVelocitySupplier,
    DoubleSupplier omegaVelocitySupplier) {
      this.swerve = swerve;
      this.superstructure = superstructure;

      this.scoreTargetSupplier = scoreTargetSupplier;
      this.alignTargetSupplier = alignTargetSupplier;

      this.canScoreSupplier = canScoreSupplier;

      xVelSupplier = xVelocitySupplier;
      yVelSupplier = yVelocitySupplier;
      omegaVelSupplier = omegaVelocitySupplier;
  }
  
  @Override
  public void initialize() {
    Pose2d targetPose = alignTargetSupplier.get().pose;
    Pose2d pose = swerve.getPose();

    initialDisplacement = Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY());
  }

  @Override
  public void execute() {
  }

  public Command withNewTarget() {
    return this;
  }

  @Override
  public void end(boolean interrupted) {}

  public static enum AlignTarget {
    A(new Pose2d());

    public final Pose2d pose;

    private AlignTarget(Pose2d pose) {
      this.pose = pose;
    }
  }
}