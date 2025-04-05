// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands.swerve;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ScoreTarget;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.SwerveConstants;

public class AlignAndScore extends Command {
  // * CONSTANTS
  public static final Distance TARGET_POSE_OFFSET = Meters.of(0.45);

  private Swerve swerve;
  private Superstructure superstructure;

  private Supplier<ScoreTarget> scoreTargetSupplier;
  private Supplier<AlignTarget> alignTargetSupplier;

  private final BooleanSupplier canScoreSupplier;

  private final DoubleSupplier xVelSupplier, yVelSupplier, omegaVelSupplier;

  private final ProfiledPIDController linearController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController angularController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  double initialDisplacement = 0.0;
  Pose2d targetPose, setpoint, pose;

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
    targetPose = alignTargetSupplier.get().pose;
    pose = swerve.getPose();

    initialDisplacement =
        Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY());

    setpoint = calculateSetpoint();
  }

  @Override
  public void execute() {
    targetPose = alignTargetSupplier.get().pose;
    pose = swerve.getPose();

    setpoint = calculateSetpoint();

    Vector<N3> difference =
        VecBuilder.fill(
            pose.getX() - setpoint.getX(),
            pose.getY() - setpoint.getY(),
            MathUtil.angleModulus(
                pose.getRotation().getRadians() - setpoint.getRotation().getRadians()));
    double out = linearController.calculate(difference.norm(), 0);
    Vector<N3> velocities = difference.unit().times(out);
    if (!atPose()) {
      swerve.runVelocities(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              velocities.get(0), velocities.get(1), velocities.get(2) / SwerveConstants.RADIUS.in(Meters), pose.getRotation()));
    }
  }

  private boolean atPose() {
    return pose.getTranslation().minus(targetPose.getTranslation()).getNorm() < 0.01 && pose.getRotation().minus(targetPose.getRotation()).getDegrees() < 2;
  }

  public Pose2d calculateSetpoint() {
    double headingRads = targetPose.getRotation().plus(Rotation2d.k180deg).getRadians();
    double displacement = Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - pose.getY());
    double prop = displacement / initialDisplacement;
    return new Pose2d(
        (TARGET_POSE_OFFSET.in(Meters) * prop * Math.cos(headingRads)) + targetPose.getX(),
        (TARGET_POSE_OFFSET.in(Meters) * prop * Math.sin(headingRads)),
        targetPose.getRotation());
  }

  public static enum AlignTarget {
    A(new Pose2d());

    public final Pose2d pose;

    private AlignTarget(Pose2d pose) {
      this.pose = pose;
    }
  }
}
