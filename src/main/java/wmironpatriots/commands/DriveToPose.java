// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import wmironpatriots.Constants;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.SwerveConstants;
import wmironpatriots.utils.GeomUtil;

public class DriveToPose extends Command {
  // static {
  //   drivekP.initDefault(0.8);
  //   drivekD.initDefault(0.0);
  //   thetakP.initDefault(4.0);
  //   thetakD.initDefault(0.0);
  //   driveMaxVelocity.initDefault(3.8);
  //   driveMaxAcceleration.initDefault(3.0);
  //   driveMaxVelocityAuto.initDefault(3.8);
  //   driveMaxAccelerationAuto.initDefault(2.5);
  //   thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
  //   thetaMaxAcceleration.initDefault(8.0);
  //   thetaMaxVelocityAuto.initDefault(4.0);
  //   thetaMaxAccelerationAuto.initDefault(4.0);
  //   driveTolerance.initDefault(0.01);
  //   thetaTolerance.initDefault(Units.degreesToRadians(1.0));
  //   ffMinRadius.initDefault(0.05);
  //   ffMaxRadius.initDefault(0.1);
  // }

  public static final double LINEAR_P = 0.8;
  public static final double LINEAR_D = 0.0;
  public static final double ANGULAR_P = 0.3;
  public static final double ANGULAR_D = 0.0;

  public static final double MAX_VEL = 3.8;
  public static final double MAX_ACCEL = 3.0;
  public static final double MAX_AUTO_VEL = 3.8;
  public static final double MAX_AUTO_ACCEL = 2.5;
  public static final double MAX_OMEGA = Units.degreesToRadians(360.0);
  public static final double MAX_OMEGA_ACCEL = 8.0;
  public static final double MAX_OMEGA_AUTO = 4.0;
  public static final double MAX_OMEGA_ACCEL_AUTO = 4.0;
  public static final double DRIVE_TOLARANCE = 0.01;
  public static final double ANGULAR_TOLRANCE = Units.degreesToRadians(1.0);
  public static final double ffMinRadius = 0.05;
  public static final double ffMaxRadius = 0.1;

  private final Swerve drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.TICK_SPEED);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.TICK_SPEED);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public boolean isRunning() {
    return running;
  }

  public DriveToPose(Swerve drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    robot = drive::getPose;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Swerve drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
    this(drive, target);
    this.robot = robot;
  }

  public DriveToPose(
      Swerve drive,
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target, robot);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getCurrentVelocities(), drive.getHeading());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointRotation = target.get().getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    running = true;
    driveController.setP(LINEAR_P);
    driveController.setD(LINEAR_D);
    driveController.setTolerance(DRIVE_TOLARANCE);
    thetaController.setP(ANGULAR_P);
    thetaController.setD(ANGULAR_D);
    thetaController.setTolerance(ANGULAR_TOLRANCE);

    // Update constraints
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            DriverStation.isAutonomous() ? MAX_AUTO_VEL : MAX_VEL,
            DriverStation.isAutonomous() ? MAX_AUTO_ACCEL : MAX_ACCEL));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            DriverStation.isAutonomous() ? MAX_OMEGA_AUTO : MAX_OMEGA,
            DriverStation.isAutonomous() ? MAX_OMEGA_ACCEL_AUTO : MAX_OMEGA_ACCEL));

    // Get current pose and target pose
    Pose2d currentPose = robot.get();
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, 0.0)
            + driveController.getSetpoint().velocity * ffScaler;
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new TrapezoidProfile.State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * ffScaler;
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastSetpointRotation = targetPose.getRotation();
    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(SwerveConstants.MAX_LINEAR_SPEED), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * SwerveConstants.MAX_ANGULAR_SPEED, thetaS);

    // Command speeds
    drive.runVelocities(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    // Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    // Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    // Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    // Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    // Logger.recordOutput(
    //     "DriveToPose/Setpoint",
    //     new Pose2d[] {
    //       new Pose2d(
    //           lastSetpointTranslation,
    //           Rotation2d.fromRadians(thetaController.getSetpoint().position))
    //     });
    // Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
