// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.subsystems.swerve.gyro.Gyro;
import wmironpatriots.subsystems.swerve.gyro.GyroIOComp;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.Module.ModuleConfig;
import wmironpatriots.subsystems.vision.Vision.PoseEstimate;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystem;

public class Swerve implements LoggedSubsystem {
  // * CONSTANTS
  public static final double MASS_KG = 54.8847;
  public static final double MOI = 5.503;
  public static final double SIDE_LENGTH_METERS = 0.7239;
  public static final double BUMPER_WIDTH_METER = 0.0889; // TODO add value
  public static final double TRACK_WIDTH_METERS = 0.596201754;

  public static final Translation2d[] MODULE_LOCS =
      new Translation2d[] {
        new Translation2d(0.381, 0.381),
        new Translation2d(0.381, -0.381),
        new Translation2d(-0.381, 0.381),
        new Translation2d(-0.381, -0.381)
      };

  public static final Rotation2d ALLIANCE_ROTATION =
      Rotation2d.fromRotations(
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0.5);

  public static final double MAX_LINEAR_SPEED = 5.11;
  public static final double MAX_ANGULAR_SPEED = 0.0;

  public static final double LINEAR_P = 0.0;
  public static final double LINEAR_I = 0.0;
  public static final double LINEAR_D = 0.0;

  public static final double ANGULAR_P = 0.0;
  public static final double ANGULAR_I = 0.0;
  public static final double ANGULAR_D = 0.0;

  public static final double HEADING_P = 0.0;
  public static final double HEADING_I = 0.0;
  public static final double HEADING_D = 0.0;

  public static final ModuleConfig[] MODULE_CONFIGS =
      new ModuleConfig[] {
        new ModuleConfig(MATRIXID.BL_PIVOT, MATRIXID.BL_DRIVE, MATRIXID.BL_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.FL_PIVOT, MATRIXID.FL_DRIVE, MATRIXID.FL_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.FR_PIVOT, MATRIXID.FR_DRIVE, MATRIXID.FR_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.BR_PIVOT, MATRIXID.BR_DRIVE, MATRIXID.BR_CANCODER, 0.0, true)
      };

  private final Module[] modules;
  private final Gyro gyro;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odo;

  private final PIDController angularFeedback, linearFeedback;

  private final Field2d f2d;

  public Swerve() {
    gyro = new GyroIOComp();
    modules = new ModuleIOComp[MODULE_CONFIGS.length];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new ModuleIOComp(MODULE_CONFIGS[i]);
    }

    kinematics = new SwerveDriveKinematics(MODULE_LOCS);
    odo =
        new SwerveDrivePoseEstimator(
            kinematics, getHeading(), getSwerveModulePoses(), new Pose2d());

    f2d = new Field2d();

    angularFeedback = new PIDController(4.5, 0.0, 0.05);
    angularFeedback.enableContinuousInput(0, 2 * Math.PI);
    angularFeedback.setTolerance(0.0523599);
    linearFeedback = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    odo.update(getHeading(), getSwerveModulePoses());
    f2d.setRobotPose(getPose());

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  public Command drive(
      DoubleSupplier xVelocity, DoubleSupplier yVelocity, Rotation2d desiredHeading) {
    return drive(
        xVelocity,
        yVelocity,
        () -> angularFeedback.calculate(getHeading().getRadians(), desiredHeading.getRadians()));
  }

  public Command drive(
      DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier omegaVelocity) {
    return this.run(
        () ->
            runVelocities(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity.getAsDouble(),
                    yVelocity.getAsDouble(),
                    omegaVelocity.getAsDouble(),
                    getHeading().plus(ALLIANCE_ROTATION))));
  }

  public void runVelocities(ChassisSpeeds velocities) {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(velocities, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED);
    for (int i = 0; i < states.length; i++) {
      modules[i].runModuleSetpoint(states[i]);
    }
  }

  public void updateVisionEstimates(PoseEstimate... poses) {
    Pose3d[] estimates = new Pose3d[poses.length];
    for (int i = 0; i < poses.length; i++) {
      odo.addVisionMeasurement(
        poses[i].pose().estimatedPose.toPose2d(), 
        poses[i].pose().timestampSeconds, 
        poses[i].stdevs());
      
      f2d
        .getObject("estimated pose " + i)
        .setPose(poses[i].pose().estimatedPose.toPose2d());
    }
  }

  public void resetOdo(Pose2d pose) {
    odo.resetPose(pose);
  }

  public void stop() {
    for (Module module : modules) {
      module.stopModule();
    }
  }

  public Pose2d getPose() {
    return odo.getEstimatedPosition();
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }

  public SwerveModulePosition[] getSwerveModulePoses() {
    SwerveModulePosition[] poses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      poses[i] = modules[i].getModulePose();
    }
    return poses;
  }
}
