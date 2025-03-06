// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static wmironpatriots.Constants.DT_TIME;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.swerve.gyro.Gyro;
import wmironpatriots.subsystems.swerve.gyro.GyroIOComp;
import wmironpatriots.subsystems.swerve.gyro.GyroIOSim;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.Module.ModuleConfig;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;
import wmironpatriots.subsystems.swerve.module.ModuleIOSim;
import wmironpatriots.util.mechanismUtil.IronSubsystem;

public class Swerve implements IronSubsystem {
  /** CONSTANTS */
  // mech constants
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
        new ModuleConfig(1, 1, 2, 9, 0.0, true),
        new ModuleConfig(2, 3, 4, 10, 0.0, true),
        new ModuleConfig(3, 5, 6, 11, 0.0, true),
        new ModuleConfig(4, 7, 8, 12, 0.0, true),
      };

  // TODO check with
  // https://www.chiefdelphi.com/t/how-to-calculate-the-max-free-speed-of-a-swerve/400741/3
  public static double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(3);
  public static double MAX_LINEAR_ACCEL_MPS_SQRD = 2;
  public static double MAX_ANGULAR_SPEED_RADS_PER_SEC =
      MAX_LINEAR_SPEED_MPS / (Math.hypot(TRACK_WIDTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0));

  /** VARIABLES */
  private final Module[] modules;

  private final Gyro gyro;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odo;

  private final Field2d f2d;
  private final StructArrayPublisher<SwerveModuleState> publisher;

  private final PIDController linearFeedback, angularFeedback, headingFeedback;

  private final Optional<SwerveDriveSimulation> simulation;

  public Swerve(Optional<SwerveDriveSimulation> simulation) {
    modules = new Module[4];
    if (Robot.isReal()) {
      gyro = new GyroIOComp();
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new ModuleIOComp(MODULE_CONFIGS[i]);
      }
    } else {
      gyro = new GyroIOSim(simulation.get().getGyroSimulation());
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new ModuleIOSim(MODULE_CONFIGS[i], simulation.get().getModules()[i]);
      }
    }

    kinematics = new SwerveDriveKinematics(MODULE_LOCS);
    odo = new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePoses(), new Pose2d());

    linearFeedback = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    angularFeedback = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    headingFeedback = new PIDController(HEADING_P, HEADING_I, HEADING_D);

    f2d = new Field2d();
    publisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("states", SwerveModuleState.struct)
            .publish();

    this.simulation = simulation;
  }

  @Override
  public void periodic() {
    for (Module module : modules) {
      module.periodic();

      updateOdo();
      f2d.setRobotPose(getPose());
      publisher.set(getModuleStates());
      stop();
    }
  }

  public Command teleopSwerveCmmd(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return this.run(
        () -> {
          var fieldRelativeVelocities =
              new ChassisSpeeds(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble());
          fieldRelativeVelocities =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeVelocities,
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getPose().getRotation()
                      : getPose()
                          .getRotation()
                          .plus(
                              Rotation2d.fromDegrees(180))); // Flips orientation if on red alliance

          this.runVelocities(fieldRelativeVelocities, false);
        });
  }

  public void runVelocities(ChassisSpeeds velocities, boolean openloopEnabled) {
    velocities = ChassisSpeeds.discretize(velocities, DT_TIME);

    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(velocities);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_LINEAR_SPEED_MPS);

    for (int i = 0; i < desiredStates.length; i++) {
      if (openloopEnabled) {
        ChassisSpeeds currentVelocities = getRobotRelativeVelocities();
        boolean focEnabled =
            Math.sqrt(
                    Math.pow(currentVelocities.vxMetersPerSecond, 2)
                        + Math.pow(
                            currentVelocities.vyMetersPerSecond,
                            2)) // converts linear velocity components to linear velocity
                < 1 * MAX_LINEAR_SPEED_MPS;

        double driveVolts =
            desiredStates[i].speedMetersPerSecond / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt;
        modules[i].runOpenloopSetpoint(desiredStates[i], focEnabled);
      } else {
        modules[i].runSetpoint(desiredStates[i]);
      }
    }
  }

  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  public void xStop() {}

  public void updateOdo() {
    odo.update(getHeading(), getModulePoses());
  }

  public void resetOdo(Pose2d pose) {
    odo.resetPose(pose);
    if (simulation.isPresent()) {
      simulation.get().setSimulationWorldPose(pose);
      simulation.get().setRobotSpeeds(new ChassisSpeeds());
    }
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return Robot.isReal()
        ? odo.getEstimatedPosition()
        : simulation.get().getSimulatedDriveTrainPose();
  }

  public ChassisSpeeds getRobotRelativeVelocities() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getHeading());
  }

  public ChassisSpeeds getFieldRelativeVelocities() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModulePosition[] getModulePoses() {
    SwerveModulePosition[] poses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < poses.length; i++) {
      poses[i] = modules[i].getModulePose();
    }
    return poses;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < states.length; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }
}
