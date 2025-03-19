// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static edu.wpi.first.units.Units.Volt;
import static wmironpatriots.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Arrays;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import wmironpatriots.Constants.FLAGS;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.swerve.gyro.Gyro;
import wmironpatriots.subsystems.swerve.gyro.GyroIOComp;
import wmironpatriots.subsystems.swerve.gyro.GyroIOSim;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;
import wmironpatriots.subsystems.swerve.module.ModuleIOSim;
import wmironpatriots.subsystems.vision.Vision.PoseEstimate;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystem;

public class Swerve implements LoggedSubsystem {
  private final Module[] modules;
  private final Gyro gyro;

  private final SwerveDriveKinematics kinematics;

  public static final Lock odoLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  private final SwerveDrivePoseEstimator odo;

  private final PIDController angularFeedback, linearFeedback, linearYFeedback;
  private final SysIdRoutine angularCharacterization, linearCharacterization, pivotCharacterization;

  private final Optional<SelfControlledSwerveDriveSimulation> simulation;

  private final Field2d f2d;
  StructArrayPublisher<SwerveModuleState> swervePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
          .publish();
  StructArrayPublisher<SwerveModuleState> setpoint =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("setpoints", SwerveModuleState.struct)
          .publish();

  public Swerve() {
    // * INIT HARDWARE
    if (Robot.isReal()) {
      gyro = new GyroIOComp();
      modules = new ModuleIOComp[MODULE_CONFIGS.length];
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new ModuleIOComp(MODULE_CONFIGS[i]);
      }

      simulation = Optional.empty();
    } else {
      simulation =
          Optional.of(
              new SelfControlledSwerveDriveSimulation(
                  new SwerveDriveSimulation(
                      driveTrainSimulationConfig.get(), new Pose2d(3.28, 3.28, new Rotation2d()))));
      simulation.get().getDriveTrainSimulation().removeAllFixtures();
      SimulatedArena.getInstance()
          .addDriveTrainSimulation(simulation.get().getDriveTrainSimulation());

      gyro = new GyroIOSim(simulation.get().getDriveTrainSimulation().getGyroSimulation());
      modules = new ModuleIOSim[MODULE_CONFIGS.length];
      for (int i = 0; i < modules.length; i++) {
        modules[i] =
            new ModuleIOSim(
                MODULE_CONFIGS[i], simulation.get().getDriveTrainSimulation().getModules()[i]);
      }
    }

    kinematics = new SwerveDriveKinematics(MODULE_LOCS);
    odo =
        new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());

    f2d = new Field2d();
    SmartDashboard.putData(f2d);

    angularFeedback = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    angularFeedback.enableContinuousInput(-Math.PI, Math.PI);
    linearFeedback = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    linearYFeedback = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

    angularCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volt.of(4),
                null,
                state -> SignalLogger.writeString("SysidAngularState", state.toString())),
            new Mechanism(
                volts -> {
                  modules[0].runCharacterizationAmps(
                      Rotation2d.fromDegrees(45), volts.baseUnitMagnitude());
                  modules[1].runCharacterizationAmps(
                      Rotation2d.fromDegrees(-45), volts.baseUnitMagnitude());
                  modules[2].runCharacterizationAmps(
                      Rotation2d.fromDegrees(45), volts.baseUnitMagnitude());
                  modules[3].runCharacterizationAmps(
                      Rotation2d.fromDegrees(45), volts.baseUnitMagnitude());
                },
                null,
                this));

    linearCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volt.of(4),
                null,
                state -> SignalLogger.writeString("SysidAngularState", state.toString())),
            new Mechanism(
                volts ->
                    Arrays.stream(modules)
                        .forEach(
                            m ->
                                m.runCharacterizationAmps(
                                    Rotation2d.fromDegrees(0), volts.baseUnitMagnitude())),
                null,
                this));

    pivotCharacterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volt.of(4),
                null,
                state -> SignalLogger.writeString("SysidAngularState", state.toString())),
            new Mechanism(
                volts ->
                    Arrays.stream(modules)
                        .forEach(m -> m.runCharacterizationAmps(volts.baseUnitMagnitude(), 0.0)),
                null,
                this));

    if (FLAGS.TUNING_MODE) {
      SmartDashboard.putData(
          "transQuasistaticForward", linearCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "transQuasistaticReverse", linearCharacterization.quasistatic(Direction.kReverse));
      SmartDashboard.putData(
          "transDynamicForward", linearCharacterization.dynamic(Direction.kForward));
      SmartDashboard.putData(
          "transDynamicReverse", linearCharacterization.dynamic(Direction.kReverse));

      SmartDashboard.putData(
          "AngularQuasistaticForward", angularCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "AngularQuasistaticReverse", angularCharacterization.quasistatic(Direction.kReverse));
      SmartDashboard.putData(
          "AngularDynamicForward", angularCharacterization.dynamic(Direction.kForward));
      SmartDashboard.putData(
          "AngularDynamicReverse", angularCharacterization.dynamic(Direction.kReverse));

      SmartDashboard.putData(
          "PivotQuasistaticForward", pivotCharacterization.quasistatic(Direction.kForward));
      SmartDashboard.putData(
          "PivotQuasistaticReverse", pivotCharacterization.quasistatic(Direction.kReverse));
      SmartDashboard.putData(
          "PivotDynamicForward", pivotCharacterization.dynamic(Direction.kForward));
      SmartDashboard.putData(
          "PivotDynamicReverse", pivotCharacterization.dynamic(Direction.kReverse));
    }
  }

  @Override
  public void periodic() {
    odo.update(gyro.getRotation2d(), getSwerveModulePoses());
    f2d.setRobotPose(getPose());
    swervePublisher.set(getSwerveModuleStates());

    SmartDashboard.putNumber("Speed MPS", getSpeedMPS());

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    simulation.get().periodic();
  }

  /**
   * Drive based on input streams.
   *
   * @param xVelocity X velocity
   * @param yVelocity Y velocity stream
   * @param desiredHeading Desired heading stream
   */
  public Command drive(
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      Rotation2d desiredHeading,
      DoubleSupplier speedSupplier) {
    return drive(
        xVelocity,
        yVelocity,
        () -> angularFeedback.calculate(getHeading().getRadians(), desiredHeading.getRadians()),
        speedSupplier);
  }

  /**
   * Drive based on input streams
   *
   * @param xVelocity X velocity stream
   * @param yVelocity Y velocity stream
   * @param omegaVelocity omega velocity stream
   */
  public Command drive(
      DoubleSupplier xVelocity,
      DoubleSupplier yVelocity,
      DoubleSupplier omegaVelocity,
      DoubleSupplier speedSupplier) {
    return this.run(
        () ->
            runVelocities(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    xVelocity.getAsDouble() * MAX_LINEAR_SPEED * speedSupplier.getAsDouble(),
                    yVelocity.getAsDouble() * MAX_LINEAR_SPEED * speedSupplier.getAsDouble(),
                    omegaVelocity.getAsDouble() * MAX_ANGULAR_SPEED,
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? getHeading()
                        : getHeading().minus(Rotation2d.fromDegrees(180)))));
  }

  /**
   * Drives to scoring target
   *
   * @param targetSupplier scoring target supplier
   */
  public Command driveToPoseCmmd(Supplier<AlignTargets> targetSupplier) {
    return driveToPoseCmmd(targetSupplier.get().pose);
  }

  /**
   * Drives to desired pose using feedback controllers
   *
   * @param desiredPose Desired pose
   */
  public Command driveToPoseCmmd(Pose2d desiredPose) {
    return this.run(
        () -> {
          Pose2d currentPose = getPose();
          var velocities =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearFeedback.calculate(currentPose.getX(), desiredPose.getX()),
                  linearYFeedback.calculate(currentPose.getY(), desiredPose.getY()),
                  angularFeedback.calculate(
                      currentPose.getRotation().getRadians(),
                      desiredPose.getRotation().getRadians()),
                  getHeading());
          runVelocities(velocities);
        });
  }

  /**
   * Run desired chassis velocities
   *
   * @param velocities desired velocities
   */
  public void runVelocities(ChassisSpeeds velocities) {
    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(velocities, 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_LINEAR_SPEED);
    setpoint.set(states);
    for (int i = 0; i < states.length; i++) {
      modules[i].runModuleSetpoint(states[i]);
    }
  }

  /**
   * Update odometry with vision estimates
   *
   * @param poses estimates
   */
  public void updateVisionEstimates(PoseEstimate... poses) {
    for (int i = 0; i < poses.length; i++) {
      odo.addVisionMeasurement(
          poses[i].pose().estimatedPose.toPose2d(),
          poses[i].pose().timestampSeconds,
          poses[i].stdevs());
      // f2d.getObject("estimated pose " + i).setPose(poses[i].pose().estimatedPose.toPose2d());
    }
  }

  /**
   * Resets odometry to pose
   *
   * @param pose odometry reset pose
   */
  public void resetOdo(Pose2d pose) {
    odo.resetPose(pose);
    simulation.ifPresent(
        s -> {
          s.setSimulationWorldPose(pose);
          s.getDriveTrainSimulation().setRobotSpeeds(new ChassisSpeeds());
        });
  }

  /** Stops all motor input for dt */
  public void stop() {
    for (Module module : modules) {
      module.stopModule();
    }
  }

  /**
   * @return current odometry estimated pose
   */
  public Pose2d getPose() {
    return odo.getEstimatedPosition();
  }

  /**
   * @return current odometry estimated heading
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * @return current chassis speed in Meters/Second
   */
  public double getSpeedMPS() {
    return Math.hypot(
        getCurrentVelocities().vxMetersPerSecond, getCurrentVelocities().vyMetersPerSecond);
  }

  /**
   * @return current chassis velocities
   */
  public ChassisSpeeds getCurrentVelocities() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * @return current swerve states
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }

  /**
   * @return current swerve poses on field
   */
  public SwerveModulePosition[] getSwerveModulePoses() {
    SwerveModulePosition[] poses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      poses[i] = modules[i].getModulePose();
    }
    return poses;
  }

  // ! This might be stupid as hell lol
  public static double allianceAddition(double value) {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
        ? Units.inchesToMeters(689.938458) - value
        : value;
  }

  /**
   * Calculates reef align pose based on two values
   *
   * @param a an integer between 0-5 that represents reef side (Goes ccw from 0 which is the side
   *     facing the barge)
   * @param b either -1 or 1 (-1 is right when facing wall)
   * @return {@link Pose2d} of desired alignment pose if values are valid; else, returns pose of a=0
   *     & b=0
   */
  public static Pose2d calculateReefPose(int a, int b) {
    if (a > 5 || a < 0) {
      a = 0;
    }

    if (!(b >= -1 || b <= 1)) {
      b = 0;
    }

    double x = ((50.49 * Math.cos(((a * Math.PI) / 3) + (0.137 * b))) + 177.25) / 39.37;
    double y = ((50.49 * Math.sin(((a * Math.PI) / 3) + (0.137 * b))) + 158.50) / 39.37;
    double theta = (a * Math.PI) / 3;

    return new Pose2d(x, y, Rotation2d.fromRadians(theta));
  }

  /** static enum containing align targets */
  public static enum AlignTargets {
    A(calculateReefPose(3, -1)),
    B(calculateReefPose(3, 1)),
    C(calculateReefPose(4, -1)),
    D(calculateReefPose(4, 1)),
    E(calculateReefPose(5, -1)),
    F(calculateReefPose(5, 1)),
    G(calculateReefPose(0, -1)),
    H(calculateReefPose(0, 1)),
    I(calculateReefPose(1, -1)),
    J(calculateReefPose(1, 1)),
    K(calculateReefPose(2, -1)),
    L(calculateReefPose(2, 1));

    private Pose2d pose;

    private AlignTargets(Pose2d pose) {
      this.pose = pose;
    }
  }

  // /** A cursed static enum containing all score target poses */
  // public static enum ScoreTargets {
  //   A(
  //       new Pose2d(
  //           allianceAddition(3.23172676579), 4.20105441609,
  // Rotation2d.fromRadians(3.14159265359))),
  //   B(
  //       new Pose2d(
  //           allianceAddition(3.23172676579), 3.85076168754,
  // Rotation2d.fromRadians(3.14159265359))),
  //   C(
  //       new Pose2d(
  //           allianceAddition(3.71526168421), 3.0132546416,
  // Rotation2d.fromRadians(4.18879020479))),
  //   D(
  //       new Pose2d(
  //           allianceAddition(4.0186240859), 2.83810827733,
  // Rotation2d.fromRadians(4.18879020479))),
  //   E(
  //       new Pose2d(
  //           allianceAddition(4.98569392274), 2.83810827733,
  // Rotation2d.fromRadians(5.23598775598))),
  //   F(
  //       new Pose2d(
  //           allianceAddition(5.28905632442), 3.0132546416,
  // Rotation2d.fromRadians(5.23598775598))),
  //   G(new Pose2d(allianceAddition(5.77259124284), 3.85076168754, Rotation2d.fromRadians(0))),
  //   H(new Pose2d(allianceAddition(5.77259124284), 4.20105441609, Rotation2d.fromRadians(0))),
  //   I(
  //       new Pose2d(
  //           allianceAddition(5.28905632442), 5.03856146203,
  // Rotation2d.fromRadians(1.0471975512))),
  //   J(
  //       new Pose2d(
  //           allianceAddition(4.98569392274), 5.2137078263,
  // Rotation2d.fromRadians(1.0471975512))),
  //   K(
  //       new Pose2d(
  //           allianceAddition(4.0186240859), 5.2137078263,
  // Rotation2d.fromRadians(2.09439510239))),
  //   L(
  //       new Pose2d(
  //           allianceAddition(3.71526168421), 5.03856146203,
  // Rotation2d.fromRadians(2.09439510239)));

  //   private Pose2d pose;

  //   private ScoreTargets(Pose2d pose) {
  //     this.pose = pose;
  //   }
  // }
}
