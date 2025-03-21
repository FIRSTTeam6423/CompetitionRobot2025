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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import java.util.function.DoubleSupplier;
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

  private final SwerveDrivePoseEstimator odo;

  private final PIDController angularFeedback;
  private final ProfiledPIDController xLinearFeedback, yLinearFeedback;
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

  private AlignTargets alignTarget = AlignTargets.A;

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
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(0.0, new Rotation2d()),
              new SwerveModulePosition(0.0, new Rotation2d()),
              new SwerveModulePosition(0.0, new Rotation2d()),
              new SwerveModulePosition(0.0, new Rotation2d()),
            },
            new Pose2d(new Translation2d(), Rotation2d.fromRadians(0.0)));

    f2d = new Field2d();
    SmartDashboard.putData(f2d);

    angularFeedback = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    angularFeedback.enableContinuousInput(0, 2 * Math.PI);
    angularFeedback.setTolerance(0.0523599);
    xLinearFeedback =
        new ProfiledPIDController(
            LINEAR_P, LINEAR_I, LINEAR_D, new TrapezoidProfile.Constraints(MAX_LINEAR_SPEED, 20));
    xLinearFeedback.setTolerance(0.05);
    yLinearFeedback =
        new ProfiledPIDController(
            LINEAR_P, LINEAR_I, LINEAR_D, new TrapezoidProfile.Constraints(MAX_LINEAR_SPEED, 20));
    yLinearFeedback.setTolerance(0.05);

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
    f2d.getObject("target").setPose(alignTarget.pose);

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

  public Command setAlignTarget(AlignTargets target) {
    return this.runOnce(() -> this.alignTarget = target);
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
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity.getAsDouble() * MAX_LINEAR_SPEED * speedSupplier.getAsDouble(),
                    yVelocity.getAsDouble() * MAX_LINEAR_SPEED * speedSupplier.getAsDouble(),
                    omegaVelocity.getAsDouble() * MAX_ANGULAR_SPEED,
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? getHeading()
                        : getHeading().minus(Rotation2d.fromDegrees(180)))));
  }

  /**
   * Drives to desired pose using feedback controllers
   *
   * @param desiredPose Desired pose
   */
  public Command driveToPoseCmmd(DoubleSupplier omegaVelocity) {
    return this.run(
        () -> {
          var currentPose = getPose();
          var desiredPose = alignTarget.pose;
          var velocities =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xLinearFeedback.calculate(currentPose.getX(), desiredPose.getX()),
                  yLinearFeedback.calculate(currentPose.getY(), desiredPose.getY()),
                  omegaVelocity.getAsDouble() * MAX_ANGULAR_SPEED,
                  // angularFeedback.calculate(
                  //     currentPose.getRotation().getRadians(),
                  //     desiredPose.getRotation().getRadians()),
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
    odo.resetPosition(gyro.getRotation2d(), getSwerveModulePoses(), pose);
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
    for (int i = 0; i < 4; i++) {
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

    if (b != -1 && b != 1) {
      b = 1;
    }
    double angle = 0.137 - 1 * 0.0175;
    // - 4 * 0.0175;
    double x = ((50.49 * Math.cos(((a * Math.PI) / 3) + (angle * b))) + 177.25) / 39.37;
    double y = ((50.49 * Math.sin(((a * Math.PI) / 3) + (angle * b))) + 158.50) / 39.37;
    double theta = (a * Math.PI) / 3;

    x =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? (Units.inchesToMeters(689.751) - x)
            : x;
    y =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? (Units.inchesToMeters(317.5) - y)
            : y;
    theta =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? theta + Math.PI : theta;

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
}
