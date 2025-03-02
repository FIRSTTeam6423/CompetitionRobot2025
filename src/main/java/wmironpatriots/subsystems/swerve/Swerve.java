// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static wmironpatriots.Constants.CANIVORE;
import static wmironpatriots.Constants.kTickSpeed;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;
import wmironpatriots.subsystems.swerve.module.ModuleIOSim;
import wmironpatriots.util.swerveUtil.SwerveConfig;

public class Swerve extends SubsystemBase {
  private final SwerveConfig config;

  private final Module[] modules;
  private final Pigeon2 pigeon;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator odo;

  private Rotation2d simHeading;

  private final Field2d f2d;

  private final StructArrayPublisher<SwerveModuleState> statesPublisher;

  public Swerve(SwerveConfig config) {
    System.out.println("Init swerve");
    // Create modules
    if (Robot.isReal()) {
      var moduleConfigs = config.getModuleConfigs();
      modules = new Module[moduleConfigs.length];
      Arrays.stream(moduleConfigs).forEach((c) -> modules[c.index - 1] = new ModuleIOComp(c));
    } else {
      var moduleConfigs = config.getModuleConfigs();
      modules = new Module[moduleConfigs.length];
      Arrays.stream(moduleConfigs).forEach((c) -> modules[c.index - 1] = new ModuleIOSim(c));
    }
    pigeon = new Pigeon2(0, CANIVORE);
    simHeading = new Rotation2d();

    // Create math objects
    kinematics = new SwerveDriveKinematics(config.getModuleLocs());
    odo = new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePoses(), new Pose2d());

    f2d = new Field2d();

    statesPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct)
            .publish();

    this.config = config;
  }

  @Override
  public void periodic() {
    // Update gyro and all modules
    Arrays.stream(modules).forEach((m) -> m.periodic());

    // Odo update
    updateOdometry();
    f2d.setRobotPose(getPose());
    SmartDashboard.putData(f2d);

    // Log swerve data
    statesPublisher.set(getModuleStates());
    // Logger.recordOutput("Swerve/ActualOutput", getVelocitiesRobotRelative());
    // Logger.recordOutput("Swerve/ActualStates", getModuleStates());

    // Stop module input when driverstation is disabled
    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    double clamped =
        MathUtil.clamp(
            getVelocitiesRobotRelative().omegaRadiansPerSecond,
            -config.getMaxAngularSpeedRadsPerSec() / 10000,
            config.getMaxAngularSpeedRadsPerSec() / 10000);
    simHeading = simHeading.rotateBy(Rotation2d.fromRadians(clamped));
  }

  /**
   * Takes Input Streams, converts them into velocities, and converts those velocties to field
   * relative speeds
   *
   * @param xSupplier + is towards alliance wall
   * @param ySupplier + is left of alliance side
   * @param omegaSupplier + is counterclockwise
   */
  public Command teleopSwerveCommmand(
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

  /**
   * Runs swerve at desired robot relative velocities
   *
   * @param velocity desired velocities
   * @param openloopEnabled enable or disable open loop voltage control
   */
  public void runVelocities(ChassisSpeeds velocity, boolean openloopEnabled) {
    velocity = ChassisSpeeds.discretize(velocity, kTickSpeed);

    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(velocity);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, config.getMaxLinearSpeedMetersPerSec());

    // Logger.recordOutput("Swerve/desiredVelocity", velocity);
    // Logger.recordOutput("Swerve/desiredSetpoints", desiredStates);

    for (int i = 0; i < desiredStates.length; i++) {
      if (openloopEnabled) {
        ChassisSpeeds currentVelocities = getVelocitiesRobotRelative();
        boolean focEnabled =
            Math.sqrt(
                    Math.pow(currentVelocities.vxMetersPerSecond, 2)
                        + Math.pow(
                            currentVelocities.vyMetersPerSecond,
                            2)) // converts linear velocity components to linear velocity
                < 1 * config.getMaxLinearSpeedMetersPerSec();

        // Converts desired motor velocity into input voltage
        // OmegaRadsPerSec/(KvRadsPerVolt)
        double driveVoltage =
            desiredStates[i].speedMetersPerSecond / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt;
        modules[i].runSetpointOpenloop(driveVoltage, desiredStates[i].angle, focEnabled);
      } else {
        modules[i].runSetpoint(desiredStates[i]);
      }
    }
  }

  /** update swerve pose estimator */
  public void updateOdometry() {
    odo.update(getHeading(), getModulePoses());
  }

  // GETTERS
  /** Returns an array of module field positions */
  public SwerveModulePosition[] getModulePoses() {
    return Arrays.stream(modules).map(Module::getModulePose).toArray(SwerveModulePosition[]::new);
  }

  /** Returns an array of module states */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getModuleState).toArray(SwerveModuleState[]::new);
  }

  /** Gets current robot heading */
  public Rotation2d getHeading() {
    return pigeon.getRotation2d();
  }

  /** Gets current robot field pose */
  public Pose2d getPose() {
    return odo.getEstimatedPosition();
  }

  /** Gets current robot velocity (robot relative) */
  public ChassisSpeeds getVelocitiesRobotRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), getHeading());
  }

  /** Get swerve configs */
  public SwerveConfig getConfig() {
    return config;
  }
}
