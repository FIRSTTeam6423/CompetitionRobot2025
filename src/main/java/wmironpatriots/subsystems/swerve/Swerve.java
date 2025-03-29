// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lib.LoggedSubsystem;
import lib.Tracer;
import wmironpatriots.Constants;
import wmironpatriots.subsystems.swerve.SwerveConstants.ModuleConfig;
import wmironpatriots.subsystems.swerve.gyro.Gyro;
import wmironpatriots.subsystems.swerve.gyro.GyroIOComp;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;

public class Swerve implements LoggedSubsystem {
  private final Gyro gyro;
  private final Module[] modules;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;

  private final Field2d f2d;

  private final StructArrayPublisher<SwerveModuleState> swervePublisher, swerveSetpointPublisher;

  public Swerve() {
    // Setup hardware
    gyro = new GyroIOComp();
    ModuleConfig[] moduleConfigs = SwerveConstants.MODULE_CONFIGS;
    modules = new ModuleIOComp[moduleConfigs.length];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new ModuleIOComp(moduleConfigs[i]);
    }

    // Init helpers
    kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_LOCS);
    odometry =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[modules.length],
            new Pose2d(new Translation2d(), Rotation2d.fromRadians(0)));

    // Setup dashboard
    f2d = new Field2d();
    SmartDashboard.putData(f2d);

    swervePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
            .publish();
    swerveSetpointPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveSetpointStates", SwerveModuleState.struct)
            .publish();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getModulePoses());

    for (Module module : modules) {
      Tracer.traceFunc("module" + module.index + "Periodic", module::periodic);
    }
    Tracer.traceFunc("gyroPeriodic", gyro::periodic);
    if (DriverStation.isDisabled()) stop();

    f2d.setRobotPose(getPose());
    swervePublisher.set(getModuleStates());
    SmartDashboard.putNumber("Speed MPS", getSpeedMPS());
  }

  /**
   * Drive based on input streams
   *
   * @param xVelocitySupplier X velocity stream
   * @param yVelocitySupplier Y velocity stream
   * @param omegaVelocitySupplier Omega velocity stream
   */
  public Command driveCmd(
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier omegaVelocitySupplier) {
    return driveCmd(xVelocitySupplier, yVelocitySupplier, omegaVelocitySupplier, () -> 1.0);
  }

  /**
   * Drive based on input streams
   *
   * @param xVelocitySupplier X velocity stream
   * @param yVelocitySupplier Y velocity stream
   * @param omegaVelocitySupplier Omega velocity stream
   * @param speedSupplier Speed modifier
   */
  public Command driveCmd(
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier omegaVelocitySupplier,
      DoubleSupplier speedSupplier) {
    var maxLinearSpeed = SwerveConstants.MAX_LINEAR_VELOCITY.in(MetersPerSecond);
    var maxAngularSpeed = SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
    return this.run(
        () ->
            runVelocities(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocitySupplier.getAsDouble() * maxLinearSpeed * speedSupplier.getAsDouble(),
                    yVelocitySupplier.getAsDouble() * maxLinearSpeed * speedSupplier.getAsDouble(),
                    omegaVelocitySupplier.getAsDouble() * maxAngularSpeed,
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? getHeading()
                        : getHeading().minus(Rotation2d.fromDegrees(180)))));
  }

  /**
   * Run desired chassis velocities
   *
   * @param velocities Desired velocities as {@link ChassisSpeeds}
   */
  public void runVelocities(ChassisSpeeds velocities) {
    velocities = ChassisSpeeds.discretize(velocities, Constants.TICK_SPEED.in(Seconds));

    // Convert to clamped SwerveModuleStates
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(velocities);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_LINEAR_VELOCITY);

    // Request modules to run states and get optimized states
    for (int i = 0; i < desiredStates.length; i++) {
      desiredStates[i] = modules[i].runModuleSetpoint(desiredStates[i]);
    }
    // Update setpoint publisher
    swerveSetpointPublisher.set(desiredStates);
  }

  /** Stops all swerve motors */
  public void stop() {
    for (Module module : modules) {
      module.stopMotors();
    }
  }

  /**
   * @return chassis speed in Meters/Second
   */
  public double getSpeedMPS() {
    var velocities = getVelocities();
    return Math.hypot(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond);
  }

  /**
   * @return states of each module in a {@link SwerveModuleState} array
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getModuleState();
    }
    return moduleStates;
  }

  /**
   * @return field poses of each module in a {@link SwerveModulePosition} array
   */
  public SwerveModulePosition[] getModulePoses() {
    SwerveModulePosition[] modulePoses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePoses.length; i++) {
      modulePoses[i] = modules[i].getModulePose();
    }

    return modulePoses;
  }

  /**
   * @return Robot centric velocities as {@link ChassisSpeeds}
   */
  public ChassisSpeeds getVelocities() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * @return chassis field pose as {@link Pose2d}
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * @return chassis heading from last reset as {@link Rotation2d}
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }
}
