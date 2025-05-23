// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lib.swerve.ChassisVelocity;
import wmironpatriots.Constants;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.Swerve.gyro.GyroHardware;
import wmironpatriots.subsystems.Swerve.gyro.GyroHardwareComp;
import wmironpatriots.subsystems.Swerve.gyro.GyroHardwareNone;
import wmironpatriots.subsystems.Swerve.module.Module;
import wmironpatriots.subsystems.Swerve.module.ModuleHardwareComp;

public class Swerve implements Subsystem {
  public static Swerve create() {
    var moduleConfigs = SwerveConstants.MODULE_CONFIGS;
    var modules = new Module[moduleConfigs.length];

    if (Robot.isReal()) {
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareComp(moduleConfigs[i]));
      }

      return new Swerve(new GyroHardwareComp(), modules);
    } else {
      // ! PLACEHOLDER FOR SIM HARDWARE INIT
      for (int i = 0; i < modules.length; i++) {
        modules[i] = new Module(new ModuleHardwareComp(moduleConfigs[i]));
      }

      return new Swerve(new GyroHardwareNone(), modules);
    }
  }

  private final Module[] modules;
  private final GyroHardware gyro;

  private Rotation2d simulatedHeading = Rotation2d.kZero;

  private final SwerveDriveKinematics kinematics = SwerveConstants.KINEMATICS;
  private final SwerveDrivePoseEstimator odometry;

  public Swerve(GyroHardware gyro, Module... modules) {
    this.modules = modules;
    this.gyro = gyro;

    odometry =
        new SwerveDrivePoseEstimator(
            kinematics, getGyroRotation2d(), getSwerveModulePositions(), new Pose2d());

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    odometry.update(getGyroRotation2d(), getSwerveModulePositions());

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  @Override
  public void simulationPeriodic() {
    var angularRate = getVelocity().getRobotRelative().omegaRadiansPerSecond;
    simulatedHeading =
        simulatedHeading.rotateBy(
            Rotation2d.fromRadians(
                !Double.isNaN(angularRate) ? angularRate * Constants.LOOPTIME.in(Seconds) : 0));
  }

  /**
   * Converts a robot centric velocity into {@link SwerveModuleState} setpoint for each module
   *
   * @param velocity {@link ChassisVelocity} representing desired robot centric velocity setpoint
   * @return {@link Command}
   */
  public Command setChassisVelocity(ChassisVelocity velocity) {
    return run(
        () -> {
          // https://github.com/wpilibsuite/allwpilib/issues/7332
          var states = kinematics.toSwerveModuleStates(velocity.getRobotRelative());
          SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);

          var speeds = kinematics.toChassisSpeeds(states);
          speeds = ChassisSpeeds.discretize(speeds, Constants.LOOPTIME.in(Seconds));

          states = kinematics.toSwerveModuleStates(speeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_LINEAR_SPEED);

          for (int i = 0; i < modules.length; i++) {
            modules[i].setSetpoints(states[i]);
          }
        });
  }

  /**
   * Stops all swerve motors
   *
   * @return {@link Command}
   */
  public Command stop() {
    return runOnce(
            () -> {
              for (Module module : modules) {
                module.stop();
              }
            })
        .withName("Stop");
  }

  /**
   * @return {@link Rotation2d} representing gyro heading since boot
   */
  public Rotation2d getGyroRotation2d() {
    return Robot.isReal() ? gyro.getRotation3d().toRotation2d() : simulatedHeading;
  }

  /**
   * @return {@link Pose2d} representing the odometry estimated position
   */
  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  /**
   * @return {@link Rotation2d} representing the chassis's yaw from last heading reset
   */
  public Rotation2d getHeadingRotation2d() {
    return getPose2d().getRotation();
  }

  /**
   * @return {@link ChassisVelocity} representing the measured chassis velocity derived from module
   *     states
   */
  public ChassisVelocity getVelocity() {
    var moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = modules[i].getSwerveModuleState();
    }

    return ChassisVelocity.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates));
  }

  /**
   * @return {@link SwerveModuleState} array representing the measured speeds and angle of modules
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    var moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = modules[i].getSwerveModuleState();
    }

    return moduleStates;
  }

  /**
   * @return {@link SwerveModulePosition} array representing the measured drive distance and angle
   *     of modules
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    var modulePoses = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modulePoses.length; i++) {
      modulePoses[i] = modules[i].getSwerveModulePosition();
    }

    return modulePoses;
  }
}
