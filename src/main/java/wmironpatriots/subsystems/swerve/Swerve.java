// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModule;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.LoggedSubsystem;
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

  private static Swerve instance;

  /**
   * @return swerve subsystem instance (creates instance if null)
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  private Swerve() {
    // Setup hardware
    gyro = new GyroIOComp();
    ModuleConfig[] moduleConfigs = SwerveConstants.MODULE_CONFIGS;
    modules = new ModuleIOComp[moduleConfigs.length];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new ModuleIOComp(moduleConfigs[i]);
    }

    // Init helpers
    kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_LOCS);
    odometry = new SwerveDrivePoseEstimator(
      kinematics, 
      gyro.getRotation2d(), 
      new SwerveModulePosition[modules.length], 
      new Pose2d(new Translation2d(), 
      Rotation2d.fromRadians(0)));

    // Setup dashboard
    f2d = new Field2d();
    SmartDashboard.putData(f2d);
    
    swervePublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
      .publish();
    swerveSetpointPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SwerveSetpointStates", SwerveModuleState.struct)
      .publish();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getModulePoses());

    for (Module module : modules) {
      module.periodic();
    }
    if (DriverStation.isDisabled()) stop();
    
    f2d.setRobotPose(getPose());
    swervePublisher.set(getModuleStates());
    SmartDashboard.putNumber("Speed MPS", getSpeedMPS());
  }

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
