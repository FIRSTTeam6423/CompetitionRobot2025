// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve;

import static org.frc6423.frc2025.Constants.kTickSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.frc6423.frc2025.subsystems.swerve.module.Module;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveConfig m_config;

  private final Module[] m_modules;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;

  private Rotation2d m_simRotation = new Rotation2d();

  private final Field2d m_f2d;

  public SwerveSubsystem(SwerveConfig config) {
    // Create modules
    var moduleConfigs = config.getModuleConfigs();
    m_modules = new Module[moduleConfigs.length];
    Arrays.stream(moduleConfigs).forEach((c) -> m_modules[c.kIndex - 1] = new Module(c));

    // Create math objects
    m_kinematics = new SwerveDriveKinematics(config.getModuleLocs());
    m_poseEstimator =
        new SwerveDrivePoseEstimator(m_kinematics, getHeading(), getModulePoses(), new Pose2d());

    m_f2d = new Field2d();

    m_config = config;
  }

  @Override
  public void periodic() {
    Arrays.stream(m_modules).forEach((m) -> m.updateInputs());

    Logger.recordOutput("Swerve/ActualOutput", getVelocitiesRobotRelative());
    Logger.recordOutput("Swerve/ActualStates", getModuleStates());

    updateOdometry();
    m_f2d.setRobotPose(getPose());
    SmartDashboard.putData(m_f2d);

    // Stop module input when driverstation is disabled
    if (DriverStation.isDisabled()) {
      for (Module module : m_modules) {
        module.stop();
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    double clamped =
        MathUtil.clamp(
            m_kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond,
            -m_config.getMaxAngularSpeedRadsPerSec(),
            m_config.getMaxAngularSpeedRadsPerSec());
    m_simRotation.rotateBy(Rotation2d.fromRadians(clamped));
  }

  /**
   * Takes axis inputs, scales them to velocties, and converts those velocties to field relative
   * speeds
   *
   * @param xSupplier + is towards alliance wall
   * @param ySupplier + is left of alliance side
   * @param omegaSupplier + is counterclockwise
   */
  public Command teleopSwerveCommmand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return this.run(
        () -> {
          double maxSpeed = m_config.getMaxLinearSpeedMetersPerSec();

          var fieldRelativeVelocities =
              new ChassisSpeeds(
                  xSupplier.getAsDouble() * maxSpeed,
                  ySupplier.getAsDouble() * maxSpeed,
                  omegaSupplier.getAsDouble() * maxSpeed);
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

    SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(velocity);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_config.getMaxLinearSpeedMetersPerSec());

    Logger.recordOutput("Swerve/desiredVelocity", velocity);
    Logger.recordOutput("Swerve/desiredSetpoints", desiredStates);

    for (int i = 0; i < desiredStates.length; i++) {
      if (openloopEnabled) {
        ChassisSpeeds currentVelocities = getVelocitiesRobotRelative();
        boolean focEnabled =
            Math.sqrt(
                    Math.pow(currentVelocities.vxMetersPerSecond, 2)
                        + Math.pow(
                            currentVelocities.vyMetersPerSecond,
                            2)) // converts linear velocity components to linear velocity
                < 1 * m_config.getMaxLinearSpeedMetersPerSec();

        // Converts desired motor velocity into input voltage
        // OmegaRadsPerSec/(KvRadsPerVolt)
        double driveVoltage =
            desiredStates[i].speedMetersPerSecond / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt;
        m_modules[i].runSetpointOpenloop(driveVoltage, desiredStates[i].angle, focEnabled);
      } else {
        m_modules[i].runSetpoint(desiredStates[i]);
      }
    }
  }

  /** update swerve pose estimator */
  public void updateOdometry() {
    m_poseEstimator.update(getHeading(), getModulePoses());
  }

  /** Gets current robot velocity (robot relative) */
  public ChassisSpeeds getVelocitiesRobotRelative() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Gets current robot heading */
  public Rotation2d getHeading() {
    return m_simRotation;
  }

  /** Gets current robot field pose */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns an array of module field positions */
  public SwerveModulePosition[] getModulePoses() {
    return Arrays.stream(m_modules).map(Module::getModulePose).toArray(SwerveModulePosition[]::new);
  }

  /** Returns an array of module states */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(m_modules).map(Module::getModuleState).toArray(SwerveModuleState[]::new);
  }
}
