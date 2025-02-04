// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import org.frc6423.frc2025.Constants.KDriveConstants.DriveControlMode;
import org.frc6423.frc2025.Robot;
import org.frc6423.frc2025.subsystems.swerve.gyro.GyroIO;
import org.frc6423.frc2025.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import org.frc6423.frc2025.subsystems.swerve.gyro.GyroIONavX;
import org.frc6423.frc2025.subsystems.swerve.module.Module;
import org.frc6423.frc2025.subsystems.swerve.module.ModuleIOSpark;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig.moduleType;
import org.frc6423.frc2025.util.swerveUtil.SwerveConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  private final Module[] m_modules;

  private final GyroIO m_gryo;
  private final GyroIOInputsAutoLogged m_gyroInputs;
  private Rotation2d m_simulationHeading;

  private final SwerveDriveKinematics m_swerveKinematics;
  private final SwerveDrivePoseEstimator m_swervePoseEstimator;
  private ChassisSpeeds m_setpointVelocity;

  private final PIDController m_rotationalVelocityFeedback;

  public SwerveSubsystem(SwerveConfig config) {
    m_gryo = config.kGyroID == 100 ? new GyroIONavX() : new GyroIONavX(); // ! Add pigeon gyro
    m_gyroInputs = new GyroIOInputsAutoLogged();
    m_simulationHeading = new Rotation2d();
    m_modules = new Module[config.kModuleConfigs.length];

    Arrays.stream(config.kModuleConfigs)
        .forEach(
            (moduleConfig) -> {
              int index = moduleConfig.kIndex - 1;

              m_modules[index] =
                  new Module(
                      (moduleConfig.kModuletype == moduleType.SPARKMAX)
                          ? new ModuleIOSpark(moduleConfig)
                          : new ModuleIOSpark(moduleConfig), // ! Add talonfx module
                      index);
            });

    m_swerveKinematics = new SwerveDriveKinematics(config.kModuleLocs);
    m_swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            m_swerveKinematics, new Rotation2d(), getModulePoses(), new Pose2d());

    m_rotationalVelocityFeedback = new PIDController(1, 0, 0); // !
  }

  @Override
  public void periodic() {
    // gyro update
    if (Robot.isReal()) m_gryo.updateInputs(m_gyroInputs);
    else
      m_simulationHeading.rotateBy(
          Rotation2d.fromRadians(getSetpointVelocity().omegaRadiansPerSecond));

    Arrays.stream(m_modules).forEach(Module::periodic); // Update each module

    Logger.processInputs("Swerve/Gyro", m_gyroInputs);
    Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    Logger.recordOutput("Swerve/DesiredVel", m_setpointVelocity);
    Logger.recordOutput("Swerve/Velocity", getVelocity());
  }

  /**
   * Drives Robot based on input velocities
   *
   * @param velXMetersPerSec X translation velocity supplier
   * @param velYMetersPerSec Y translation velocity supplier
   * @param velOmegaRadsPerSec Omega rotation velocity supplier
   * @return {@link Command}
   */
  public Command drive(
      double velXMetersPerSec, double velYMetersPerSec, double velOmegaRadsPerSec) {
    return this.run(
        () ->
            setSetpointVelocities(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    velXMetersPerSec,
                    velYMetersPerSec,
                    velOmegaRadsPerSec,
                    DriverStation.getAlliance().get() == Alliance.Blue
                        ? getHeading()
                        : getHeading().minus(Rotation2d.fromRotations(0.5))),
                DriveControlMode.OPENLOOP));
  }

  /**
   * Drives Robot based on input velocities and desired heading
   *
   * @param velXMetersPerSec X translation velocity supplier
   * @param velYMetersPerSec Y translation velocity supplier
   * @param desiredHeading Desired robot orientation
   * @return {@link Command}
   */
  public Command drive(
      double velXMetersPerSec, double velYMetersPerSec, Rotation2d desiredHeading) {
    return this.run(
        () ->
            setSetpointVelocities(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    velXMetersPerSec,
                    velYMetersPerSec,
                    m_rotationalVelocityFeedback.calculate(
                        getHeading().getRadians(), desiredHeading.getRadians()),
                    getHeading()),
                DriveControlMode.OPENLOOP));
  }

  /** Set setpoint velocities */
  public void setSetpointVelocities(ChassisSpeeds desiredSpeeds, DriveControlMode controlMode) {
    m_setpointVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            desiredSpeeds, getHeading()); // converts to field relative speeds
    setModuleStates(
        m_swerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(m_setpointVelocity, 0.02)),
        controlMode);
  }

  /** Set swerve module setpoints */
  public void setModuleStates(SwerveModuleState[] states, DriveControlMode mode) {
    Arrays.stream(m_modules).forEach((m) -> m.setDesiredSate(states[m.getIndex()], mode));
  }

  /** Returns desired chassis velocities */
  public ChassisSpeeds getSetpointVelocity() {
    return new ChassisSpeeds();
  }

  /** Get current velocity */
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        m_swerveKinematics.toChassisSpeeds(getModuleStates()), getHeading());
  }

  /** Get current heading */
  public Rotation2d getHeading() {
    return Robot.isReal() ? m_gyroInputs.orientation.toRotation2d() : m_simulationHeading;
  }

  /** Get Swerve Module Positions */
  public SwerveModulePosition[] getModulePoses() {
    return Arrays.stream(m_modules)
        .map(Module::getCurrentPose)
        .toArray(SwerveModulePosition[]::new);
  }

  /** Get Swerve Module States */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(m_modules).map(Module::getCurrentState).toArray(SwerveModuleState[]::new);
  }
}
