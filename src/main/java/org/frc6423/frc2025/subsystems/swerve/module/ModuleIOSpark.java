// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

public class ModuleIOSpark implements ModuleIO {

  private final SparkMax m_pivotM, m_driveM;
  private SparkMaxConfig m_pivotConf, m_driveConf;
  private final RelativeEncoder m_pivotEncoder, m_driveEncoder;
  private final AbsoluteEncoder m_pivotABSEncoder;

  private final SparkClosedLoopController m_pivotFeedback, m_driveFeedback;

  public ModuleIOSpark(ModuleConfig config) {
    // Pivot init
    m_pivotM = new SparkMax(config.kPivotID, MotorType.kBrushless);
    m_pivotEncoder = m_pivotM.getEncoder();
    m_pivotABSEncoder = m_pivotM.getAbsoluteEncoder();

    m_pivotFeedback = m_pivotM.getClosedLoopController();
    m_pivotConf = config.kPivotConfigSparkMax;

    m_pivotM.configure(m_pivotConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Drive init
    m_driveM = new SparkMax(config.kDriveID, MotorType.kBrushless);
    m_driveEncoder = m_driveM.getEncoder();

    m_driveFeedback = m_driveM.getClosedLoopController();
    m_driveConf = config.kDriveConfigSparkMax;

    m_driveM.configure(m_driveConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.pivotEnabled = true; // !
    inputs.driveEnabled = true;

    inputs.pivotABSPose = Rotation2d.fromRotations(m_pivotABSEncoder.getPosition());
    inputs.pivotPose = Rotation2d.fromRotations(m_pivotEncoder.getPosition());
    inputs.pivotVelRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_pivotEncoder.getVelocity());
    inputs.pivotAppliedVolts = m_pivotM.getAppliedOutput() * m_pivotM.getBusVoltage();
    inputs.pivotSupplyCurrent = m_pivotM.getOutputCurrent();

    inputs.drivePoseRads = m_driveEncoder.getPosition();
    inputs.driveVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity());
    inputs.driveAppliedVolts = m_driveM.getAppliedOutput() * m_driveM.getBusVoltage();
    inputs.driveSupplyCurrent = m_driveM.getOutputCurrent();
  }

  @Override
  public void runPivotVolts(double volts) {
    m_pivotM.setVoltage(volts);
  }

  @Override
  public void runDriveVolts(double volts) {
    m_driveM.setVoltage(volts);
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    m_pivotFeedback.setReference(angle.getRadians(), ControlType.kPosition);
  }

  @Override
  public void setDriveVelocity(double velMetersPerSec, double ff) {
    m_driveFeedback.setReference(velMetersPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void setCoastMode(boolean enabled) {
    IdleMode idleMode = enabled ? IdleMode.kCoast : IdleMode.kBrake;
    m_pivotConf.idleMode(idleMode);
    m_driveConf.idleMode(idleMode);

    m_pivotM.configure(
        m_pivotConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_driveM.configure(
        m_driveConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void stop() {
    m_pivotM.stopMotor();
    m_driveM.stopMotor();
  }
}
