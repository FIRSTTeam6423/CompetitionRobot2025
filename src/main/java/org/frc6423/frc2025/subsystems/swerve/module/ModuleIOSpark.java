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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

public class ModuleIOSpark implements ModuleIO {

  private final SparkMax m_pivotMotor, m_driveMotor;
  private SparkMaxConfig m_pivotConfig, m_driveConfig;
  private final RelativeEncoder m_pivotRelativeEncoder, m_driveEncoder;
  private final AbsoluteEncoder m_pivotEncoder;

  private final SparkClosedLoopController m_pivotFeedback, m_driveFeedback;

  public ModuleIOSpark(ModuleConfig config) {
    // Pivot init
    m_pivotMotor = new SparkMax(config.kPivotID, MotorType.kBrushless);
    m_pivotRelativeEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder();

    m_pivotFeedback = m_pivotMotor.getClosedLoopController();
    m_pivotConfig = config.kPivotConfigSparkMax;

    m_pivotMotor.configure(
        m_pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Drive init
    m_driveMotor = new SparkMax(config.kDriveID, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveFeedback = m_driveMotor.getClosedLoopController();
    m_driveConfig = config.kDriveConfigSparkMax;

    m_driveMotor.configure(
        m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.pivotEnabled = true; // !
    inputs.driveEnabled = true;

    inputs.pivotABSPose = Rotation2d.fromRotations(m_pivotEncoder.getPosition());
    inputs.pivotPose = Rotation2d.fromRotations(m_pivotRelativeEncoder.getPosition());
    inputs.pivotVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_pivotRelativeEncoder.getVelocity());
    inputs.pivotAppliedVolts = m_pivotMotor.getAppliedOutput() * m_pivotMotor.getBusVoltage();
    inputs.pivotSupplyCurrent = m_pivotMotor.getOutputCurrent();

    inputs.drivePoseRads = m_driveEncoder.getPosition();
    inputs.driveVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity());
    inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
    inputs.driveSupplyCurrent = m_driveMotor.getOutputCurrent();
  }

  @Override
  public void setPivotVolts(double volts) {
    m_pivotMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVolts(double volts) {
    m_driveMotor.setVoltage(volts);
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
  public void stop() {
    m_pivotMotor.stopMotor();
    m_driveMotor.stopMotor();
  }
}
