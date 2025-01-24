// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import static org.frc6423.frc2025.Constants.KDriveConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.frc6423.frc2025.Constants.KDriveConstants.ModuleConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ModuleIOSpark implements ModuleIO {

  private final SparkMax m_pivotMotor, m_driveMotor;
  private final SparkMaxConfig m_pivotConfig, m_driveConfig;

  private final RelativeEncoder m_pivotRelativeEncoder, m_driveEncoder;
  private final DutyCycleEncoder m_pivotEncoder;

  private final SparkClosedLoopController m_pivotFeedback, m_driveFeedback;
  private final SimpleMotorFeedforward m_driveFeedforward;

  public ModuleIOSpark(ModuleConfig config) {
    // Pivot init

    m_pivotMotor = new SparkMax(config.pivotID(), MotorType.kBrushless);
    m_pivotRelativeEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder = new DutyCycleEncoder(config.pivotABSID());

    m_pivotFeedback = m_pivotMotor.getClosedLoopController();
    m_pivotConfig = new SparkMaxConfig();

    // Drive init

    m_driveMotor = new SparkMax(config.driveID(), MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    
    m_driveFeedback = m_driveMotor.getClosedLoopController();
    m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    m_driveConfig = new SparkMaxConfig();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {}

  @Override
  public void periodic() {}

  @Override
  public void setPivotVolts(double volts) {}

  @Override
  public void setDriveVolts(double volts) {}

  @Override
  public void setPivotAngle(Rotation2d angle) {}

  @Override
  public void setDriveVelocity(double velMetersPerSec) {}

  @Override
  public void setPivotCoastMode(boolean enabled) {}

  @Override
  public void setDriveCoastMode(boolean enabled) {}

  @Override
  public void stop() {}
}
