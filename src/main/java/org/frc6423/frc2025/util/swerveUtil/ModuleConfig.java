// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.util.swerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConfig {
  public int kIndex;
  public moduleType kModuletype;

  public int kPivotID;
  public int kDriveID;
  public int kPivotABSID;

  public Rotation2d kPivotOffset;
  public boolean kPivotInverted;

  public TalonFXConfiguration kPivotConfig, kDriveConfig;
  public CANcoderConfiguration kCANcoderConfig;

  public static enum moduleType {
    SPARKMAX,
    TALONFX
  }

  public ModuleConfig(
      int index,
      moduleType type,
      int pivotID,
      int driveID,
      int pivotABSID,
      Rotation2d pivotOffset,
      boolean pivotInverted,
      TalonFXConfiguration pivotConfig,
      TalonFXConfiguration driveConfig,
      CANcoderConfiguration CANcoderConfig) {
    this.kIndex = index;
    this.kModuletype = type;
    this.kPivotID = pivotID;
    this.kDriveID = driveID;
    this.kPivotABSID = pivotABSID;
    this.kPivotOffset = pivotOffset;
    this.kPivotInverted = pivotInverted;

    pivotConfig.MotorOutput.Inverted = pivotInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    pivotConfig.Feedback.FeedbackRemoteSensorID = kPivotABSID;

    this.kPivotConfig = pivotConfig;
    this.kDriveConfig = driveConfig;
    this.kCANcoderConfig = CANcoderConfig;

  }

  public ModuleConfig(
      int index,
      moduleType type,
      int pivotID,
      int driveID,
      int pivotABSID,
      Rotation2d pivotOffset,
      boolean pivotInverted) {
    this.kIndex = index;
    this.kModuletype = type;
    this.kPivotID = pivotID;
    this.kDriveID = driveID;
    this.kPivotABSID = pivotABSID;
    this.kPivotOffset = pivotOffset;
    this.kPivotInverted = pivotInverted;
  }
}
