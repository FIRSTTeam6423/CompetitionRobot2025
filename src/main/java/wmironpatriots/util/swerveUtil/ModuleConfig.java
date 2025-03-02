// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.swerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConfig {
  public int index;

  public int pivotID;
  public int driveID;
  public int pivotCancoderID;

  public Rotation2d pivotOffset;
  public boolean pivotInverted;

  public final TalonFXConfiguration pivotConfig, driveConfig;
  public final CANcoderConfiguration cancoderConfig;

  public double pivotReduction, driveReduction;
  public double wheelRadiusMeters;

  /** Create a new TalonFX modul */
  public ModuleConfig(
      int index,
      int pivotID,
      int driveID,
      int pivotABSID,
      Rotation2d pivotOffset,
      boolean pivotInverted,
      double pivotReduction,
      double driveReduction,
      double wheelRadius,
      TalonFXConfiguration pivotConfig,
      TalonFXConfiguration driveConfig,
      CANcoderConfiguration CANcoderConfig) {
    this.index = index;
    this.pivotID = pivotID;
    this.driveID = driveID;
    this.pivotCancoderID = pivotABSID;
    this.pivotOffset = pivotOffset;
    this.pivotInverted = pivotInverted;

    pivotConfig.MotorOutput.Inverted =
        pivotInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotCancoderID;

    this.pivotReduction = pivotReduction;
    this.driveReduction = driveReduction;
    this.wheelRadiusMeters = wheelRadius;

    this.pivotConfig = pivotConfig;
    this.driveConfig = driveConfig;
    this.cancoderConfig = CANcoderConfig;
  }
}
