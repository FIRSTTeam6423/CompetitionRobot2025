// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbIOComp extends Climb {
  private final SparkMax climb;
  private final SparkMaxConfig config;

  public ClimbIOComp() {
    climb = new SparkMax(2, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
  }

  @Override
  public Command runClimb(double volts) {
    return this.run(() -> climb.setVoltage(volts));
  }
}
