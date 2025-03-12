// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.chute;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;

public class ChuteIOComp extends Chute {
  private final SparkMax roller;
  private final SparkMaxConfig rollerConfig;

  public ChuteIOComp() {
    roller = new SparkMax(3, MotorType.kBrushless);
    rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    chuteAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    chuteSpeedRPM = roller.get();
    chuteSupplyCurrent = roller.getOutputCurrent();
  }

  @Override
  public Command runChuteSpeedCmmd(double speed) {
    return this.run(
        () -> {
          roller.set(speed);
        });
  }
}
