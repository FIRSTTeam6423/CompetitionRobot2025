// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.tail.roller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import wmironpatriots.Constants.MATRIXID;

public class RollerIOComp extends Roller {
  private final SparkMax roller;
  private final RelativeEncoder rollerEncoder;
  private final SparkMaxConfig rollerConf;

  public RollerIOComp() {
    roller = new SparkMax(MATRIXID.TAIL_ROLLER, MotorType.kBrushless);
    rollerEncoder = roller.getEncoder();

    rollerConf = new SparkMaxConfig();
    roller.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    poseRevs = rollerEncoder.getPosition();
    velRPM = rollerEncoder.getVelocity();
  }

  @Override
  protected void setRollerVolts(double volts) {
    roller.setVoltage(volts);
  }

  @Override
  protected void setRollerSpeed(double speeds) {
    roller.set(speeds);
  }

  @Override
  protected void stopRollers() {
    roller.stopMotor();
  }
}
