// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.intake;

import static wmironpatriots.Constants.CANIVORE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOComp extends Intake {
  private final TalonFX pivot;
  private final TalonFXConfiguration pivotConf;

  public IntakeIOComp() {
    pivot = new TalonFX(15, CANIVORE);
    pivotConf = new TalonFXConfiguration();

    pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConf.Slot0.withKP(0.0).withKI(0.0).withKD(0); // PID config
    pivotConf.Slot0.withKG(0.0).withKS(0.0).withKV(0.0).withKA(0.0); // feedforward config

    pivotConf.CurrentLimits.StatorCurrentLimit = 80.0;
    pivotConf.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConf.CurrentLimits.SupplyCurrentLimit = 80.0;
    pivotConf.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
    pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConf.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    pivotConf.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    pivotConf.MotionMagic.MotionMagicCruiseVelocity = 0.03; // ! TODO
    pivotConf.MotionMagic.MotionMagicAcceleration = 0.03;
    pivotConf.MotionMagic.MotionMagicJerk = 0.0;

    pivot.getConfigurator().apply(pivotConf);
    pivot.optimizeBusUtilization();
  }

  @Override
  protected void runPivotControl(ControlRequest request) {
    pivot.setControl(request);
  }

  @Override
  protected void enablePivotCoasting(boolean booleanEnable) {}
}
