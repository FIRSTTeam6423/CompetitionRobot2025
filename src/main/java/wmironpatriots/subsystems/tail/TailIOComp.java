// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class TailIOComp extends Tail {
  private final TalonFX pivot;
  private final SparkMax roller;

  private final TalonFXConfiguration pivotConf;
  private final SparkMaxConfig rollerConf;

  private final BaseStatusSignal sigPose, sigVel, sigVolts, sigCurrent;

  private final PositionVoltage reqPose = new PositionVoltage(0.0).withEnableFOC(true);
  private final VoltageOut reqVolts = new VoltageOut(0.0).withEnableFOC(true);

  public TailIOComp() {
    pivot = new TalonFX(13, "rio");
    roller = new SparkMax(1, MotorType.kBrushless);

    pivotConf = new TalonFXConfiguration();
    pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConf.Slot0.kG = 0.0;
    pivotConf.Slot0.kV = 0.0;
    pivotConf.Slot0.kP = 0.3;
    pivotConf.Slot0.kI = 0.0;
    pivotConf.Slot0.kD = 0.0;

    pivotConf.CurrentLimits.StatorCurrentLimit = 20.0;
    pivotConf.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConf.CurrentLimits.SupplyCurrentLimit = 20.0;
    pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConf.CurrentLimits.SupplyCurrentLowerLimit = -20.0;

    pivot.getConfigurator().apply(pivotConf);

    rollerConf = new SparkMaxConfig();
    rollerConf.idleMode(IdleMode.kBrake);
    roller.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sigPose = pivot.getPosition();
    sigVel = pivot.getVelocity();
    sigVolts = pivot.getMotorVoltage();
    sigCurrent = pivot.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, sigPose, sigVel, sigVolts, sigCurrent);
  }

  @Override
  public void periodic() {
    pivotMotorOk = BaseStatusSignal.refreshAll(sigPose, sigVel, sigVolts, sigCurrent).isOK();

    pivotPoseRevs = sigPose.getValueAsDouble();
    pivotVelRPM = sigPose.getValueAsDouble();
    pivotAppliedVolts = sigVolts.getValueAsDouble();
    pivotSupplyCurrentAmps = sigCurrent.getValueAsDouble();

    rollerAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    rollerSupplyCurrentAmps = roller.getOutputCurrent();
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivot.setControl(reqVolts.withEnableFOC(true).withOutput(volts));
  }

  @Override
  protected void runPivotSetpoint(double setpointRevs) {
    pivot.setControl(reqPose.withEnableFOC(true).withPosition(setpointRevs));
  }

  @Override
  protected void runRollerSpeed(double speed) {
    roller.setVoltage(speed);
  }

  @Override
  protected void setEncoderPose(double poseRevs) {
    pivot.setPosition(poseRevs);
  }

  @Override
  protected void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  protected void stopRollers() {
    roller.stopMotor();
  }

  @Override
  protected void pivotCoastingEnabled(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotConf.MotorOutput.NeutralMode = idleMode;
  }
}
