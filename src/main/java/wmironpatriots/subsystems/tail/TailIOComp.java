// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import wmironpatriots.Constants.MATRIXID;

public class TailIOComp extends Tail {
  private final TalonFX pivot;
  private final SparkMax rollers;
  
  private final TalonFXConfiguration pivotConf;
  private final SparkMaxConfig rollerConf;

  private final BaseStatusSignal pose, vel, volts, current;

  private final VoltageOut reqVolts;
  private final MotionMagicVoltage reqPose;

  public TailIOComp() {
    pivot = new TalonFX(MATRIXID.TAIL_PIVOT, MATRIXID.RIO);
    rollers = new SparkMax(MATRIXID.TAIL_ROLLER, MotorType.kBrushless);

    pivotConf = new TalonFXConfiguration();
    pivotConf.Audio.AllowMusicDurDisable = true;
    pivotConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    pivotConf.Slot0.kG = 0.0;
    pivotConf.Slot0.kV = 2.5;
    pivotConf.Slot0.kP = 0.4;
    pivotConf.Slot0.kI = 0.1;
    pivotConf.Slot0.kD = 0.0;
    pivotConf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotConf.MotionMagic.MotionMagicAcceleration = 0.0;
    pivotConf.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    pivotConf.MotionMagic.MotionMagicExpo_kA = 0.0;
    pivotConf.MotionMagic.MotionMagicJerk = 0.0;

    pivotConf.CurrentLimits.StatorCurrentLimit = 20.0;
    pivotConf.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConf.CurrentLimits.SupplyCurrentLimit = 20.0;
    pivotConf.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConf.CurrentLimits.SupplyCurrentLowerLimit = -20.0;
    pivot.getConfigurator().apply(pivotConf);

    pose = pivot.getPosition();
    vel = pivot.getVelocity();
    volts = pivot.getMotorVoltage();
    current = pivot.getStatorCurrent();

    rollerConf = new SparkMaxConfig();
    rollers.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    reqVolts = new VoltageOut(0.0).withEnableFOC(true);
    reqPose = new MotionMagicVoltage(0.0).withEnableFOC(true);
  }

  @Override
  public void periodic() {
    poseRevs = pose.getValueAsDouble();
    velRPM = vel.getValueAsDouble();
    appliedVolts = volts.getValueAsDouble();
    currentAmps = current.getValueAsDouble();
  }

  @Override
  protected void setPivotVolts(double volts) {
    pivot.setControl(reqVolts.withOutput(volts));
  }

  @Override
  protected void setPivotPose(double poseRevs) {
    pivot.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  protected void setEncoderPose(double poseRevs) {
    pivot.setPosition(poseRevs);
  }

  @Override
  protected void setRollerVolts(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  protected void stopMotors() {
    pivot.stopMotor();
    rollers.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    pivotConf.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    rollerConf.idleMode(enabled ? IdleMode.kCoast : IdleMode.kBrake);

    pivot.getConfigurator().apply(pivotConf);
    rollers.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
