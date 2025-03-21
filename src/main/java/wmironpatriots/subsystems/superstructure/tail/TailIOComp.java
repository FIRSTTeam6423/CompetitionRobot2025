// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.tail;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import wmironpatriots.Constants.MATRIXID;

public class TailIOComp extends Tail {
  private final TalonFX pivot;

  private final TalonFXConfiguration pivotConf;

  private final BaseStatusSignal pose, vel, volts, current;

  private final VoltageOut reqVolts;
  private final PositionVoltage reqPose;

  private final DigitalInput beam;

  public TailIOComp() {
    pivot = new TalonFX(MATRIXID.TAIL_PIVOT, MATRIXID.RIO);

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

    reqVolts = new VoltageOut(0.0).withEnableFOC(true);
    reqPose = new PositionVoltage(0.0).withEnableFOC(true);

    beam = new DigitalInput(3);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(pose, vel, volts, current);

    poseRevs = pose.getValueAsDouble();
    velRPM = vel.getValueAsDouble();
    appliedVolts = volts.getValueAsDouble();
    currentAmps = current.getValueAsDouble();

    beamTriggered = !beam.get();

    System.out.println(beam.get());
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
  protected void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    pivotConf.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    pivot.getConfigurator().apply(pivotConf);
  }
}
