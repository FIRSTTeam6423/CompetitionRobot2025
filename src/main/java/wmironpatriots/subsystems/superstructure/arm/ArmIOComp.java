// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import wmironpatriots.Constants.MATRIXID;

public class ArmIOComp extends Arm {
  private final TalonFX motor;

  private final TalonFXConfiguration conf;

  private final BaseStatusSignal poseSig, currentSig, torqueSig, voltsSig, tempSig;

  private final TorqueCurrentFOC reqCurrent;
  private final PositionTorqueCurrentFOC reqPose;

  private final DigitalInput beam;

  public ArmIOComp() {
    motor = new TalonFX(MATRIXID.TAIL_PIVOT, MATRIXID.RIO);

    conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.Slot0.kG = 0.0;
    conf.Slot0.kV = 2.5;
    conf.Slot0.kP = 0.4;
    conf.Slot0.kI = 0.1;
    conf.Slot0.kD = 0.0;
    conf.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    conf.MotionMagic.MotionMagicAcceleration = 0.0;
    conf.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    conf.MotionMagic.MotionMagicExpo_kA = 0.0;
    conf.MotionMagic.MotionMagicJerk = 0.0;

    conf.CurrentLimits.StatorCurrentLimit = 20.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimit = 20.0;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLowerLimit = -20.0;
    motor.getConfigurator().apply(conf);

    poseSig = motor.getPosition();
    currentSig = motor.getSupplyCurrent();
    torqueSig = motor.getTorqueCurrent();
    voltsSig = motor.getMotorVoltage();
    tempSig = motor.getDeviceTemp();

    reqCurrent = new TorqueCurrentFOC(0.0);
    reqPose = new PositionTorqueCurrentFOC(0.0);

    beam = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(poseSig, currentSig, torqueSig, voltsSig, tempSig);
    inputs.data = new ArmData(poseSig.getValueAsDouble(), currentSig.getValueAsDouble(), torqueSig.getValueAsDouble(), voltsSig.getValueAsDouble(), tempSig.getValueAsDouble(), !beam.get());
  }

  @Override
  protected void setMotorCurrent(double amps) {
    motor.setControl(reqCurrent.withOutput(amps));
  }

  @Override
  protected void setMotorPose(double poseRevs) {
    motor.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  protected void setEncoderPose(double poseRevs) {
    motor.setPosition(poseRevs);
  }

  @Override
  public void stopMotors() {
    motor.stopMotor();
  }

  @Override
  protected void enableCoastMode(boolean enabled) {
    conf.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    motor.getConfigurator().apply(conf);
  }
}