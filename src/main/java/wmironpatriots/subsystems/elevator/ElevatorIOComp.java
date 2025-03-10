// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import static wmironpatriots.Constants.MATRIXID.CANCHAN;
import static wmironpatriots.Constants.MATRIXID.ELEVATOR_CHILD;
import static wmironpatriots.Constants.MATRIXID.ELEVATOR_PARENT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOComp extends Elevator {
  private final TalonFX parent, child;
  private final TalonFXConfiguration motorConf;

  private final MotionMagicVoltage reqPose;
  private final VoltageOut reqVolts;

  private final BaseStatusSignal pose, vel, volts, current;

  public ElevatorIOComp() {
    parent = new TalonFX(ELEVATOR_PARENT, CANCHAN);
    child = new TalonFX(ELEVATOR_CHILD, CANCHAN);

    motorConf = new TalonFXConfiguration();
    motorConf.Audio.AllowMusicDurDisable = true;
    motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConf.CurrentLimits.StatorCurrentLimit = 80.0;
    motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConf.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConf.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConf.Slot0.kG = 0.59;
    motorConf.Slot0.kV = 4.12;
    motorConf.Slot0.kP = 1;
    motorConf.Slot0.kI = 0.0;
    motorConf.Slot0.kD = 0.0;
    motorConf.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    motorConf.MotionMagic.MotionMagicAcceleration = 0.0;
    motorConf.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    motorConf.MotionMagic.MotionMagicExpo_kV = 0.0;
    motorConf.MotionMagic.MotionMagicExpo_kA = 0.0;
    motorConf.MotionMagic.MotionMagicJerk = 0.0;

    parent.getConfigurator().apply(motorConf);
    parent.setPosition(0.0);
    child.getConfigurator().apply(motorConf);
    child.setControl(new Follower(parent.getDeviceID(), true));

    reqPose = new MotionMagicVoltage(0.0).withEnableFOC(true);
    reqVolts = new VoltageOut(0.0).withEnableFOC(true);

    pose = parent.getPosition();
    vel = parent.getVelocity();
    volts = parent.getMotorVoltage();
    current = parent.getStatorCurrent();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(pose, vel, volts, current);
    poseRevs = pose.getValueAsDouble();
    velRPM = vel.getValueAsDouble();
    appliedVolts = volts.getValueAsDouble();
    appliedVolts = current.getValueAsDouble();
  }

  @Override
  protected void setMotorVolts(double volts) {
    parent.setControl(reqVolts.withOutput(volts));
  }

  @Override
  protected void setMotorPose(double poseRevs) {
    parent.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  protected void setEncoderPose(double poseRevs) {
    parent.setPosition(poseRevs);
  }

  @Override
  protected void stopMotors() {
    parent.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    motorConf.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    parent.getConfigurator().apply(motorConf);
  }
}
