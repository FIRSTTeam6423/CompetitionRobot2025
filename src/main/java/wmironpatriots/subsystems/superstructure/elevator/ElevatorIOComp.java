// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import wmironpatriots.Constants.MATRIXID;

public class ElevatorIOComp extends Elevator {
  private final TalonFX parent, child;
  private final TalonFXConfiguration conf;

  private final PositionTorqueCurrentFOC reqPose;
  private final TorqueCurrentFOC reqTorque;

  private final BaseStatusSignal pose;
  private final BaseStatusSignal parentCurrent, parentTorque, parentVolts, parentTemp;
  private final BaseStatusSignal childCurrent, childTorque, childVolts, childTemp;

  public ElevatorIOComp() {
    parent = new TalonFX(MATRIXID.ELEVATOR_PARENT, MATRIXID.CANCHAN);
    child = new TalonFX(MATRIXID.ELEVATOR_CHILD, MATRIXID.CANCHAN);

    conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    conf.CurrentLimits.StatorCurrentLimit = 80.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimit = 40.0;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;

    conf.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    conf.TorqueCurrent.PeakReverseTorqueCurrent = 40.0;
    conf.TorqueCurrent.TorqueNeutralDeadband = 0.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.Slot1.kG = 0.59;
    conf.Slot1.kV = 4.12;
    conf.Slot1.kP = 1;
    conf.Slot1.kI = 0.0;
    conf.Slot1.kD = 0.0;
    conf.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    // TODO get torque based constants
    conf.Slot0.kG = 0.0;
    conf.Slot0.kV = 0.0;
    conf.Slot0.kP = 0.0;
    conf.Slot0.kI = 0.0;
    conf.Slot0.kD = 0.0;
    conf.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    parent.getConfigurator().apply(conf);
    parent.setPosition(0.0);
    child.getConfigurator().apply(conf);
    child.setControl(new Follower(parent.getDeviceID(), true));

    reqPose = new PositionTorqueCurrentFOC(0.0);
    reqTorque = new TorqueCurrentFOC(0.0);

    pose = parent.getPosition();
    parentCurrent = parent.getStatorCurrent();
    parentTorque = parent.getTorqueCurrent();
    parentVolts = parent.getMotorVoltage();
    parentTemp = parent.getDeviceTemp();

    childCurrent = child.getStatorCurrent();
    childTorque = child.getTorqueCurrent();
    childVolts = child.getMotorVoltage();
    childTemp = child.getDeviceTemp();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        parentCurrent,
        parentTorque,
        parentVolts,
        parentTemp,
        childCurrent,
        childTorque,
        childVolts,
        childTemp);

    poseRevs = pose.getValueAsDouble();
    parentCurrentAmps = parentCurrent.getValueAsDouble();
    parentTorqueAmps = parentTorque.getValueAsDouble();
    parentAppliedVolts = parentVolts.getValueAsDouble();
    parentTempCelsius = parentTemp.getValueAsDouble();

    childCurrentAmps = childCurrent.getValueAsDouble();
    childTorqueAmps = childTorque.getValueAsDouble();
    childAppliedVolts = childVolts.getValueAsDouble();
    childTempCelsius = childTemp.getValueAsDouble();
  }

  @Override
  public void setMotorCurrent(double amps) {
    parent.setControl(reqTorque.withOutput(amps));
  }

  @Override
  public void setMotorPose(double poseRevs) {
    parent.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  public void stopMotors() {
    parent.stopMotor();
  }

  @Override
  public void setEncoderPose(double poseRevs) {
    parent.setPosition(poseRevs);
  }

  @Override
  public void enableCoastMode(boolean enabled) {
    conf.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    parent.getConfigurator().apply(conf);
    child.getConfigurator().apply(conf);
  }
}
