// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import lib.utils.TalonFxUtil;
import wmironpatriots.subsystems.Swerve.SwerveConstants.ModuleConfig;

/**
 * Represents a Swerve module with the following specifications:
 *
 * <p>MK4i /w l3 ratio
 *
 * <p>Kraken x60 for pivot
 *
 * <p>Kraken x44 for drive
 *
 * <p>CANcoder absolute encoder
 */
public class ModuleHardwareComp implements ModuleHardware {
  private final int index;

  protected final TalonFX pivot, drive;
  private final CANcoder cancoder;

  private final TalonFXConfiguration pivotCfg, driveCfg;
  private final CANcoderConfiguration cancoderCfg;

  private final VoltageOut voltReq = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC poseReq = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velReq = new VelocityTorqueCurrentFOC(0.0);

  private final BaseStatusSignal pivotPose, pivotVolts, pivotCurrent, pivotTorque;
  private final BaseStatusSignal drivePose, driveSpeed, driveVolts, driveCurrent, driveTorque;
  private final BaseStatusSignal cancoderPose;

  public ModuleHardwareComp(ModuleConfig moduleConfig) {
    index = moduleConfig.index();

    pivot = new TalonFX(moduleConfig.pivotId().getId(), moduleConfig.pivotId().getBusName());
    drive = new TalonFX(moduleConfig.driveId().getId(), moduleConfig.driveId().getBusName());

    cancoder =
        new CANcoder(moduleConfig.encoderId().getId(), moduleConfig.encoderId().getBusName());

    // Pivot Configs
    pivotCfg = TalonFxUtil.getDefaultTalonFxCfg();

    pivotCfg.MotorOutput.Inverted =
        moduleConfig.pivotInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotCfg.CurrentLimits.StatorCurrentLimit = 40.0;
    pivotCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotCfg.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    pivotCfg.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    pivotCfg.TorqueCurrent.TorqueNeutralDeadband = 0.0;
    pivotCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    pivotCfg.ClosedLoopGeneral.ContinuousWrap = true;
    pivotCfg.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId().getId();
    pivotCfg.Feedback.FeedbackRotorOffset = moduleConfig.encoderOffsetRevs();
    pivotCfg.Feedback.RotorToSensorRatio = 0.0; // TODO
    pivotCfg.Feedback.SensorToMechanismRatio = 0.0;
    pivotCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    pivotCfg.Slot0.kP = 600.0;
    pivotCfg.Slot0.kD = 50.0;
    pivotCfg.Slot0.kA = 0.0;
    pivotCfg.Slot0.kV = 10;
    pivotCfg.Slot0.kS = 0.014;

    pivot.getConfigurator().apply(pivotCfg);

    // Drive Configs
    driveCfg = TalonFxUtil.getDefaultTalonFxCfg();

    driveCfg.MotorOutput.Inverted =
        moduleConfig.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveCfg.CurrentLimits.StatorCurrentLimit = 120.0;
    driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    driveCfg.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    driveCfg.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    driveCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    driveCfg.ClosedLoopGeneral.ContinuousWrap = true;
    driveCfg.Feedback.SensorToMechanismRatio = 0.0; // TODO
    driveCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    driveCfg.Slot0.kP = 35.0;
    driveCfg.Slot0.kD = 0.0;
    driveCfg.Slot0.kA = 0.0;
    driveCfg.Slot0.kV = 0.0;
    driveCfg.Slot0.kS = 5.0;

    drive.getConfigurator().apply(driveCfg);

    // CANcoder configs
    cancoderCfg = new CANcoderConfiguration();

    cancoderCfg.MagnetSensor.MagnetOffset = moduleConfig.encoderOffsetRevs();
    cancoderCfg.MagnetSensor.SensorDirection =
        moduleConfig.pivotInverted()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    cancoder.getConfigurator().apply(cancoderCfg);

    // Status Signals
    pivotPose = pivot.getPosition();
    pivotVolts = pivot.getMotorVoltage();
    pivotCurrent = pivot.getStatorCurrent();
    pivotTorque = pivot.getTorqueCurrent();

    drivePose = drive.getPosition();
    driveSpeed = drive.getVelocity();
    driveVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveTorque = drive.getTorqueCurrent();

    cancoderPose = cancoder.getAbsolutePosition();
  }

  @Override
  public LoggableState getLoggableState() {
    return new LoggableState(
        index,
        BaseStatusSignal.refreshAll(pivotPose, pivotVolts, pivotCurrent, pivotTorque).isOK(),
        pivotPose.getValueAsDouble(),
        poseReq.Position,
        pivotVolts.getValueAsDouble(),
        pivotCurrent.getValueAsDouble(),
        pivotTorque.getValueAsDouble(),
        BaseStatusSignal.refreshAll(drivePose, driveSpeed, driveVolts, driveCurrent, driveTorque)
            .isOK(),
        drivePose.getValueAsDouble(),
        driveSpeed.getValueAsDouble(),
        velReq.Velocity,
        driveVolts.getValueAsDouble(),
        driveCurrent.getValueAsDouble(),
        driveTorque.getValueAsDouble(),
        BaseStatusSignal.refreshAll(cancoderPose).isOK(),
        cancoderPose.getValueAsDouble());
  }

  @Override
  public void setPivotAppliedVolts(double volts) {
    pivot.setControl(voltReq.withOutput(volts));
  }

  @Override
  public void setDriveAppliedVolts(double volts) {
    drive.setControl(voltReq.withOutput(volts));
  }

  @Override
  public void setPivotSetpointPose(double poseRevs) {
    pivot.setControl(poseReq.withPosition(poseRevs));
  }

  @Override
  public void setDriveSetpointSpeed(double speedMps) {
    drive.setControl(velReq.withVelocity(speedMps));
  }

  @Override
  public void stop() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  public void coastingEnabled(boolean enabled) {
    var neutralMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    pivotCfg.MotorOutput.NeutralMode = neutralMode;
    driveCfg.MotorOutput.NeutralMode = neutralMode;

    pivot.getConfigurator().apply(pivotCfg);
    drive.getConfigurator().apply(driveCfg);
  }
}
