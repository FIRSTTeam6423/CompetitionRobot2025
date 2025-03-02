// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static wmironpatriots.Constants.CANIVORE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOComp extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;

  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final VoltageOut reqMotorVolts =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final VelocityVoltage reqMotorVel =
      new VelocityVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final PositionVoltage reqMotorPose =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  // Signals
  private final BaseStatusSignal sigPivotABSPoseRots,
      sigPivotPoseRots,
      sigPivotVelRPM,
      sigPivotAppliedVolts,
      sigPivotSupplyCurrent,
      sigPivotTorqueCurrent;
  private final BaseStatusSignal sigDrivePoseRots,
      sigDriveVelRPM,
      sigDriveAppliedVolts,
      sigDriveSupplyCurrent,
      sigDriveTorqueCurrent;

  public ModuleIOComp(ModuleConfig config) {
    super(config);
    pivot = new TalonFX(config.pivotID, CANIVORE);
    drive = new TalonFX(config.driveID, CANIVORE);
    cancoder = new CANcoder(config.pivotCancoderID);

    pivotConf = config.pivotConfig;
    driveConf = config.driveConfig;
    cancoderConf = config.cancoderConfig;

    cancoderConf.MagnetSensor.MagnetOffset = config.pivotOffset.getRotations();
    cancoderConf.MagnetSensor.SensorDirection =
        config.pivotInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(config.cancoderConfig);

    sigPivotABSPoseRots = cancoder.getAbsolutePosition();
    sigPivotPoseRots = pivot.getPosition();
    sigPivotVelRPM = pivot.getVelocity();
    sigPivotAppliedVolts = pivot.getMotorVoltage();
    sigPivotSupplyCurrent = pivot.getStatorCurrent();
    sigPivotTorqueCurrent = pivot.getTorqueCurrent();

    sigDrivePoseRots = drive.getPosition();
    sigDriveVelRPM = drive.getVelocity();
    sigDriveAppliedVolts = drive.getMotorVoltage();
    sigDriveSupplyCurrent = drive.getStatorCurrent();
    sigDriveTorqueCurrent = drive.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, sigPivotPoseRots, sigDrivePoseRots);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        sigPivotABSPoseRots,
        sigPivotVelRPM,
        sigPivotAppliedVolts,
        sigPivotSupplyCurrent,
        sigPivotTorqueCurrent,
        sigDriveVelRPM,
        sigDriveAppliedVolts,
        sigDriveSupplyCurrent,
        sigDriveTorqueCurrent);

    cancoder.optimizeBusUtilization();
    pivot.optimizeBusUtilization();
    drive.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    super.periodic();
    BaseStatusSignal.refreshAll(
        sigPivotABSPoseRots,
        sigPivotPoseRots,
        sigPivotVelRPM,
        sigPivotAppliedVolts,
        sigPivotSupplyCurrent,
        sigPivotTorqueCurrent,
        sigDrivePoseRots,
        sigDriveVelRPM,
        sigDriveAppliedVolts,
        sigDriveSupplyCurrent,
        sigDriveTorqueCurrent);

    pivotOk = true; // ! needs debouncers
    driveOk = true;

    pivotABSPoseRads = sigPivotABSPoseRots.getValueAsDouble() * 2 * Math.PI;
    pivotPoseRads = sigPivotPoseRots.getValueAsDouble() * 2 * Math.PI;
    pivotVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(sigPivotVelRPM.getValueAsDouble());
    pivotAppliedVolts = sigPivotAppliedVolts.getValueAsDouble();
    pivotSupplyCurrent = sigPivotSupplyCurrent.getValueAsDouble();
    pivotTorqueCurrent = sigPivotTorqueCurrent.getValueAsDouble();

    drivePoseMeters = sigDrivePoseRots.getValueAsDouble() * 2 * Math.PI * config.wheelRadiusMeters;
    driveVelMPS = Units.rotationsPerMinuteToRadiansPerSecond(sigDriveVelRPM.getValueAsDouble());
    driveAppliedVolts = sigDriveAppliedVolts.getValueAsDouble();
    driveSupplyCurrent = sigDriveSupplyCurrent.getValueAsDouble();
    driveTorqueCurrent = sigDriveTorqueCurrent.getValueAsDouble();
  }

  @Override
  protected void runPivotPose(double poseRads) {
    pivot.setControl(reqMotorPose.withPosition(poseRads / (2 * Math.PI)));
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivot.setControl(reqMotorVolts.withOutput(volts).withEnableFOC(true));
  }

  @Override
  protected void runDriveVolts(double volts, boolean focEnabled) {
    drive.setControl(reqMotorVolts.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  protected void runDriveVel(double velMPS, double torqueff) {
    drive.setControl(reqMotorVel.withVelocity(velMPS).withFeedForward(torqueff));
  }

  @Override
  protected void stopMotors() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotConf.MotorOutput.NeutralMode = idleMode;
    driveConf.MotorOutput.NeutralMode = idleMode;

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
  }
}
