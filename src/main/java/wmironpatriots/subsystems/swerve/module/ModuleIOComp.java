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
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOComp extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;

  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

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
    pivot = new TalonFX(config.kPivotID, CANIVORE);
    drive = new TalonFX(config.kDriveID, CANIVORE);
    cancoder = new CANcoder(config.kPivotABSID);

    // Robot.talonHandler.registerTalon(m_pivotM);
    // Robot.talonHandler.registerTalon(m_driveM);

    pivotConf = config.kPivotConfigTalonFX;
    driveConf = config.kDriveConfigTalonFX;
    cancoderConf = config.kCANcoderConfig;

    cancoderConf.MagnetSensor.MagnetOffset = config.kPivotOffset.getRotations();
    cancoderConf.MagnetSensor.SensorDirection =
        config.kPivotInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(config.kCANcoderConfig);

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

    pivotABSPose = Rotation2d.fromRotations(sigPivotABSPoseRots.getValueAsDouble());
    pivotPose = Rotation2d.fromRotations(sigPivotPoseRots.getValueAsDouble());
    pivotVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(sigPivotVelRPM.getValueAsDouble());
    pivotAppliedVolts = sigPivotAppliedVolts.getValueAsDouble();
    pivotSupplyCurrent = sigPivotSupplyCurrent.getValueAsDouble();
    pivotTorqueCurrent = sigPivotTorqueCurrent.getValueAsDouble();

    drivePoseRads = Units.rotationsToRadians(sigDrivePoseRots.getValueAsDouble());
    driveVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(sigDriveVelRPM.getValueAsDouble());
    driveAppliedVolts = sigDriveAppliedVolts.getValueAsDouble();
    driveSupplyCurrent = sigDriveSupplyCurrent.getValueAsDouble();
    driveTorqueCurrent = sigDriveTorqueCurrent.getValueAsDouble();
  }

  @Override
  protected void runPivotControl(ControlRequest request) {
    pivot.setControl(request);
  }

  @Override
  protected void runDriveControl(ControlRequest request) {
    drive.setControl(request);
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
