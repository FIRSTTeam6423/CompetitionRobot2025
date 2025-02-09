// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.frc6423.frc2025.util.motorUtil.TalonFXUtil;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

public class ModuleIOTalonFX implements ModuleIO {

  private final TalonFX m_pivotMotor, m_driveMotor;
  private final CANcoder m_pivotCANcoder;

  private final TalonFXConfiguration m_pivotConfig, m_driveConfig;

  // Control Requests
  private final VoltageOut m_voltReq;
  private final TorqueCurrentFOC m_torqueCurrentReq;
  private final PositionTorqueCurrentFOC m_poseReq;
  private final VelocityTorqueCurrentFOC m_velReq;

  // Signals
  private final BaseStatusSignal m_sigPivotABSPoseRots,
      m_sigPivotPoseRots,
      m_sigPivotVelRPM,
      m_sigPivotAppliedVolts,
      m_sigPivotSupplyCurrent,
      m_sigPivotTorqueCurrent;
  private final BaseStatusSignal m_sigDrivePoseRots,
      m_sigDriveVelRPM,
      m_sigDriveAppliedVolts,
      m_sigDriveSupplyCurrent,
      m_sigDriveTorqueCurrent;

  public ModuleIOTalonFX(ModuleConfig config) {
    m_pivotMotor = new TalonFX(config.kPivotID);
    m_driveMotor = new TalonFX(config.kDriveID);

    m_pivotConfig = config.kPivotConfigTalonFX;
    m_driveConfig = config.kDriveConfigTalonFX;

    TalonFXUtil.registerMotor(m_pivotMotor);
    TalonFXUtil.registerMotor(m_driveMotor);

    m_pivotCANcoder = new CANcoder(config.kPivotABSID);

    m_voltReq = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    m_torqueCurrentReq = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    m_poseReq = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    m_velReq = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

    m_sigPivotABSPoseRots = m_pivotCANcoder.getAbsolutePosition();
    m_sigPivotPoseRots = m_pivotMotor.getPosition();
    m_sigPivotVelRPM = m_pivotMotor.getVelocity();
    m_sigPivotAppliedVolts = m_pivotMotor.getMotorVoltage();
    m_sigPivotSupplyCurrent = m_pivotMotor.getStatorCurrent();
    m_sigPivotTorqueCurrent = m_pivotMotor.getTorqueCurrent();

    m_sigDrivePoseRots = m_driveMotor.getPosition();
    m_sigDriveVelRPM = m_driveMotor.getVelocity();
    m_sigDriveAppliedVolts = m_driveMotor.getMotorVoltage();
    m_sigDriveSupplyCurrent = m_driveMotor.getStatorCurrent();
    m_sigDriveTorqueCurrent = m_driveMotor.getTorqueCurrent();

    // ! Register to odo thread

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_sigPivotABSPoseRots,
        m_sigPivotPoseRots,
        m_sigPivotVelRPM,
        m_sigPivotAppliedVolts,
        m_sigPivotSupplyCurrent,
        m_sigPivotTorqueCurrent,
        m_sigDrivePoseRots,
        m_sigDriveVelRPM,
        m_sigDriveAppliedVolts,
        m_sigDriveSupplyCurrent,
        m_sigDriveTorqueCurrent);

    m_pivotCANcoder.optimizeBusUtilization();
    m_pivotMotor.optimizeBusUtilization();
    m_driveMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_sigPivotABSPoseRots,
        m_sigPivotPoseRots,
        m_sigPivotVelRPM,
        m_sigPivotAppliedVolts,
        m_sigPivotSupplyCurrent,
        m_sigPivotTorqueCurrent,
        m_sigDrivePoseRots,
        m_sigDriveVelRPM,
        m_sigDriveAppliedVolts,
        m_sigDriveSupplyCurrent,
        m_sigDriveTorqueCurrent);

    inputs.pivotEnabled = true; // ! needs debouncers
    inputs.driveEnabled = true;

    inputs.pivotABSPose =
        Rotation2d.fromRotations(m_sigPivotABSPoseRots.getValueAsDouble())
            .minus(new Rotation2d()); // ! Minus offset
    inputs.pivotPose = Rotation2d.fromRotations(m_sigPivotPoseRots.getValueAsDouble());
    inputs.pivotVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_sigPivotVelRPM.getValueAsDouble());
    inputs.pivotAppliedVolts = m_sigPivotAppliedVolts.getValueAsDouble();
    inputs.pivotSupplyCurrent = m_sigPivotSupplyCurrent.getValueAsDouble();
    inputs.pivotTorqueCurrent = m_sigPivotTorqueCurrent.getValueAsDouble();

    inputs.drivePoseRads = Units.rotationsToRadians(m_sigDrivePoseRots.getValueAsDouble());
    inputs.driveVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_sigDriveVelRPM.getValueAsDouble());
    inputs.driveAppliedVolts = m_sigDriveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrent = m_sigDriveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrent = m_sigDriveTorqueCurrent.getValueAsDouble();
  }

  @Override
  public void setPivotVolts(double volts, boolean focEnabled) {
    m_pivotMotor.setControl(m_voltReq.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setDriveVolts(double volts, boolean focEnabled) {
    m_driveMotor.setControl(m_voltReq.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setPivotTorqueCurrent(double currentAmps) {
    m_pivotMotor.setControl(m_torqueCurrentReq.withOutput(currentAmps));
  }

  @Override
  public void setDriveTorqueCurrent(double currentAmps) {
    m_driveMotor.setControl(m_torqueCurrentReq.withOutput(currentAmps));
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    m_pivotMotor.setControl(m_poseReq.withPosition(angle.getRotations()));
  }

  @Override
  public void setDriveVelocity(double velMetersPerSec, double ff) {
    m_driveMotor.setControl(m_velReq.withVelocity(velMetersPerSec).withFeedForward(ff));
  }

  @Override
  public void enableCoast(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_pivotConfig.MotorOutput.NeutralMode = idleMode;
    m_driveConfig.MotorOutput.NeutralMode = idleMode;

    m_pivotMotor.getConfigurator().apply(m_pivotConfig);
    m_driveMotor.getConfigurator().apply(m_driveConfig);
  }

  @Override
  public void stop() {
    m_pivotMotor.stopMotor();
    m_driveMotor.stopMotor();
  }
}
