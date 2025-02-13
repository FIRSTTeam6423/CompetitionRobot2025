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
import org.frc6423.frc2025.util.motorUtil.CTReUtil;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

public class ModuleIOTalonFX implements ModuleIO {

  private final TalonFX m_pivotM, m_driveM;
  private final CANcoder m_pivotEncoder;

  private final TalonFXConfiguration m_pivotConf, m_driveConf;

  // Control Requests
  private final VoltageOut m_voltReq;
  private final TorqueCurrentFOC m_torqueReq;
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
    m_pivotM = new TalonFX(config.kPivotID);
    m_driveM = new TalonFX(config.kDriveID);

    m_pivotConf = config.kPivotConfigTalonFX;
    m_driveConf = config.kDriveConfigTalonFX;

    CTReUtil.registerMotor(m_pivotM);
    CTReUtil.registerMotor(m_driveM);

    m_pivotEncoder = new CANcoder(config.kPivotABSID);

    m_voltReq = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
    m_torqueReq = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    m_poseReq = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    m_velReq = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

    m_sigPivotABSPoseRots = m_pivotEncoder.getAbsolutePosition();
    m_sigPivotPoseRots = m_pivotM.getPosition();
    m_sigPivotVelRPM = m_pivotM.getVelocity();
    m_sigPivotAppliedVolts = m_pivotM.getMotorVoltage();
    m_sigPivotSupplyCurrent = m_pivotM.getStatorCurrent();
    m_sigPivotTorqueCurrent = m_pivotM.getTorqueCurrent();

    m_sigDrivePoseRots = m_driveM.getPosition();
    m_sigDriveVelRPM = m_driveM.getVelocity();
    m_sigDriveAppliedVolts = m_driveM.getMotorVoltage();
    m_sigDriveSupplyCurrent = m_driveM.getStatorCurrent();
    m_sigDriveTorqueCurrent = m_driveM.getTorqueCurrent();

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

    m_pivotEncoder.optimizeBusUtilization();
    m_pivotM.optimizeBusUtilization();
    m_driveM.optimizeBusUtilization();
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
  public void runPivotVolts(double volts, boolean focEnabled) {
    m_pivotM.setControl(m_voltReq.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void runDriveVolts(double volts, boolean focEnabled) {
    m_driveM.setControl(m_voltReq.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  public void setPivotTorque(double currentAmps) {
    m_pivotM.setControl(m_torqueReq.withOutput(currentAmps));
  }

  @Override
  public void setDriveTorque(double currentAmps) {
    m_driveM.setControl(m_torqueReq.withOutput(currentAmps));
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    m_pivotM.setControl(m_poseReq.withPosition(angle.getRotations()));
  }

  @Override
  public void setDriveVelocity(double velMetersPerSec, double torqueFF) {
    m_driveM.setControl(m_velReq.withVelocity(velMetersPerSec).withFeedForward(torqueFF));
  }

  @Override
  public void setCoastMode(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_pivotConf.MotorOutput.NeutralMode = idleMode;
    m_driveConf.MotorOutput.NeutralMode = idleMode;

    m_pivotM.getConfigurator().apply(m_pivotConf);
    m_driveM.getConfigurator().apply(m_driveConf);
  }

  @Override
  public void stop() {
    m_pivotM.stopMotor();
    m_driveM.stopMotor();
  }
}
