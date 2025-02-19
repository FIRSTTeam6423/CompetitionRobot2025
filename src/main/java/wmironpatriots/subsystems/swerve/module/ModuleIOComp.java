// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static wmironpatriots.Constants.kCANbus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import wmironpatriots.Robot;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOComp extends Module {
  private final TalonFX m_pivotM, m_driveM;
  private final CANcoder m_pivotEncoder;

  private final TalonFXConfiguration m_pivotConf, m_driveConf;

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

  public ModuleIOComp(ModuleConfig config) {
    super(config);
    m_pivotM = new TalonFX(config.kPivotID, kCANbus);
    m_driveM = new TalonFX(config.kDriveID, kCANbus);

    // Robot.talonHandler.registerTalon(m_pivotM);
    // Robot.talonHandler.registerTalon(m_driveM);

    m_pivotConf = config.kPivotConfigTalonFX;
    m_driveConf = config.kDriveConfigTalonFX;

    m_pivotEncoder = new CANcoder(config.kPivotABSID);

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
  public void periodic() {
    super.periodic();
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

    pivotOk = true; // ! needs debouncers
    driveOk = true;

    pivotABSPose =
        Rotation2d.fromRotations(m_sigPivotABSPoseRots.getValueAsDouble())
            .minus(new Rotation2d()); // ! Minus offset
    pivotPose = Rotation2d.fromRotations(m_sigPivotPoseRots.getValueAsDouble());
    pivotVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_sigPivotVelRPM.getValueAsDouble());
    pivotAppliedVolts = m_sigPivotAppliedVolts.getValueAsDouble();
    pivotSupplyCurrent = m_sigPivotSupplyCurrent.getValueAsDouble();
    pivotTorqueCurrent = m_sigPivotTorqueCurrent.getValueAsDouble();

    drivePoseRads = Units.rotationsToRadians(m_sigDrivePoseRots.getValueAsDouble());
    driveVelRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_sigDriveVelRPM.getValueAsDouble());
    driveAppliedVolts = m_sigDriveAppliedVolts.getValueAsDouble();
    driveSupplyCurrent = m_sigDriveSupplyCurrent.getValueAsDouble();
    driveTorqueCurrent = m_sigDriveTorqueCurrent.getValueAsDouble();
  }

  @Override
  protected void runPivotControl(ControlRequest request) {
    m_pivotM.setControl(request);
  }

  @Override
  protected void runDriveControl(ControlRequest request) {
    m_driveM.setControl(request);
  }

  @Override
  protected void stopMotors() {
    m_pivotM.stopMotor();
    m_driveM.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_pivotConf.MotorOutput.NeutralMode = idleMode;
    m_driveConf.MotorOutput.NeutralMode = idleMode;

    m_pivotM.getConfigurator().apply(m_pivotConf);
    m_driveM.getConfigurator().apply(m_driveConf);
  }
}
