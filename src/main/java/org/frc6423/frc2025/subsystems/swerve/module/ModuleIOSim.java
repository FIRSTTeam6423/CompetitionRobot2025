// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim m_pivotSim, m_driveSim;
  private final TalonFX m_driveMotor;
  private final VoltageOut m_driveVoltage;
  private final VelocityTorqueCurrentFOC m_driveVelocityControl;

  private final PIDController m_pivotFeedback;

  private double pivotAppliedVolts;
  private double driveReduction;

  public ModuleIOSim(ModuleConfig config) {
    DCMotor pivotMotor = DCMotor.getKrakenX60(1);
    DCMotor driveMotor = DCMotor.getKrakenX60(1);

    m_driveMotor = new TalonFX(config.kDriveID);
    m_driveMotor.getConfigurator().apply(config.kDriveConfigTalonFX);

    m_driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
    m_driveVelocityControl = new VelocityTorqueCurrentFOC(0.0).withSlot(0);

    m_pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                pivotMotor, 0.004, config.kPivotConfigTalonFX.Feedback.RotorToSensorRatio),
            pivotMotor);
    driveReduction = config.kDriveConfigTalonFX.Feedback.RotorToSensorRatio;
    m_driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotor, 0.025, driveReduction), driveMotor);

    m_pivotFeedback = new PIDController(100.0, 0, 0);

    m_pivotFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    TalonFXSimState driveSimState = m_driveMotor.getSimState();
    driveSimState.Orientation = ChassisReference.Clockwise_Positive;

    m_driveSim.setInput(driveSimState.getMotorVoltage());

    m_pivotSim.update(0.02);
    m_driveSim.update(0.02);
    driveSimState.setRotorVelocity((m_driveSim.getAngularVelocityRPM() / 60) * driveReduction);

    inputs.pivotEnabled = true;
    inputs.driveEnabled = true;

    inputs.pivotABSPose = Rotation2d.fromRadians(m_pivotSim.getAngularPositionRad());
    inputs.pivotPose = inputs.pivotABSPose;
    inputs.pivotVelRadsPerSec = m_pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotAppliedVolts = m_pivotSim.getInputVoltage();
    inputs.pivotSupplyCurrent = m_pivotSim.getCurrentDrawAmps();

    inputs.drivePoseRads = m_driveSim.getAngularPositionRad();
    inputs.driveVelRadsPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveSim.getInputVoltage();
    inputs.driveSupplyCurrent = m_driveSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_pivotSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void setDriveVolts(double volts) {
    m_driveMotor.setControl(m_driveVoltage.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    setPivotVolts(
        m_pivotFeedback.calculate(m_pivotSim.getAngularPositionRad(), angle.getRadians()));
  }

  @Override
  public void setDriveVelocity(double velMetersPerSec, double ff) {
    m_driveMotor.setControl(
        m_driveVelocityControl.withVelocity(velMetersPerSec).withFeedForward(ff)); // !
  }

  @Override
  public void enableCoast(boolean enabled) {}

  @Override
  public void stop() {
    m_pivotSim.setInputVoltage(0);
    m_driveSim.setInputVoltage(0);
  }
}
