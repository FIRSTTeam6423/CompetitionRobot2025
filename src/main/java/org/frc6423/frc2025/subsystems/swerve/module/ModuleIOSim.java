// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import static org.frc6423.frc2025.Constants.KDriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc6423.frc2025.Constants.KDriveConstants.ModuleConfig;

public class ModuleIOSim implements ModuleIO {

  private final DCMotorSim m_pivotSim, m_driveSim;
  private final PIDController m_pivotFeedback, m_driveFeedback;

  public ModuleIOSim(ModuleConfig config) {
    m_pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(2.1314, 0.33291), DCMotor.getKrakenX60(1));
    m_driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(2.1314, 0.33291), DCMotor.getKrakenX60(1));

    m_pivotFeedback = new PIDController(kPivotP, kPivotI, kPivotD);
    m_driveFeedback = new PIDController(kDriveP, kDriveI, kDriveD);

    m_pivotFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
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
  public void periodic() {}

  @Override
  public void setPivotVolts(double volts) {
    // System.out.println(volts);
    m_pivotSim.setInputVoltage(volts);
    m_pivotSim.update(0.02);
  }

  @Override
  public void setDriveVolts(double volts) {
    m_driveSim.setInputVoltage(volts);
    m_driveSim.update(0.02);
  }

  @Override
  public void setPivotAngle(Rotation2d angle) {
    System.out.println(angle.getDegrees());
    setPivotVolts(
        m_pivotFeedback.calculate(m_pivotSim.getAngularPositionRad(), angle.getRadians()));
  }

  @Override
  public void setDriveVelocity(double velMetersPerSec, double ff) {
    setDriveVolts(
        m_driveFeedback.calculate(
            m_driveSim.getAngularVelocityRadPerSec(), velMetersPerSec / kWheelRadius));
  }

  @Override
  public void setPivotCoastMode(boolean enabled) {}

  @Override
  public void setDriveCoastMode(boolean enabled) {}

  @Override
  public void stop() {
    m_pivotSim.setInputVoltage(0);
    m_driveSim.setInputVoltage(0);
  }
}
