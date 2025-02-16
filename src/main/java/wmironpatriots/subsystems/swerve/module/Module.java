// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import wmironpatriots.Robot;
import wmironpatriots.util.swerveUtil.ModuleConfig;
import wmironpatriots.util.swerveUtil.ModuleConfig.moduleType;

import org.frc6423.frc2025.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_IO;

  private final int m_index;
  private final ModuleConfig m_config;

  private final ModuleIOInputsAutoLogged m_inputs;

  private SimpleMotorFeedforward m_driveff;

  public Module(ModuleConfig config) {
    m_IO =
        (Robot.isReal())
            ? (config.kModuletype == moduleType.TALONFX)
                ? new ModuleIOTalonFX(config)
                : new ModuleIOSpark(config)
            : new ModuleIOSim(config);

    m_index = config.kIndex;
    this.m_config = config;

    m_inputs = new ModuleIOInputsAutoLogged();

    m_driveff = new SimpleMotorFeedforward(0.14, 0.134);
  }

  /** Update auto logged inputs */
  public void updateInputs() {
    m_IO.updateInputs(m_inputs);
  }

  /** Periodically ran logic */
  public void periodic() {
    Logger.processInputs("Swerve/Module" + getModuleIndex(), m_inputs);
  }

  /** Run SwerveModuleState setpoint */
  public SwerveModuleState runSetpoint(SwerveModuleState setpointState) {
    setpointState.optimize(getPivotAngle());
    setpointState.speedMetersPerSecond *=
        Math.cos(setpointState.angle.minus(getPivotAngle()).getRadians());

    double speedMPS = setpointState.speedMetersPerSecond;
    m_IO.setPivotAngle(setpointState.angle);
    m_IO.setDriveVelocity(
        speedMPS,
        m_driveff.calculate(
            (speedMPS / m_config.kWheelRadiusMeters)
                * (m_config.kDriveReduction
                    / DCMotor.getKrakenX60Foc(1)
                        .KtNMPerAmp))); // m_driveff.calculate(speedMPS/m_config.kWheelRadiusMeters)
    // *
    // m_config.kWheelRadiusMeters); // !
    return setpointState;
  }

  /** Run SwerveModuleState setpoint with setpoint wheel torque (torque-based ff) */
  public SwerveModuleState runSetpoint(SwerveModuleState setpointState, double driveTorqueNm) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runSetpoint'");
  }

  /** Runs SwerveModuleState setpoint but runs drive in open loop mode */
  public SwerveModuleState runSetpointOpenloop(
      double voltage, Rotation2d angle, boolean FOCEnabled) {
    SwerveModuleState setpointState = new SwerveModuleState(voltage, angle);

    setpointState.optimize(getPivotAngle());
    setpointState.speedMetersPerSecond *=
        Math.cos(setpointState.angle.minus(getPivotAngle()).getRadians());

    double speedMPS = setpointState.speedMetersPerSecond;
    m_IO.setPivotAngle(setpointState.angle);
    m_IO.runPivotVolts(speedMPS, FOCEnabled); // !
    return new SwerveModuleState();
  }

  /** Set pivot angle setpoint */
  public void setPivotAngle(Rotation2d desiredAngle) {
    m_IO.setPivotAngle(desiredAngle);
  }

  /** Set drive torque current setpoint */
  public void runDriveCurrent(double currentAmps) {
    m_IO.setDriveTorque(currentAmps);
  }

  /** Enable module coasting */
  public void enableCoast(boolean enabled) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'enableCoast'");
  }

  /** Stop all motor input */
  public void stop() {
    m_IO.stop();
  }

  /** Get Module index */
  public int getModuleIndex() {
    return m_index;
  }

  /** Get Module config */
  public ModuleConfig getModuleConfig() {
    return m_config;
  }

  /** Gets Drive Speed in MPS */
  public double getVelMetersPerSec() {
    return m_inputs.driveVelRadsPerSec * m_config.kWheelRadiusMeters;
  }

  /** returns current module angle */
  public Rotation2d getPivotAngle() {
    return m_inputs.pivotPose;
  }

  /** Returns drive pose in meters */
  public double getPoseMeters() {
    return m_inputs.drivePoseRads * m_config.kWheelRadiusMeters;
  }

  /** Returns swerve field pose */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(getPoseMeters(), getPivotAngle());
  }

  /** Returns Module state */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelMetersPerSec(), getPivotAngle());
  }
}
