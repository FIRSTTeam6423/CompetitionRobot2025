// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import monologue.Logged;
import monologue.Annotations.Log;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public abstract class Module implements Logged {
  /** LOGGED VALUES */
  @Log protected boolean pivotOk = false;

  @Log protected boolean driveOk = false;

  @Log protected Rotation2d pivotABSPose = new Rotation2d();
  @Log protected Rotation2d pivotPose = new Rotation2d();
  @Log protected double pivotVelRadsPerSec;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotSupplyCurrent;
  @Log protected double pivotTorqueCurrent;

  @Log protected double drivePoseRads;
  @Log protected double driveVelRadsPerSec;
  @Log protected double driveAppliedVolts;
  @Log protected double driveSupplyCurrent;
  @Log protected double driveTorqueCurrent;

  /** VARIABLES */
  private final VoltageOut m_motorVoltsOutReq =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final VelocityVoltage m_motorVelocityOutReq =
      new VelocityVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final PositionVoltage m_motorPositionOutReq =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(0.14, 0.134);

  private ModuleConfig m_config;

  public Module(ModuleConfig config) {
    m_config = config;
  }

  /** Periodically ran logic */
  public void periodic() {}

  /** Run setpoint module state */
  public SwerveModuleState runSetpoint(SwerveModuleState setpoint) {
    setpoint.optimize(pivotABSPose);
    setpoint.speedMetersPerSecond *= Math.cos(setpoint.angle.minus(pivotABSPose).getRadians());

    double speedMPS = setpoint.speedMetersPerSecond;
    runPivotControl(m_motorPositionOutReq.withPosition(setpoint.angle.getRotations()));
    runDriveControl(
        m_motorVelocityOutReq
            .withVelocity(speedMPS)
            .withFeedForward(
                m_driveFF.calculate(
                    (speedMPS / m_config.kWheelRadiusMeters)
                        * (m_config.kDriveReduction / DCMotor.getKrakenX60Foc(1).KtNMPerAmp))));

    return new SwerveModuleState();
  }

  /** Run setpoint module state with setpoint wheel torque (torque based ff) */
  public SwerveModuleState runSetpoint(SwerveModuleState setpointState, double driveTorqueNm) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runSetpoint'");
  }

  /** Run setpoint module state but with open loop drive control */
  public SwerveModuleState runSetpointOpenloop(
      double voltage, Rotation2d angle, boolean FOCEnabled) {
    SwerveModuleState setpointState = new SwerveModuleState(voltage, angle);

    setpointState.optimize(pivotABSPose);
    setpointState.speedMetersPerSecond *=
        Math.cos(setpointState.angle.minus(pivotABSPose).getRadians());

    double speedMPS = setpointState.speedMetersPerSecond;
    runPivotControl(m_motorPositionOutReq.withPosition(setpointState.angle.getRotations()));
    runDriveControl(m_motorVoltsOutReq.withOutput(speedMPS).withEnableFOC(FOCEnabled)); // !
    return new SwerveModuleState();
  }

  /** Stops module */
  public void stop() {
    stopMotors();
  }

  /** Enable coast mode to move robot easier */
  public void moduleCoasting(boolean enabled) {
    motorCoasting(enabled);
  }

  /** Returns module angle */
  public Rotation2d getModuleAngle() {
    return pivotPose;
  }

  /** Converts drive motor speed to mps */
  public double getModuleSpeedMPS() {
    return driveVelRadsPerSec * m_config.kWheelRadiusMeters;
  }

  /** Returns module field pose */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(drivePoseRads * m_config.kWheelRadiusMeters, pivotPose);
  }

  /** Returns current module pivot & drive motor state */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getModuleSpeedMPS(), pivotPose);
  }

  /** HARDWARE METHODS */
  /** Run Pivot motor with control request */
  protected abstract void runPivotControl(ControlRequest request);

  /** Run Drive motor with control request */
  protected abstract void runDriveControl(ControlRequest request);

  /** Stop all motor input */
  protected abstract void stopMotors();

  /** Enable or disable motor coasting */
  protected abstract void motorCoasting(boolean enabled);
}
