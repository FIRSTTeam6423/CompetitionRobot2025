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
import monologue.Annotations.Log;
import monologue.Logged;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public abstract class Module implements Logged {
  /** LOGGED VALUES */
  @Log public boolean pivotOk = false;

  @Log public boolean driveOk = false;

  @Log public double pivotABSPoseRads;
  @Log public double pivotPoseRads;
  @Log public double pivotVelRadsPerSec;
  @Log public double pivotAppliedVolts;
  @Log public double pivotSupplyCurrent;
  @Log public double pivotTorqueCurrent;

  @Log public double drivePoseMeters;
  @Log public double driveVelMPS;
  @Log public double driveAppliedVolts;
  @Log public double driveSupplyCurrent;
  @Log public double driveTorqueCurrent;

  private final SimpleMotorFeedforward driveff;

  protected ModuleConfig config;

  public Module(ModuleConfig config) {
    this.config = config;

    driveff = new SimpleMotorFeedforward(0.088468, 2.1314, 0.33291);
  }

  /** Periodically ran logic */
  public void periodic() {}

  /** Run setpoint module state */
  public SwerveModuleState runSetpoint(SwerveModuleState setpoint) {
    setpoint.optimize(getModuleAngle());
    setpoint.speedMetersPerSecond *= Math.cos(setpoint.angle.minus(getModuleAngle()).getRadians());

    double speedMPS = setpoint.speedMetersPerSecond;
    runPivotPose(setpoint.angle.getRadians());
    runDriveVel(speedMPS, driveff.calculate(speedMPS) + DCMotor.getKrakenX60(1).KtNMPerAmp);
    return setpoint;
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

    setpointState.optimize(getModuleAngle());
    setpointState.speedMetersPerSecond *= Math.cos(setpointState.angle.minus(angle).getRadians());

    double speedMPS = setpointState.speedMetersPerSecond;
    runPivotPose(setpointState.angle.getRadians());
    runDriveVolts(speedMPS);
    return setpointState;
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
    return Rotation2d.fromRadians(pivotPoseRads);
  }

  /** Returns module field pose */
  public SwerveModulePosition getModulePose() {
    return new SwerveModulePosition(drivePoseMeters, getModuleAngle());
  }

  /** Returns current module pivot & drive motor state */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(drivePoseMeters, getModuleAngle());
  }

  /** HARDWARE METHODS */
  /** Run Pivot motor with pose request */
  protected abstract void runPivotPose(double poseRads);

  /** Run Pivot motor with voltage request */
  protected abstract void runPivotVolts(double volts);

  /** Run Drive motor with foc voltage request */
  protected void runDriveVolts(double volts) {
    runDriveVolts(volts, true);
  }

  /** Run Drive motor with voltage request */
  protected abstract void runDriveVolts(double volts, boolean focEnabled);

  /** Run Drive motor with velocity request */
  protected abstract void runDriveVel(double velMPS, double torqueff);

  /** Stop all motor input */
  protected abstract void stopMotors();

  /** Enable or disable motor coasting */
  protected abstract void motorCoasting(boolean enabled);
}
