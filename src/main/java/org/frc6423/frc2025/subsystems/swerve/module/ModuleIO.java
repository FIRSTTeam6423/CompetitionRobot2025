// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public class ModuleIOInputs {
    public boolean pivotEnabled = false;
    public boolean driveEnabled = false;

    public Rotation2d pivotABSPose = new Rotation2d();
    public Rotation2d pivotPose = new Rotation2d();
    public double pivotVelRadsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrent = 0.0;
    public double pivotTorqueCurrent = 0.0;

    public double drivePoseRads = 0.0;
    public double driveVelRadsPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrent = 0.0;
    public double driveTorqueCurrent = 0.0;
  }

  /** Updates logged akit values */
  public void updateInputs(ModuleIOInputs inputs);

  /** Update module hardware */
  public void periodic();

  /** Set Pivot motor voltage */
  public void setPivotVolts(double volts);

  /** Set Drive motor voltage */
  public void setDriveVolts(double volts);

  /** Set Module angle goal */
  public void setPivotAngle(Rotation2d angle);

  /** Set Module velocity goal */
  public void setDriveVelocity(double velMetersPerSec, double ff);

  /** Stop all motor inputs */
  public default void stop() {
    setPivotVolts(0);
    setDriveVolts(0);
  };
}