// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.util.swerveUtil;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConfig {
  public ModuleConfig[] kModuleConfigs;

  public Translation2d[] kModuleLocs;
  public double kTrackWidthY;
  public double kTrackWidthX;
  public double kBumperWidth;
  public double kBumperLength;
  public double kMassKg;

  public double kMaxLinearVel;
  public double kMaxLinearAccel;
  public double kMaxAngularVel;

  public int kGyroID;

  /**
   * Creates a new Swerve config
   *
   * @param moduleConfigs
   * @param moduleLocs
   * @param trackWidthY Distance between center of wheels (inches)
   * @param trackWidthX Distance between center of wheels (inches)
   * @param bumperWidth Bumper thickness
   * @param bumperLength Bumper length
   * @param mass mass in Kg
   * @param maxLinearVel max linear velocity in mps
   * @param maxLinearAccel max linear accel in mps^2
   * @param gyroID put 100 if NavX
   */
  public SwerveConfig(
      ModuleConfig[] moduleConfigs,
      Translation2d[] moduleLocs,
      double trackWidthY,
      double trackWidthX,
      double bumperWidth,
      double bumperLength,
      double mass,
      double maxLinearVel,
      double maxLinearAccel,
      int gyroID) {
    this.kModuleConfigs = moduleConfigs;

    this.kModuleLocs = moduleLocs;
    this.kTrackWidthY = trackWidthY;
    this.kTrackWidthX = trackWidthX;
    this.kBumperWidth = bumperWidth;
    this.kBumperLength = bumperLength;
    this.kMassKg = mass;

    this.kMaxLinearVel = maxLinearVel;
    this.kMaxLinearAccel = maxLinearAccel;

    double driveBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);
    this.kMaxAngularVel = maxLinearVel / driveBaseRadius;

    this.kGyroID = gyroID;
  }
}
