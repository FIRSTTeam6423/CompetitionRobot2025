// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package lib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

// TODO
/** Generalized characterization for a swerve chassis */
public abstract class SwerveConfig {
  /**
   * @return {@link Mass} representing the chassis mass
   */
  public abstract Mass getMass();

  /**
   * @return {@link Distance} representing the avg thickness of the bumpers
   */
  public abstract Distance getBumperWidth();

  /**
   * @return {@link Distance} representing the distance between the center of wheels on each side
   */
  public abstract Distance getTrackWidth();

  /**
   * @return {@link Distance} representing the diagonal distance between the centers of wheels
   */
  public Distance getRadius() {
    return Meters.of(Math.hypot(getTrackWidth().in(Meters), getTrackWidth().in(Meters)));
  }

  /**
   * @return {@link MomentOfInertia} representing the MOI of the chassis
   */
  public abstract MomentOfInertia getMOI();

  /**
   * @return {@link LinearVelocity} representing the max linear speed of the chassis
   */
  public abstract LinearVelocity getMaxLinearSpeed();

  /**
   * @return {@link AngularVelocity} representing the max angular speed of the chassis
   */
  public AngularVelocity getMaxAngularSpeed() {
    return RadiansPerSecond.of(getMaxLinearSpeed().in(MetersPerSecond) / getRadius().in(Meters));
  }

  public abstract ProfiledPIDController getLinearVelocityFeedback();

  public abstract ProfiledPIDController getAngularVelocityFeedback();

  /**
   * @return {@link Translation2d} array representing the module positions relative to the center of
   *     chassis
   */
  public abstract Translation2d[] getModuleLocs();

  /**
   * @return {@link SwerveDriveKinematics} WPIlib inverse kinematics class for chassis
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleLocs());
  }
}