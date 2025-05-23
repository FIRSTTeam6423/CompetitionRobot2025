// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/** Represents drivetrain characterization */
public class SwerveConstants {
  /**
   * Represents the constants of a single module
   *
   * @param index Module identifier
   * @param pivotId Pivot motor CAN ID
   * @param driveId Drive motor CAN ID
   * @param encoderId Encoder CAN/PWM ID
   * @param encoderOffsetRevs Encoder measurement offset in Revs
   * @param pivotInverted Is pivot motor inverted?
   * @param driveInverted Is drive motor inverted?
   */
  public static record ModuleConfig(
      int index,
      int pivotId,
      int driveId,
      int encoderId,
      double encoderOffsetRevs,
      boolean pivotInverted,
      boolean driveInverted) {}

  public static final Mass MASS = Kilograms.of(0);
  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0);

  public static final Distance BUMPER_WIDTH = Meters.of(0);
  public static final Distance TRACK_WIDTH = Meters.of(0);
  public static final Distance RADIUS = Meters.of(0);

  public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(0);
  public static final AngularVelocity MAX_ANGULAR_RATE =
      RadiansPerSecond.of(MAX_LINEAR_SPEED.in(MetersPerSecond) / RADIUS.in(Meters));

  public static final ProfiledPIDController LINEAR_VELOCITY_FEEDBACK_CONTROLLER =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  public static final ProfiledPIDController ANGULAR_RATE_FEEDBACK_CONTROLLER =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  public static final Distance WHEEL_RADIUS = Meters.of(0.0);

  public static final Translation2d[] MODULE_OFFSETS = new Translation2d[4];

  public static final ModuleConfig[] MODULE_CONFIGS = new ModuleConfig[4];

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_OFFSETS);
}
