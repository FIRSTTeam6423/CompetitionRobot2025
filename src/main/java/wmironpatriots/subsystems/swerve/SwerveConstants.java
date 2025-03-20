// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.Module.ModuleConfig;

// TODO clean this
public class SwerveConstants {
  public static final double MASS_KG = 54.8847;
  public static final double MOI = 5.503;
  public static final double SIDE_LENGTH_METERS = 0.7239;
  public static final double BUMPER_WIDTH_METER = 0.0889;
  public static final double TRACK_WIDTH_METERS = 0.596201754;
  public static final double RADIUS_METERS =
      Math.hypot(TRACK_WIDTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d[] MODULE_LOCS =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
        new Translation2d(TRACK_WIDTH_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-TRACK_WIDTH_METERS / 2.0, TRACK_WIDTH_METERS / 2.0),
        new Translation2d(-TRACK_WIDTH_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0),
      };
  //   new Translation2d[] {
  //     new Translation2d(-0.381, 0.381),
  //     new Translation2d(0.381, 0.381),
  //     new Translation2d(-0.381, -0.381),
  //     new Translation2d(0.381, -0.381)
  //   };

  public static final Rotation2d ALLIANCE_ROTATION =
      Rotation2d.fromRotations(
          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0.5);

  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / RADIUS_METERS;

  public static final double LINEAR_P = 3.0; // 4.5;
  public static final double LINEAR_I = 0.0;
  public static final double LINEAR_D = 0.05;

  public static final double ANGULAR_P = 0.0;
  public static final double ANGULAR_I = 0.0;
  public static final double ANGULAR_D = 0.0;

  public static final double ODO_FREQ = 250.0;

  public static final ModuleConfig[] MODULE_CONFIGS =
      new ModuleConfig[] {
        new ModuleConfig(
            0, MATRIXID.FL_PIVOT, MATRIXID.FL_DRIVE, MATRIXID.FL_CANCODER, 0.29, true, true, false),
        new ModuleConfig(
            1, MATRIXID.FR_PIVOT, MATRIXID.FR_DRIVE, MATRIXID.FR_CANCODER, 0.01, true, true, false),
        new ModuleConfig(
            2, MATRIXID.BL_PIVOT, MATRIXID.BL_DRIVE, MATRIXID.BL_CANCODER, 0.36, true, true, false),
        new ModuleConfig(
            3, MATRIXID.BR_PIVOT, MATRIXID.BR_DRIVE, MATRIXID.BR_CANCODER, 0.07, true, true, false),
      };

  // public static final ModuleConfig[] MODULE_CONFIGS =
  //       new ModuleConfig[] {
  //         new ModuleConfig(
  //             3,
  //             MATRIXID.BR_PIVOT,
  //             MATRIXID.BR_DRIVE,
  //             MATRIXID.BR_CANCODER,
  //             -0.07,
  //             // 0.424, // -0.016,
  //             false,
  //             true,
  //             true),
  //         new ModuleConfig(
  //             1,
  //             MATRIXID.FR_PIVOT,
  //             MATRIXID.FR_DRIVE,
  //             MATRIXID.FR_CANCODER,
  //             -0.01,
  //             false,
  //             true,
  //             true),
  //         new ModuleConfig(
  //             2,
  //             MATRIXID.BL_PIVOT,
  //             MATRIXID.BL_DRIVE,
  //             MATRIXID.BL_CANCODER,
  //             -0.36,
  //             false,
  //             true,
  //             true),
  //         new ModuleConfig(
  //             0,
  //             MATRIXID.FL_PIVOT,
  //             MATRIXID.FL_DRIVE,
  //             MATRIXID.FL_CANCODER,
  //             -0.29,
  //             false,
  //             true,
  //             true),
  //       };

  public static final Optional<DriveTrainSimulationConfig> driveTrainSimulationConfig =
      Robot.isSimulation()
          ? Optional.of(
              DriveTrainSimulationConfig.Default()
                  // Specify gyro type (for realistic gyro drifting and error simulation). i dont
                  // wanna
                  // deal w too much error lol
                  .withGyro(() -> new GyroSimulation(0.0, 0.0))
                  // Specify swerve module (for realistic swerve dynamics)
                  .withSwerveModule(
                      new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60Foc(1),
                          DCMotor.getKrakenX60Foc(1),
                          Module.DRIVE_REDUCTION,
                          Module.PIVOT_REDUCTION,
                          Volts.of(0.1),
                          Volts.of(0.2),
                          Meter.of(Module.WHEEL_RADIUS_METERS),
                          KilogramSquareMeters.of(0.03),
                          1.2))
                  // Configures the track length and track width (spacing between swerve modules)
                  // .withTrackLengthTrackWidth(
                  //     Meter.of(Swerve.TRACK_WIDTH_METERS), Meter.of(Swerve.TRACK_WIDTH_METERS))
                  // // Configures the bumper size (dimensions of the robot bumper)
                  // .withBumperSize(Inches.of(30), Inches.of(30))
                  // .withRobotMass(Kilograms.of(Swerve.MASS_KG))
                  .withCustomModuleTranslations(MODULE_LOCS))
          : Optional.empty();
}
