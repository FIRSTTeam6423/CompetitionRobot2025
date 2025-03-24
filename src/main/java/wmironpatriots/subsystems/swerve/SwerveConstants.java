// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.Robot;

public class SwerveConstants {
  // * CAN Constants
  public static final Frequency ODO_FREQ = Hertz.of(250.0);

  // * Chassis Constants
  public static final Mass MASS = Kilograms.of(54.8847);

  public static final Distance BUMPER_WIDTH = Meters.of(0.0889);
  public static final Distance SIDE_LENGTH = Meters.of(0.7239);
  public static final Distance TRACK_WIDTH = Meters.of(0.596201754);
  public static final Distance RADIUS =
      Meters.of(Math.hypot(TRACK_WIDTH.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0));

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(5.503);
  public static final LinearVelocity MAX_LINEAR_VELOCITY = FeetPerSecond.of(16.5);
  public static final AngularVelocity MAX_ANGULAR_VELOCITY =
      RadiansPerSecond.of(MAX_LINEAR_VELOCITY.in(MetersPerSecond) / RADIUS.in(Meters));

  public static final double LINEAR_P = 3.0;
  public static final double LINEAR_I = 0.0;
  public static final double LINEAR_D = 0.05;

  public static final double ANGULAR_P = 0.0;
  public static final double ANGULAR_I = 0.0;
  public static final double ANGULAR_D = 0.0;

  // * Module Constants
  public static final double PIVOT_REDUCTION = 21.428571428571427;
  public static final double DRIVE_REDUCTION = 6.122448979591837;
  public static final Distance WHEEL_RADIUS_METERS = Meters.of(0.049784);

  static double trackWidth = TRACK_WIDTH.in(Meters);
  public static final Translation2d[] MODULE_LOCS =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, trackWidth / 2.0),
        new Translation2d(trackWidth / 2.0, -trackWidth / 2.0),
        new Translation2d(-trackWidth / 2.0, trackWidth / 2.0),
        new Translation2d(-trackWidth / 2.0, -trackWidth / 2.0),
      };

  public static record ModuleConfig(
      int index,
      int pivotID,
      int driveID,
      int cancoderID,
      double cancoderOffsetRevs,
      boolean pivotInverted,
      boolean driveInverted,
      boolean encoderInverted) {}

  public static final ModuleConfig[] MODULE_CONFIGS =
      new ModuleConfig[] {
        new ModuleConfig(
            0,
            MATRIXID.FL_PIVOT,
            MATRIXID.FL_DRIVE,
            MATRIXID.FL_CANCODER,
            -0.16,
            true,
            false,
            false),
        new ModuleConfig(
            1, MATRIXID.FR_PIVOT, MATRIXID.FR_DRIVE, MATRIXID.FR_CANCODER, 0.01, true, true, false),
        new ModuleConfig(
            2, MATRIXID.BL_PIVOT, MATRIXID.BL_DRIVE, MATRIXID.BL_CANCODER, 0.36, true, true, false),
        new ModuleConfig(
            3,
            MATRIXID.BR_PIVOT,
            MATRIXID.BR_DRIVE,
            MATRIXID.BR_CANCODER,
            -0.26,
            true,
            false,
            false),
      };

  public static CANcoderConfiguration getCancoderConf(double cancoderOffsetRevs, boolean inverted) {
    CANcoderConfiguration conf = new CANcoderConfiguration();
    conf.MagnetSensor.MagnetOffset = cancoderOffsetRevs;
    conf.MagnetSensor.SensorDirection =
        inverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    return conf;
  }

  public static TalonFXConfiguration getPivotConf(int cancoderID, boolean inverted) {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    conf.CurrentLimits.StatorCurrentLimit = 40.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;

    conf.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    conf.TorqueCurrent.PeakReverseTorqueCurrent = 40.0;
    conf.TorqueCurrent.TorqueNeutralDeadband = 0.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    conf.Feedback.SensorToMechanismRatio = 1;
    conf.Feedback.RotorToSensorRatio = PIVOT_REDUCTION;
    conf.Feedback.FeedbackRemoteSensorID = cancoderID;
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    conf.Slot0.kP = 600.0;
    conf.Slot0.kD = 50.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 10;
    conf.Slot0.kS = 0.014;

    conf.ClosedLoopGeneral.ContinuousWrap = true;

    return conf;
  }

  public static TalonFXConfiguration getDriveConf(boolean inverted) {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    conf.Audio.AllowMusicDurDisable = true;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conf.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    conf.CurrentLimits.StatorCurrentLimit = 120.0;
    conf.CurrentLimits.StatorCurrentLimitEnable = true;

    conf.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    conf.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    conf.Feedback.SensorToMechanismRatio =
        DRIVE_REDUCTION / (WHEEL_RADIUS_METERS.in(Meters) * 2 * Math.PI);
    conf.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    conf.Slot0.kP = 35.0;
    conf.Slot0.kD = 0.0;
    conf.Slot0.kA = 0.0;
    conf.Slot0.kV = 0.0;
    conf.Slot0.kS = 5.0;

    conf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    conf.ClosedLoopGeneral.ContinuousWrap = true;

    return conf;
  }

  // * Simulation Constants
  public static final Optional<DriveTrainSimulationConfig> driveTrainSimulationConfig =
      Robot.isSimulation()
          ? Optional.of(
              DriveTrainSimulationConfig.Default()
                  .withGyro(() -> new GyroSimulation(0.0, 0.0))
                  .withSwerveModule(
                      new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60Foc(1),
                          DCMotor.getKrakenX60Foc(1),
                          DRIVE_REDUCTION,
                          PIVOT_REDUCTION,
                          Volts.of(0.1),
                          Volts.of(0.2),
                          WHEEL_RADIUS_METERS,
                          KilogramSquareMeters.of(0.03),
                          1.2))
                  .withTrackLengthTrackWidth(TRACK_WIDTH, TRACK_WIDTH)
                  .withRobotMass(MASS)
                  .withCustomModuleTranslations(MODULE_LOCS))
          : Optional.empty();
}
