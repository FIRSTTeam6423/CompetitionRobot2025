// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.module.Module;

public class Constants {
  public static double TICK_SPEED = 0.02;

  public static final class MAPPLESIM {
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
                    .withCustomModuleTranslations(Swerve.MODULE_LOCS))
            : Optional.empty();
  }

  /** Flags for runtime */
  public static class FLAGS {
    public static final boolean TUNING_MODE = true;
    public static final boolean SUPERSTRUCTURE_ENABLED = false;
  }

  /** static class containing all device ids */
  public static class MATRIXID {
    // * CANIVORE LOOP
    public static final CANBus CANCHAN = new CANBus("CANchan"); // :3
    public static final int PIGEON = 0;
    public static final int BL_PIVOT = 1;
    public static final int BL_DRIVE = 2;
    public static final int FL_PIVOT = 3;
    public static final int FL_DRIVE = 4;
    public static final int FR_PIVOT = 5;
    public static final int FR_DRIVE = 6;
    public static final int BR_PIVOT = 7;
    public static final int BR_DRIVE = 8;
    public static final int BL_CANCODER = 9;
    public static final int FL_CANCODER = 10;
    public static final int FR_CANCODER = 11;
    public static final int BR_CANCODER = 12;
    public static final int ELEVATOR_PARENT = 14;
    public static final int ELEVATOR_CHILD = 15;

    // * RIO LOOP
    public static final CANBus RIO = new CANBus("rio");
    public static final int TAIL_ROLLER = 1;
    public static final int CHUTE_ROLLER = 2;
    public static final int TAIL_PIVOT = 13;
  }
}
