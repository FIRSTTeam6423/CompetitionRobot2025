// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.tail.Tail;

/** A class */
public class Constants {
  public static final double DT_TIME = 0.02;

  public static final CANBus CANIVORE = new CANBus("CANchan"); // 3:

  public static final Optional<DriveTrainSimulationConfig> SWERVE_SIM_CONFIG =
      Robot.isSimulation()
          ? Optional.of(
              DriveTrainSimulationConfig.Default()
                  .withGyro(() -> new GyroSimulation(0.01, 0.01))
                  .withSwerveModule(
                      new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60Foc(1),
                          DCMotor.getKrakenX60Foc(1),
                          Module.DRIVE_REDUCTION,
                          Module.PIVOT_REDUCTION,
                          Volts.of(0.01),
                          Volts.of(0.01),
                          Meter.of(Module.WHEEL_RADIUS_METERS),
                          KilogramSquareMeters.of(0.03),
                          1.5))
                  .withTrackLengthTrackWidth(
                      Meters.of(Swerve.TRACK_WIDTH_METERS),
                      Meters.of(Swerve.TRACK_WIDTH_METERS)) // TODO add back bumper widt
                  .withBumperSize(
                      Meters.of(Swerve.BUMPER_WIDTH_METER), Meters.of(Swerve.BUMPER_WIDTH_METER))
                  .withRobotMass(Kilograms.of(Swerve.MASS_KG))
                  .withCustomModuleTranslations(Swerve.MODULE_LOCS))
          : Optional.empty();

  public static enum LevelTarget {
    L1(Elevator.POSE_L1, Tail.POSE_IN_ANGLE),
    L2(Elevator.POSE_L2, Tail.POSE_IN_ANGLE),
    L3(Elevator.POSE_L3, Tail.POSE_IN_ANGLE),
    L4(Elevator.POSE_L4, Tail.POSE_OUT_ANGLE);
    public final double elevatorPoseRevs, tailPoseRads;

    private LevelTarget(double elevatorPoseRevs, double tailPoseRads) {
      this.elevatorPoseRevs = elevatorPoseRevs;
      this.tailPoseRads = tailPoseRads;
    }
  }

  public static enum BranchTarget {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P
  }
}
