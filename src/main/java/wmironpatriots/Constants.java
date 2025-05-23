// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Time;
import lib.drivers.CanDeviceId;

public class Constants {
  public static Time LOOPTIME = Seconds.of(0.02);

  /** Flags for runtime */
  public static class FLAGS {
    public static final boolean TUNING_MODE = false;
  }

  /** static class containing all device ids */
  public static class MATRIXID {
    // * CANIVORE LOOP
    public static final CANBus CANCHAN = new CANBus("CANchan"); // :3
    public static final CanDeviceId PIGEON = new CanDeviceId(0, CANCHAN.getName());
    public static final CanDeviceId BL_PIVOT = new CanDeviceId(1, CANCHAN.getName());
    public static final CanDeviceId BL_DRIVE = new CanDeviceId(2, CANCHAN.getName());
    public static final CanDeviceId FL_PIVOT = new CanDeviceId(3, CANCHAN.getName());
    public static final CanDeviceId FL_DRIVE = new CanDeviceId(4, CANCHAN.getName());
    public static final CanDeviceId FR_PIVOT = new CanDeviceId(5, CANCHAN.getName());
    public static final CanDeviceId FR_DRIVE = new CanDeviceId(6, CANCHAN.getName());
    public static final CanDeviceId BR_PIVOT = new CanDeviceId(7, CANCHAN.getName());
    public static final CanDeviceId BR_DRIVE = new CanDeviceId(8, CANCHAN.getName());
    public static final CanDeviceId BL_CANCODER = new CanDeviceId(9, CANCHAN.getName());
    public static final CanDeviceId FL_CANCODER = new CanDeviceId(10, CANCHAN.getName());
    public static final CanDeviceId FR_CANCODER = new CanDeviceId(11, CANCHAN.getName());
    public static final CanDeviceId BR_CANCODER = new CanDeviceId(12, CANCHAN.getName());
    public static final CanDeviceId ELEVATOR_PARENT = new CanDeviceId(14, CANCHAN.getName());
    public static final CanDeviceId ELEVATOR_CHILD = new CanDeviceId(15, CANCHAN.getName());

    // * RIO LOOP
    public static final CANBus RIO = new CANBus("rio");
    public static final CanDeviceId TAIL_ROLLER = new CanDeviceId(1, RIO.getName());
    public static final CanDeviceId CHUTE_ROLLER = new CanDeviceId(2, RIO.getName());
    public static final CanDeviceId TAIL_PIVOT = new CanDeviceId(13, RIO.getName());
  }
}
