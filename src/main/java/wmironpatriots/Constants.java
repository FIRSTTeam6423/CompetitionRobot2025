// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.CANBus;

import lib.drivers.CanDevice;

public class Constants {
  public static double LOOPTIME = 0.02;

  /** Flags for runtime */
  public static class FLAGS {
    public static final boolean TUNING_MODE = false;
  }

  /** static class containing all device ids */
  public static class MATRIXID {
    // * CANIVORE LOOP
    public static final CANBus CANCHAN = new CANBus("CANchan"); // :3
    public static final CanDevice PIGEON = new CanDevice(0, CANCHAN.getName());
    public static final CanDevice BL_PIVOT = new CanDevice(1, CANCHAN.getName());
    public static final CanDevice BL_DRIVE = new CanDevice(2, CANCHAN.getName());
    public static final CanDevice FL_PIVOT = new CanDevice(3, CANCHAN.getName());
    public static final CanDevice FL_DRIVE = new CanDevice(4, CANCHAN.getName());
    public static final CanDevice FR_PIVOT = new CanDevice(5, CANCHAN.getName());
    public static final CanDevice FR_DRIVE = new CanDevice(6, CANCHAN.getName());
    public static final CanDevice BR_PIVOT = new CanDevice(7, CANCHAN.getName());
    public static final CanDevice BR_DRIVE = new CanDevice(8, CANCHAN.getName());
    public static final CanDevice BL_CANCODER = new CanDevice(9, CANCHAN.getName());
    public static final CanDevice FL_CANCODER = new CanDevice(10, CANCHAN.getName());
    public static final CanDevice FR_CANCODER = new CanDevice(11, CANCHAN.getName());
    public static final CanDevice BR_CANCODER = new CanDevice(12, CANCHAN.getName());
    public static final CanDevice ELEVATOR_PARENT = new CanDevice(14, CANCHAN.getName());
    public static final CanDevice ELEVATOR_CHILD = new CanDevice(15, CANCHAN.getName());

    // * RIO LOOP
    public static final CANBus RIO = new CANBus("rio");
    public static final CanDevice TAIL_ROLLER = new CanDevice(1, RIO.getName());
    public static final CanDevice CHUTE_ROLLER = new CanDevice(2, RIO.getName());
    public static final CanDevice TAIL_PIVOT = new CanDevice(13, RIO.getName());
  }
}
