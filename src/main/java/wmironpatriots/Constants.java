// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.CANBus;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.tail.Tail;

/** A class */
public class Constants {
  public static final double kTickSpeed = 0.02;

  public static final CANBus CANIVORE = new CANBus("CANchan"); // 3:

  public static enum ReefTarget {
    L1(Elevator.POSE_L1, Tail.POSE_LNONFOUR_RADS),
    L2(Elevator.POSE_L2, Tail.POSE_LNONFOUR_RADS),
    L3(Elevator.POSE_L3, Tail.POSE_LNONFOUR_RADS),
    L4(Elevator.POSE_L4, Tail.POSE_L4_RADS);
    public final double elevatorPoseRevs, tailPoseRads;

    private ReefTarget(double elevatorPoseRevs, double tailPoseRads) {
      this.elevatorPoseRevs = elevatorPoseRevs;
      this.tailPoseRads = tailPoseRads;
    }
  }
}
