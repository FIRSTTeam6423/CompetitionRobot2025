// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.util.motorUtil;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Arrays;
import java.util.Random;

/** A class that's responsible for configuring all CTRe devices */
public class CTReUtil {
  public static CANBus CANchan = new CANBus("CAN-chan"); // primary can bus :3

  private static final Orchestra robotOrchestra = new Orchestra();
  private static TalonFX[] globalTalonArray;

  /** Register Talon to global talon array */
  public static void registerMotor(TalonFX talon) {
    globalTalonArray[globalTalonArray.length] = talon;
  }

  /** Initalizes Talon orchestra */
  public static void initOrchestra() {
    Arrays.stream(globalTalonArray).forEach((t) -> robotOrchestra.addInstrument(t));
  }

  /** Plays song */
  public static void playSong() {
    robotOrchestra.play();
  }

  /** Pause song */
  public static void pauseSong() {
    robotOrchestra.pause();
  }

  /** Stop song */
  public static void stopSong() {
    robotOrchestra.stop();
  }

  /** Loads song of songID */
  public static void loadSong(int songID) {
    var status = robotOrchestra.loadMusic(robotSongArray[songID]);

    if (!status.isOK()) {
      System.out.println("SongID requested is out of bounds");
    }
  }

  /** Requests a random song from song array */
  public static void playRandomSong() {
    loadSong(new Random().nextInt(robotSongArray.length));
  }

  public static final String[] robotSongArray = new String[] {"Test 1", "Test 2"};
}
