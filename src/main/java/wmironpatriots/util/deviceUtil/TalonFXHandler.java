// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.deviceUtil;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Arrays;
import java.util.Random;

/** A class for handling all CTRe devices */
public class TalonFXHandler {
  public static final CANBus CANchan = new CANBus("CANchan"); // Main CANbus :3
  private TalonFX[] m_globalTalonArray;
  private final Orchestra m_orchestra = new Orchestra();

  /** Register Talon to global talon array */
  public void registerTalon(TalonFX talon) {
    m_globalTalonArray[m_globalTalonArray.length] = talon;
  }

  /** enable orchestra mode on all talons */
  public void configureOrchestra() {
    AudioConfigs config =
        new AudioConfigs()
            .withAllowMusicDurDisable(true)
            .withBeepOnBoot(true)
            .withBeepOnConfig(true);

    Arrays.stream(m_globalTalonArray)
        .forEach(
            (m) -> {
              m.getConfigurator().apply(config);
              m_orchestra.addInstrument(m);
            });
  }

  /** Plays random song from list */
  public void playRandom() {
    int randomIndex = new Random().nextInt(0, OrchestraSongs.values().length);
    loadSong(OrchestraSongs.values()[randomIndex]);
  }

  /** Load specific orchestra song */
  public void loadSong(OrchestraSongs song) {
    m_orchestra.loadMusic(song.path);
  }

  /** Play song */
  public void playSong() {
    m_orchestra.play();
  }

  /** Pause song */
  public void pauseSong() {
    m_orchestra.pause();
  }

  /** Stop song */
  public void stopSong() {
    m_orchestra.stop();
  }

  public static enum OrchestraSongs {
    LoveIsAnOpenDoor("chirps/1.chrp"),
    Miku("chirps/2.chrp");

    public final String path;

    private OrchestraSongs(String path) {
      this.path = path;
    }
  }
}
