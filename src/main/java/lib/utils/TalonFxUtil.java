// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package lib.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Utility class for managing TalonFX motors */
public class TalonFxUtil {
  /**
   * @return a template {@link TalonFXConfiguration}
   */
  public static TalonFXConfiguration getDefaultTalonFxCfg() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Audio.AllowMusicDurDisable = true;
    cfg.Audio.BeepOnBoot = true;
    cfg.Audio.BeepOnConfig = true;

    return cfg;
  }
}
