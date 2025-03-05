// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.util.ntUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class TunableDouble implements DoubleSupplier {
  private final String key;
  private double initValue;

  public TunableDouble(String key, double initValue) {
    SmartDashboard.putNumber(key, initValue);

    this.key = key;
    this.initValue = initValue;
  }

  /** shorter version of {@link getAsDouble} */
  public double get() {
    return this.getAsDouble();
  }

  @Override
  public double getAsDouble() {
    return SmartDashboard.getNumber(key, initValue);
  }
}
