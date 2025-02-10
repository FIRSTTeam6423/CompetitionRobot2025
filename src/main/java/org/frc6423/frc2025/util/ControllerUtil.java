// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.util;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public class ControllerUtil {

  /** Applies a deadband of 0.098 */
  public static DoubleSupplier applyDeadband(DoubleSupplier axis, boolean invert) {
    return () -> MathUtil.applyDeadband(axis.getAsDouble(), 0.098) * (invert ? -1 : 1);
  }
}
