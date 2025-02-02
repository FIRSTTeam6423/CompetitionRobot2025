// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.util;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

public class ControllerUtil {
  public static double applyDeadband(DoubleSupplier axis) {
    return MathUtil.applyDeadband(axis.getAsDouble(), 0.02);
  }
}
