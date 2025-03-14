// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.utils.deviceUtils;

import edu.wpi.first.math.MathUtil;

public class JoystickUtil {
  /**
   * Takes joystick axis input and modifies it with teleop modifiers - Deadbands by specified value
   * - Raises output to 2
   *
   * @param value axis input
   * @param deadband joystick deadband
   * @return modified inputs
   */
  public static double applyTeleopModifier(double value, double deadband) {
    var bah = MathUtil.applyDeadband(value, deadband);
    return Math.copySign(Math.pow(bah, 2), bah);
  }

  /**
   * Takes joystick axis input and modifies it with teleop modifiers - Deadbands by 0.05 - Raises
   * output to 2
   *
   * @param value axis input
   * @return modified inputs
   */
  public static double applyTeleopModifier(double value) {
    return applyTeleopModifier(value);
  }
}
