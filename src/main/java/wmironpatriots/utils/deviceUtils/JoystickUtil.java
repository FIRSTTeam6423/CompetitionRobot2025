package wmironpatriots.utils.deviceUtils;

import edu.wpi.first.math.MathUtil;

public class JoystickUtil {
    /**
     * Takes joystick axis input and modifies it with teleop modifiers
     * - Deadbands by specified value
     * - Raises output to 2
     * 
     * @param value axis input
     * @param deadband joystick deadband
     * @return modified inputs
     */
    public static double applyTeleopModifier(double value, double deadband) {
        value = MathUtil.applyDeadband(value, deadband);
        return Math.copySign(Math.pow(value, 2), value);
    }

    /**
     * Takes joystick axis input and modifies it with teleop modifiers
     * - Deadbands by 0.05
     * - Raises output to 2
     * 
     * @param value axis input
     * @return modified inputs
     */
    public static double applyTeleopModifier(double value) {
        return applyTeleopModifier(value);
    }
}
