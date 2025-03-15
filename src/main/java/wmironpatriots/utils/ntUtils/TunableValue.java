package wmironpatriots.utils.ntUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import wmironpatriots.Constants.FLAGS;

/**
 * Slightly modified version of 6328's 2022 TunableNumber class
 * 
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableValue {
  private static final String tableKey = "TunableNumbers";

  private String key;
  private double defaultValue;
  private double lastHasChangedValue = defaultValue;

  /**
   * Create a new TunableNumber
   * 
   * @param dashboardKey Key on dashboard
   */
  public TunableValue(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   * 
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableValue(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   * 
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   * 
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    SmartDashboard.setDefaultNumber(key, defaultValue);
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   * 
   * @return The current value
   */
  public double get() {
    return FLAGS.TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue)
        : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   * 
   * @return True if the number has changed since the last time this method was called, false
   *         otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}