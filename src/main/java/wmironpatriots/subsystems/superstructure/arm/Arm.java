package wmironpatriots.subsystems.superstructure.arm;

import lib.LoggedSubsystem;

public class Arm implements LoggedSubsystem {
  private static Arm instance;

  /**
   * @return arm instance (creates instance if null)
   */
  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }

    return instance;
  }

  private Arm() {}  
}
