package wmironpatriots.subsystems.superstructure.climber;

import lib.LoggedSubsystem;

public class Climber implements LoggedSubsystem {
  private static Climber instance;

  /**
   * @return climber instance (creates instance if null)
   */
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }

    return instance;
  }

  private Climber() {}
}
