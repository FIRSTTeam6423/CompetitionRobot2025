package wmironpatriots.subsystems.superstructure.elevator;

import lib.LoggedSubsystem;

public class Elevator implements LoggedSubsystem {
  private static Elevator instance;

  /**
   * @return elevator instance (creates instance if null)
   */
  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }

    return instance;
  }

  public Elevator() {}

  @Override
  public void periodic() {}
}