package wmironpatriots.subsystems.superstructure;

public class Superstructure {

  private static Superstructure instance;

  /**
   * @return superstructure instance (creates instance if null)
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  private Superstructure() {}
}
