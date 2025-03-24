package wmironpatriots;

import edu.wpi.first.wpilibj.RobotBase;

// ! DO NOT modify this file unless you know what you're doing
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
