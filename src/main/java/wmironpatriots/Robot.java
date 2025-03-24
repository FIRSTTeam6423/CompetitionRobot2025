package wmironpatriots;

import lib.LoggedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends LoggedRobot {
  public Robot() {
    super(Constants.TICK_SPEED);
  }

  @Override
  public Command getAuton() {
    return Commands.run(() -> {}); // ! PLACEHOLDER
  }
}
