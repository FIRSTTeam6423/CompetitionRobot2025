// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.LoggedCommandRobot;
import monologue.Monologue;

public class Robot extends LoggedCommandRobot {
  private final CommandXboxController driver, operator;

  private final Alert browningOut;

  public Robot() {
    // * SYSTEMS INIT
    // Shuts up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // logs build data to the datalog
    final String meta = "/BuildData/";
    Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
    Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    Monologue.log(meta + "Version", BuildConstants.VERSION);
    Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
    Monologue.log(meta + "GitDirty", String.valueOf(BuildConstants.DIRTY));
    Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
    Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
    Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);

    // DO NOT REMOVE!
    // Signal Logger is set to auto enable when connected to FMS, causing massive delay
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // Sets up alerts
    browningOut = new Alert("Browning Out!", AlertType.kWarning);
    new Trigger(() -> RobotController.isBrownedOut())
        .onTrue(Commands.run(() -> browningOut.set(true)));

    // Log dashboard inforation periodically
    addPeriodic(
        () -> {
          SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
          SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        },
        0.1);

    // * HARDWARE INIT
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);
  }

  /** Command for driver controller rumble */
  private Command rumbleDriver(double value) {
    return Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value));
  }

  /** Command for driver controller rumble */
  private Command rumbleOperator(double value) {
    return Commands.run(() -> operator.setRumble(RumbleType.kBothRumble, value));
  }

  @Override
  protected Command getAutonCommand() {
    // ! PLACEHOLDER
    return Commands.none();
  }
}
