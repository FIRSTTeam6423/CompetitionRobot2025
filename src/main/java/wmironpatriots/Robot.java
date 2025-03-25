// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.LoggedCommandRobot;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import wmironpatriots.subsystems.swerve.Swerve;

public class Robot extends LoggedCommandRobot {
  private final Swerve swerve;

  private final Alert brownout;

  public Robot() {
    // * INITALIZE SYSTEMS
    super(Constants.TICK_SPEED.in(Seconds));

    // Monologue setup
    Monologue.setupMonologue(
        this,
        "/Logged",
        new MonologueConfig()
            .withDatalogPrefix("")
            .withOptimizeBandwidth(DriverStation::isFMSAttached)
            .withLazyLogging(true));
    // logs build data to the datalog
    final String meta = "/BuildData/";
    Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
    Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
    Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
    Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
    Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Monologue.log(meta + "GitDirty", "All changes committed");
        break;
      case 1:
        Monologue.log(meta + "GitDirty", "Uncomitted changes");
        break;
      default:
        Monologue.log(meta + "GitDirty", "Unknown");
        break;
    }
    // ! Uncomment next line if you expirence massive lag/loop-overuns while connected to fms
    // SignalLogger.stop(); SignalLogger.enableAutoLogging(false);

    // Setup alerts
    brownout = new Alert("Brownout detected!", AlertType.kWarning);
    Trigger isBrowningOut =
        new Trigger(RobotController::isBrownedOut)
            .onTrue(Commands.runOnce(() -> brownout.set(true)));

    // * INITALIZE SUBSYSTEMS
    swerve = new Swerve();

    // * CONFIGURE GAME BEHAVIOR
  }

  @Override
  public Command getAuton() {
    return Commands.run(() -> {}); // ! PLACEHOLDER
  }
}
