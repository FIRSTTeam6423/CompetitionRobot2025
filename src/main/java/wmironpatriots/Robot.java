// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.Tracer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import wmironpatriots.Constants.FLAGS;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.util.deviceUtil.JoystickUtil;

public class Robot extends LoggedRobot {
  // Controllers
  private final XboxController driver, operator;

  // Subsystems
  private final Swerve swerve;
  private final Superstructure superstructure;

  // Command scheduler pointer
  private final CommandScheduler scheduler;

  // Commands
  private Command auton;

  // Alerts
  private final Alert brownout;

  private final Timer gcTimer;

  public Robot() {
    // * INITALIZE SYSTEMS
    super(Constants.TICK_SPEED.in(Seconds));

    // Garbage collector timer
    gcTimer = new Timer();
    gcTimer.start();

    // Record Akit metadata
    final String meta = "/BuildData/";
    Logger.recordMetadata(meta + "RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata(meta + "BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata(meta + "GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata(meta + "GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata(meta + "GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata(meta + "GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata(meta + "GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata(meta + "GitDirty", "Unknown");
        break;
    }

    // Start akit
    Logger.start();

    // setup data receivers
    if (isReal()) {
      // Comp bot should record to logs + nt
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new RLOGServer());
      LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
    } else {
      if (FLAGS.REPLAY_MODE) {
        new Alert("Replay mode enabled", AlertType.kInfo).set(true);
        ;

        // Run as fast as possible
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog(); // Requests logs
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read log
        Logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Saves output
      } else {
        // Sim bot should only log to nt
        Logger.addDataReceiver(new RLOGServer());
      }
    }

    // ! Uncomment next line if you expirence massive lag/loop-overuns while connected to fms
    // SignalLogger.stop(); SignalLogger.enableAutoLogging(false);

    // * INITALIZE SUBSYSTEMS AND DEVICES
    driver = new XboxController(0);
    operator = new XboxController(1);

    // Init subsystem singletons
    swerve = Swerve.create();
    superstructure = Superstructure.create();

    // * CONFIGURE GAME BEHAVIOR
    swerve.setDefaultCommand(
        swerve.driveCmd(
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftY),
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftX),
            () -> -JoystickUtil.applyTeleopModifier(driver::getRightX)));

    // Setup alerts
    brownout = new Alert("Brownout detected", AlertType.kWarning);

    // Setup commands and command scheduler
    scheduler = CommandScheduler.getInstance();
  }

  @Override
  public void robotPeriodic() {
    Tracer.traceFunc("CommandScheduler", scheduler::run);

    // Update Alerts
    brownout.set(RobotController.isBrownedOut());

    // I love our rio 1.0
    if (gcTimer.hasElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void autonomousInit() {
    if (auton != null) {
      auton.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void teleopInit() {
    if (auton != null) {
      auton.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
