// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;

public abstract class LoggedCommandRobot extends TimedRobot implements Logged {
  private final Timer gcTimer = new Timer();
  protected final CommandScheduler scheduler = CommandScheduler.getInstance();

  private Command autonCommand;

  public LoggedCommandRobot() {
    this(kDefaultPeriod);
  }

  public LoggedCommandRobot(double period) {
    super(period);
    // Monologue setup
    Monologue.setupMonologue(
        this,
        "/Logged",
        new MonologueConfig(DriverStation::isFMSAttached, "", false, true)
            .withDatalogPrefix("")
            .withOptimizeBandwidth(DriverStation::isFMSAttached)
            .withLazyLogging(true));

    gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    // Profiles the Command Scheduler and Monologue
    Tracer.traceFunc("CommandScheduler", scheduler::run);
    Tracer.traceFunc("Monologue", Monologue::updateAll);

    // RIO memory optimization
    if (gcTimer.hasElapsed(5)) {
      System.gc();
    }
  }

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
    System.gc();
  }

  @Override
  public void autonomousInit() {
    autonCommand = getAutonCommand();

    if (autonCommand != null) {
      autonCommand.schedule();
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
    if (autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    scheduler.cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /**
   * @return {@link Command} to schedule on autonomous init
   */
  protected abstract Command getAutonCommand();
}