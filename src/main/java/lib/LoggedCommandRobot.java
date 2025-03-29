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

/**
 * Logged command based robot; To actuall enable monologue for logging, call the
 * Monologue::setupMonologue method
 */
public abstract class LoggedCommandRobot extends TimedRobot implements Logged {
  private final CommandScheduler scheduler;

  private final Command auton;

  private final Timer gcTimer;

  public LoggedCommandRobot() {
    this(0.02);
  }

  public LoggedCommandRobot(double tickSpeed) {
    super(tickSpeed);
    // Monologue setup
    Monologue.setupMonologue(
        this,
        "/Logged",
        new MonologueConfig()
            .withDatalogPrefix("")
            .withOptimizeBandwidth(DriverStation::isFMSAttached)
            .withLazyLogging(true));

    scheduler = CommandScheduler.getInstance();

    auton = getAuton();

    gcTimer = new Timer();
    gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    Tracer.traceFunc("CommandScheduler", scheduler::run);
    Tracer.traceFunc("MonologueUpdate", Monologue::updateAll);

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
      getAuton().schedule();
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

  protected abstract Command getAuton();
}
