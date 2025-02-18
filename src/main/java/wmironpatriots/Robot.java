// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import org.frc6423.frc2025.BuildConstants;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler m_scheduler = CommandScheduler.getInstance();

  private final CommandPS5Controller m_driveController;

  private final Elevator m_elevator;

  public Robot() {
    startupMonologue();

    RobotController.setBrownoutVoltage(.0);

    m_driveController = new CommandPS5Controller(0);
    // Subsystem init
    m_elevator = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOSim();

    m_driveController
        .square()
        .whileTrue(m_elevator.runTargetPoseCommand(m_elevator.kL3PoseMeters))
        .onFalse(m_elevator.runTargetPoseCommand(0.0));
  }

  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Run command scheduler
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_elevator.runPoseZeroingCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  private void startupMonologue() {
    DriverStation.silenceJoystickConnectionWarning(true);

    if (isReal()) {
      SignalLogger.enableAutoLogging(false);
    }

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

    // Command logging
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          Monologue.log("Commands/" + command.getName(), active);
        };
    m_scheduler.onCommandInitialize(
        (Command command) -> {
          logCommandFunction.accept(command, true);
        });
    m_scheduler.onCommandFinish(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
    m_scheduler.onCommandInterrupt(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
  }
}
