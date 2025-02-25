// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.Superstructure.Requests;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;
import wmironpatriots.subsystems.tail.TailIOSim;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final CommandXboxController driveController;

  private final Elevator elevator;
  private final Tail tail;
  // private final Swerve swerve;
  private final Superstructure superstructure;

  private final Visualizer visualizer;

  public Robot() {
    startupMonologue();

    driveController = new CommandXboxController(0);
    // Subsystem init
    // swerve = new Swerve(new CompBotSwerveConfigs());
    elevator = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOSim();
    tail = Robot.isReal() ? new TailIOComp() : new TailIOSim();

    visualizer = new Visualizer(elevator, tail);

    // swerve.setDefaultCommand(
    //     swerve.teleopSwerveCommmand(
    //         ControllerUtil.applyDeadband(driveController::getLeftY, false),
    //         ControllerUtil.applyDeadband(driveController::getLeftX, false),
    //         ControllerUtil.applyDeadband(driveController::getRightX, false)));

    // Init superstructure
    Map<Requests, Trigger> triggerMap = new HashMap<Superstructure.Requests, Trigger>();

    // triggerMap.put(Requests.INTAKE_CHUTE, driveController.a());
    // triggerMap.put(Requests.REEF_SCORE, driveController.b());

    driveController.x().whileTrue(tail.setTargetPoseCommand(Tail.POSE_OUT_RADS));
    driveController.a().whileTrue(tail.setTargetPoseCommand(Tail.POSE_IN_RADS));

    superstructure = new Superstructure(elevator, tail, triggerMap);
  }

  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Run command scheduler
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    Monologue.updateAll();

    visualizer.periodic();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    elevator.zeroPoseCommand();
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
    scheduler.onCommandInitialize(
        (Command command) -> {
          logCommandFunction.accept(command, true);
        });
    scheduler.onCommandFinish(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
    scheduler.onCommandInterrupt(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        });
  }
}
