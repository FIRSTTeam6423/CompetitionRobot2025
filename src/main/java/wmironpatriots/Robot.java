// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.Superstructure.Requests;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.constants.CompBotSwerveConfigs;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;
import wmironpatriots.subsystems.tail.TailIOSim;
import wmironpatriots.util.ControllerUtil;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final CommandXboxController driveController;

  private final Elevator elevator;
  private final Tail tail;
  private final Swerve swerve;

  private final Visualizer visualizer;

  public Robot() {
    // * MONOLOGUE SETUP
    DriverStation.silenceJoystickConnectionWarning(true);

    if (isReal()) {
      SignalLogger.enableAutoLogging(false);
    }

    Monologue.setupMonologue(
        this,
        "/Robot",
        new MonologueConfig(DriverStation::isFMSAttached, "", false, false).withLazyLogging(true));

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

    // * SUBSYSTEM INIT
    driveController = new CommandXboxController(0);

    swerve = new Swerve(new CompBotSwerveConfigs());
    tail = Robot.isReal() ? new TailIOComp() : new TailIOSim();
    elevator = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOSim();

    // Create new superstructure visualizer
    visualizer = new Visualizer(elevator, tail);

    // Default commands
    swerve.setDefaultCommand(
        swerve.teleopSwerveCommmand(
            ControllerUtil.applyDeadband(driveController::getLeftY, false),
            ControllerUtil.applyDeadband(driveController::getLeftX, false),
            ControllerUtil.applyDeadband(driveController::getRightX, false)));

    elevator.setDefaultCommand(
        Commands.sequence(
            elevator.runPoseZeroingCmmd().onlyIf(() -> !elevator.isZeroed()),
            elevator.setTargetPoseCmmd(0.0).until(() -> elevator.inSetpointRange()),
            elevator.stopMotorInputCmmd()));

    // * SUPERSTRUCTURE INIT
    Map<Requests, Trigger> triggerMap = new HashMap<Superstructure.Requests, Trigger>();

    driveController.x().whileTrue(tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS));
    driveController.a().whileTrue(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS));

    new Superstructure(swerve, elevator, tail, triggerMap);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Monologue.updateAll();
    visualizer.periodic();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    elevator.zeroPoseCmmd();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
