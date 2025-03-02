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
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.chute.ChuteIOComp;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.constants.CompBotSwerveConfigs;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;
import wmironpatriots.subsystems.tail.TailIOSim;
import wmironpatriots.util.deviceUtil.InputStream;
import wmironpatriots.util.deviceUtil.OperatorController;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final CommandXboxController driveController;
  private final OperatorController operatorController;

  private final Swerve swerve;
  private final Tail tail;
  private final Elevator elevator;
  private final Chute chute;

  private final RobotVisualizer visualizer;

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
    operatorController = new OperatorController(1);

    swerve = new Swerve(new CompBotSwerveConfigs());
    if (Robot.isReal()) {
      tail = new TailIOComp();
      elevator = new ElevatorIOComp();
      chute = new ChuteIOComp();
    } else {
      tail = new TailIOSim();
      elevator = new ElevatorIOSim();
      chute = new Chute() {};
    }

    // * DEFAULT COMMANDS
    // Set up driver input streams
    double maxSpeed = swerve.getConfig().getMaxLinearSpeedMetersPerSec();
    double maxAngularSpeed = swerve.getConfig().getMaxAngularSpeedRadsPerSec();

    InputStream x = InputStream.of(driveController::getLeftY);
    InputStream y = InputStream.of(driveController::getLeftX);
    InputStream speed = InputStream.of(driveController::getRightTriggerAxis).deadband(0.1, 1.0);

    InputStream hypot =
        InputStream.hypot(y, x).clamp(1).deadband(0.05, 1.0).signedPow(2).scale(maxSpeed);

    InputStream theta = InputStream.arcTan(y, x);
    x = hypot.scale(hypot.scale(theta.map(Math::cos)));
    y = hypot.scale(hypot.scale(theta.map(Math::sin)));

    InputStream omega =
        InputStream.of(driveController::getRightX)
            .deadband(0.05, 1.0)
            .signedPow(2.0)
            .scale(maxAngularSpeed);

    swerve.setDefaultCommand(swerve.teleopSwerveCommmand(x, y, omega));

    elevator.setDefaultCommand(
        Commands.sequence(
            elevator.runPoseZeroingCmmd().onlyIf(() -> !elevator.isZeroed()),
            elevator.setTargetPoseCmmd(Elevator.IDLE).until(() -> elevator.inSetpointRange()),
            elevator.stopMotorInputCmmd()));

    tail.setDefaultCommand(
        Commands.sequence(
            tail.runPoseZeroingCmmd()
                .onlyIf(() -> !tail.isZeroed() && Superstructure.isTailSafe(elevator, tail)),
            tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS)
                .until(() -> Superstructure.isTailSafe(elevator, tail)),
            tail.setTargetPoseCmmd(Tail.POSE_IN_RADS).until(() -> tail.inSetpointRange()),
            tail.stopMotorInputCmmd()));

    // * SUPERSTRUCTURE INIT
    Map<Requests, Trigger> triggerMap = new HashMap<Superstructure.Requests, Trigger>();

    driveController.x().whileTrue(tail.setTargetPoseCmmd(Tail.POSE_OUT_RADS));
    driveController.a().whileTrue(tail.setTargetPoseCmmd(Tail.POSE_IN_RADS));

    new Superstructure(
        swerve,
        elevator,
        tail,
        chute,
        triggerMap,
        () -> operatorController.getBranchTarget(),
        () -> operatorController.getLevelTarget());

    // Create new superstructure visualizer
    visualizer = new RobotVisualizer(elevator, tail);
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
