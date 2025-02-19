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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import org.frc6423.frc2025.BuildConstants;

import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.Superstructure.StructState;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.constants.CompBotSwerveConfigs;
import wmironpatriots.util.ControllerUtil;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler m_scheduler = CommandScheduler.getInstance();

  private final CommandXboxController m_driveController;

  private final Elevator m_elevator;
  private final Swerve m_swerve;
  private final Superstructure m_superstructure;

  public Robot() {
    startupMonologue();

    RobotController.setBrownoutVoltage(.0);

    m_driveController = new CommandXboxController(0);
    // Subsystem init
    m_swerve = new Swerve(new CompBotSwerveConfigs());
    m_elevator = Robot.isReal() ? new ElevatorIOComp() : new ElevatorIOSim();


    m_swerve.setDefaultCommand(
        m_swerve.teleopSwerveCommmand(
            ControllerUtil.applyDeadband(m_driveController::getLeftY, false),
            ControllerUtil.applyDeadband(m_driveController::getLeftX, false),
            ControllerUtil.applyDeadband(m_driveController::getRightX, false)));
    
    m_elevator.setDefaultCommand(m_elevator.runTargetPoseCommand(0.0));


    // Init superstructure
    Map<StructState, Trigger> triggerMap = new HashMap<Superstructure.StructState, Trigger>();
    triggerMap.put(StructState.L2_SETUP, m_driveController.a());

    m_superstructure = new Superstructure(m_elevator, triggerMap);

    // Debug triggers
    m_driveController.a().whileTrue(m_elevator.runTargetPoseCommand(1.717));

    m_driveController.x().whileTrue(m_elevator.runTargetPoseCommand(5.88));

    m_driveController.y().whileTrue(m_elevator.runTargetPoseCommand(12.9));
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
    m_elevator.zeroPoseCommand();
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
