// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.junction.LoggedRobot;
import wmironpatriots.subsystems.swerve.SwerveSubsystem;
import wmironpatriots.subsystems.swerve.constants.CompBotSwerveConfigs;
import wmironpatriots.util.ControllerUtil;
import wmironpatriots.util.deviceUtil.TalonFXHandler;

public class Robot extends LoggedRobot implements Logged {
  private final CommandScheduler m_scheduler = CommandScheduler.getInstance();

  private final PS5Controller m_driveController;

  public static final TalonFXHandler talonHandler = new TalonFXHandler();

  private final SwerveSubsystem m_swerveSubsystem;

  public Robot() {
    startupMonologue();

    RobotController.setBrownoutVoltage(6.0);

    m_driveController = new PS5Controller(0);
    // Subsystem init
    m_swerveSubsystem = new SwerveSubsystem(new CompBotSwerveConfigs());

    // Default Commands
    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.teleopSwerveCommmand(
            ControllerUtil.applyDeadband(m_driveController::getLeftY, false),
            ControllerUtil.applyDeadband(m_driveController::getLeftX, false),
            ControllerUtil.applyDeadband(m_driveController::getRightX, false)));
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
  public void teleopInit() {}

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
