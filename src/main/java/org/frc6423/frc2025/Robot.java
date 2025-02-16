// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025;

import static org.frc6423.frc2025.Constants.*;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc6423.frc2025.subsystems.elevator.Elevator;
import org.frc6423.frc2025.subsystems.swerve.SwerveSubsystem;
import org.frc6423.frc2025.subsystems.swerve.constants.CompBotSwerveConfigs;
import org.frc6423.frc2025.util.ControllerUtil;
import org.frc6423.frc2025.util.deviceUtil.TalonFXHandler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private final PS5Controller m_driveController;

  public static final TalonFXHandler talonHandler = new TalonFXHandler();

  private final SwerveSubsystem m_swerveSubsystem;

  public Robot() {
    // AKit init
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Version", BuildConstants.VERSION);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Dirty", String.valueOf(BuildConstants.DIRTY));
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    switch (getDeployMode()) {
      case SIMULATION:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

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
}
