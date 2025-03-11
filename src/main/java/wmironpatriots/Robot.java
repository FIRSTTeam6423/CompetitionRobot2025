// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.Superstructure.ReefLevel;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.chute.ChuteIOComp;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;

public class Robot extends TimedRobot implements Logged {
  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Tail tail;
  private final Chute chute;

  private final CommandXboxController operator;

  public Robot() {
    // * MONOLOGUE SETUP
    DriverStation.silenceJoystickConnectionWarning(true);

    Monologue.setupMonologue(
        this,
        "/Robot",
        new MonologueConfig(DriverStation::isFMSAttached, "", false, true)
            .withLazyLogging(true)
            .withDatalogPrefix("")
            .withOptimizeBandwidth(DriverStation::isFMSAttached));

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

    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    operator = new CommandXboxController(1);

    elevator = new ElevatorIOComp();
    tail = new TailIOComp();
    chute = new ChuteIOComp();

    superstructure = new Superstructure(elevator, tail, chute);

    elevator.setDefaultCommand(superstructure.defaultElevatorCmmd());
    tail.setDefaultCommand(superstructure.defaultTailCmmd());
    chute.setDefaultCommand(superstructure.defaultChuteCmmd());

    operator.a().whileTrue(superstructure.score(ReefLevel.L4));
    operator.b().whileTrue(superstructure.intakeCoral());
    operator.povRight().whileTrue(superstructure.outtakeCoral());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Monologue.updateAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}
