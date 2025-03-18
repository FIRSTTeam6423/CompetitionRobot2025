// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import org.ironmaple.simulation.SimulatedArena;
import wmironpatriots.Constants.FLAGS;
import wmironpatriots.commands.Autonomous;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ReefBranch;
import wmironpatriots.subsystems.superstructure.Superstructure.ReefLevel;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.Swerve.ScoreTargets;
import wmironpatriots.subsystems.vision.Vision;
import wmironpatriots.subsystems.vision.VisionIOComp;
import wmironpatriots.utils.deviceUtils.JoystickUtil;

public class Robot extends TimedRobot implements Logged {
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  private final Optional<Superstructure> superstructure;
  private final Optional<Vision> vision;
  private final Swerve swerve;

  private final Alert tuningEnabled, superstructureDisabled, browningOut;

  private final SendableChooser<Command> autoChooser;

  public Robot() {
    // * SYSTEMS INIT
    // Shuts up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initalize logging
    initMonologue();
    // Frees memory DO NOT remove
    SignalLogger.enableAutoLogging(false);

    // Sets up alerts
    tuningEnabled = new Alert("Tuning mode is enabled!", AlertType.kWarning);
    superstructureDisabled = new Alert("Superstructure is disabled!", AlertType.kWarning);
    browningOut = new Alert("Browning Out!", AlertType.kWarning);

    // Flag checks
    if (FLAGS.TUNING_MODE) tuningEnabled.set(true);
    if (!FLAGS.SUPERSTRUCTURE_ENABLED) superstructureDisabled.set(true);

    // Brownout trigger
    new Trigger(() -> RobotController.isBrownedOut())
        .onTrue(Commands.run(() -> browningOut.set(true)));

    // * INIT HARDWARE
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    if (Robot.isReal()) {
      swerve = new Swerve();
      vision = Optional.of(new VisionIOComp());
      addPeriodic(() -> swerve.updateVisionEstimates(vision.get().getEstimatedPoses()), 0.02);

      superstructure =
          FLAGS.SUPERSTRUCTURE_ENABLED ? Optional.of(new Superstructure(swerve)) : Optional.empty();
    } else {
      swerve = new Swerve();
      vision = Optional.empty();

      SimulatedArena.getInstance().addDriveTrainSimulation(swerve.getSimulation().get());
      swerve.resetOdo(swerve.getSimulation().get().getSimulatedDriveTrainPose());

      superstructure = Optional.empty();
      superstructureDisabled.set(true);
    }

    // * SETUP BINDS
    swerve.setDefaultCommand(
        swerve.drive(
            () -> JoystickUtil.applyTeleopModifier(driver::getLeftY),
            () -> JoystickUtil.applyTeleopModifier(driver::getLeftX),
            () -> JoystickUtil.applyTeleopModifier(driver::getRightX),
            () -> MathUtil.clamp(1.1 - driver.getRightTriggerAxis(), 0.0, 1.0)));

    driver.rightBumper().whileTrue(swerve.driveToPoseCmmd(() -> ScoreTargets.A));

    // configures bindings only if superstructure is enabled
    superstructure.ifPresent(
        s -> {
          s.robotInIntakingZone.whileTrue(rumbleDriver(0.2));

          driver.a().whileTrue(s.scoreCoralCmmd(ReefLevel.L4));

          driver.b().whileTrue(s.autoAlignCmmd(ReefBranch.A));
        });

    autoChooser = Autonomous.configureAutons(swerve);
    SmartDashboard.putData(autoChooser);
  }

  /** Command for driver controller rumble */
  private Command rumbleDriver(double value) {
    return Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value));
  }

  /** Initalizes monologue from generated build constants */
  private void initMonologue() {
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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (Robot.isSimulation()) SimulatedArena.getInstance().simulationPeriodic();

    Monologue.updateAll();

    SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Brownout?", RobotController.isBrownedOut());
    SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoChooser.getSelected().schedule();
  }

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
