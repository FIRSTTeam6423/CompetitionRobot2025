// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
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
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.vision.Vision;
import wmironpatriots.subsystems.vision.VisionIOComp;
import wmironpatriots.utils.deviceUtils.JoystickUtil;

public class Robot extends TimedRobot implements Logged {
  private final CommandXboxController driver, operator;

  // private final Superstructure superstructure;
  private final Optional<Vision> vision;
  private final Swerve swerve;

  private final Alert browningOut;

  public Robot() {
    // * SYSTEMS INIT
    // Shuts up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initalize logging
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
    // Frees memory DO NOT REMOVE
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // Sets up alerts
    browningOut = new Alert("Browning Out!", AlertType.kWarning);

    // Brownout trigger
    new Trigger(() -> RobotController.isBrownedOut())
        .onTrue(Commands.run(() -> browningOut.set(true)));

    // * INIT HARDWARE
    driver = new CommandXboxController(0);
    operator = new CommandXboxController(1);

    swerve = new Swerve();
    // addPeriodic(() -> swerve.showAlignTarget(target.get()), 0.02);
    vision = Optional.of(new VisionIOComp());
    addPeriodic(() -> swerve.updateVisionEstimates(vision.get().getEstimatedPoses()), 0.02);
    // superstructure = new Superstructure(swerve);

    // * SETUP BINDS
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftY),
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftX),
            () -> JoystickUtil.applyTeleopModifier(driver::getRightX),
            () -> MathUtil.clamp(1.1 - driver.getRightTriggerAxis(), 0.0, 1.0)));

    driver
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    swerve.resetOdo(
                        new Pose2d(swerve.getPose().getTranslation(), new Rotation2d()))));

    // configures bindings only if superstructure is enabled
    // driver.a().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L4));
  }

  /** Command for driver controller rumble */
  private Command rumbleDriver(double value) {
    return Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value));
  }

  /** Command for driver controller rumble */
  private Command rumbleOperator(double value) {
    return Commands.run(() -> operator.setRumble(RumbleType.kBothRumble, value));
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
