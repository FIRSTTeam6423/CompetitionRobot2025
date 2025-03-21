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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Logged;
import wmironpatriots.subsystems.climb.Climb;
import wmironpatriots.subsystems.climb.ClimbIOComp;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ReefLevel;
import wmironpatriots.subsystems.superstructure.chute.ChuteIOComp;
import wmironpatriots.subsystems.superstructure.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.superstructure.tail.TailIOComp;
import wmironpatriots.subsystems.superstructure.tail.roller.RollerIOComp;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.Swerve.AlignTargets;
import wmironpatriots.subsystems.vision.Vision;
import wmironpatriots.subsystems.vision.VisionIOComp;
import wmironpatriots.utils.deviceUtils.JoystickUtil;

public class Robot extends TimedRobot implements Logged {
  private final CommandXboxController driver, operator;
  private final CommandPS5Controller operatorDos;

  private final Superstructure superstructure;
  private final Optional<Vision> vision;
  private final Climb climb;
  private final Swerve swerve;

  private final Alert browningOut;

  private int side, branch;

  Timer gcTimer = new Timer();

  public Robot() {
    // * SYSTEMS INIT
    // Shuts up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initalize logging
    // Monologue.setupMonologue(
    //     this,
    //     "/Robot",
    //     new MonologueConfig(DriverStation::isFMSAttached, "", false, true)
    //         .withLazyLogging(true)
    //         .withDatalogPrefix("")
    //         .withOptimizeBandwidth(DriverStation::isFMSAttached));

    // logs build data to the datalog
    // final String meta = "/BuildData/";
    // Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
    // Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
    // Monologue.log(meta + "Version", BuildConstants.VERSION);
    // Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
    // Monologue.log(meta + "GitDirty", String.valueOf(BuildConstants.DIRTY));
    // Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
    // Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
    // Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);
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
    operatorDos = new CommandPS5Controller(2);

    side = 1;
    branch = 0;

    swerve = new Swerve();
    climb = new ClimbIOComp();
    vision = Optional.of(new VisionIOComp());
    addPeriodic(
        () -> {
          swerve.updateVisionEstimates(vision.get().getEstimatedPoses());
        },
        0.02);
    addPeriodic(
        () -> {
          swerve.setAlignTarget(getTarget());
        },
        0.1);
    superstructure =
        new Superstructure(
            swerve, new ElevatorIOComp(), new TailIOComp(), new RollerIOComp(), new ChuteIOComp());

    climb.setDefaultCommand(climb.runClimb(0));

    // * SETUP BINDS
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftY),
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftX),
            () -> -JoystickUtil.applyTeleopModifier(driver::getRightX),
            () -> MathUtil.clamp(1.1 - driver.getRightTriggerAxis(), 0.0, 1.0)));
    driver
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    swerve.resetOdo(
                        new Pose2d(swerve.getPose().getTranslation(), new Rotation2d()))));

    driver.leftTrigger(0.3).whileTrue(climb.runClimb(-8));
    driver.b().whileTrue(swerve.driveToPoseCmmd());
    driver.rightBumper().whileTrue(superstructure.score());

    // driver.y().whileTrue(swerve.driveToPoseCmmd(() -> Swerve.AlignTargets.A));
    operator.a().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L1));
    operator.x().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L2));
    operator.y().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L3));
    operator.b().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L4));
    operator.leftBumper().whileTrue(superstructure.intakeCoralCmmd());
    operator.rightBumper().whileTrue(superstructure.outtakeCoralCmmd());
    operator.povLeft().whileTrue(climb.runClimb(8));

    operatorDos.povDown().onTrue(setbah(1, branch));
    operatorDos.povLeft().onTrue(setbah(2, branch));
    operatorDos.povUp().onTrue(setbah(3, branch));
    operatorDos.povRight().onTrue(setbah(4, branch));
    operatorDos.cross().onTrue(setbah(5, branch));
    operatorDos.square().onTrue(setbah(6, branch));

    operatorDos.R1().onTrue(setbah(side, 0));
    operatorDos.L2().onTrue(setbah(side, 1));

    gcTimer.start();
  }

  public Command setbah(int side, int other) {
    return Commands.runOnce(
        () -> {
          this.side = side;
          this.branch = other;
        });
  }

  public AlignTargets getTarget() {
    return AlignTargets.values()[((side * 2) - branch) - 1];
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

    SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Brownout?", RobotController.isBrownedOut());
    SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
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
