// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Logged;
import wmironpatriots.commands.Autonomous;
import wmironpatriots.commands.DriveToPose;
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
  private final CommandJoystick operatorJoystick;

  private final Superstructure superstructure;
  private final Optional<Vision> vision;
  private final Climb climb;
  private final Swerve swerve;

  private final Alert browningOut;

  private int side, branch;

  Timer gcTimer = new Timer();

  private final SendableChooser<Command> autons;

  Command auton;

  private Command goTo;

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
    operatorJoystick = new CommandJoystick(2);

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
    superstructure =
        new Superstructure(
            swerve, new ElevatorIOComp(), new TailIOComp(), new RollerIOComp(), new ChuteIOComp());

    goTo = new DriveToPose(swerve, swerve::getAlignPose);

    climb.setDefaultCommand(climb.runClimb(0));

    autons = Autonomous.configureAutons(swerve, superstructure);

    // * SETUP BINDS
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftY),
            () -> -JoystickUtil.applyTeleopModifier(driver::getLeftX),
            () -> -JoystickUtil.applyTeleopModifier(driver::getRightX),
            () -> 1.0));
    // () -> MathUtil.clamp(1.5 - driver.getRightTriggerAxis(), 0.0, 1.0)));
    driver
        .a()
        .whileTrue(
            Commands.run(
                () ->
                    swerve.resetOdo(
                        new Pose2d(swerve.getPose().getTranslation(), new Rotation2d()))));
    driver.rightBumper().whileTrue(superstructure.score());
    driver.x().whileTrue(goTo);
    driver.rightTrigger(0.3).onTrue(setbah(0.0));
    driver.leftTrigger(0.3).onTrue(setbah(1.0));

    // driver.y().whileTrue(swerve.driveToPoseCmmd(() -> Swerve.AlignTargets.A));
    operator.a().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L1));
    operator.x().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L2));
    operator.y().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L3));
    operator.b().whileTrue(superstructure.scoreCoralCmmd(ReefLevel.L4));
    operator.leftBumper().whileTrue(superstructure.intakeCoralCmmd());
    operator.rightBumper().whileTrue(superstructure.outtakeCoralCmmd());
    operator.leftTrigger(.03).whileTrue(superstructure.AutointakeCoralCmmd());
    operator.povLeft().whileTrue(climb.runClimb(8));

    operator.povUp().whileTrue(superstructure.HIGHdeAlgaeCommand());
    operator.povDown().whileTrue(superstructure.LOWdeAlgaeCommand());

    operatorJoystick.button(11).onTrue(setbah(1));
    operatorJoystick.button(9).onTrue(setbah(2));
    operatorJoystick.button(7).onTrue(setbah(3));
    operatorJoystick.button(8).onTrue(setbah(4));
    operatorJoystick.button(10).onTrue(setbah(5));
    operatorJoystick.button(12).onTrue(setbah(6));

    gcTimer.start();

    SmartDashboard.putData(autons);
  }

  public Command setbah(double other) {
    return Commands.runOnce(
        () -> {
          this.branch = (int) other;
          swerve.setAlignTarget(AlignTargets.values()[((side * 2) - branch) - 1]);
        });
  }

  public Command setbah(int side) {
    return Commands.runOnce(
        () -> {
          this.side = side;
          swerve.setAlignTarget(AlignTargets.values()[((side * 2) - branch) - 1]);
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
  public void autonomousInit() {
    auton = autons.getSelected();
    if (auton != null) {
      auton.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (auton != null) {
      auton.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}
