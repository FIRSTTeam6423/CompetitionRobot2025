// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static wmironpatriots.Constants.SWERVE_SIM_CONFIG;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;
import monologue.Logged;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import wmironpatriots.commands.ChoreoFactory;
import wmironpatriots.generated.TunerConstants;
import wmironpatriots.subsystems.CommandSwerveDrivetrain;
import wmironpatriots.subsystems.Superstructure;
import wmironpatriots.subsystems.Superstructure.Requests;
import wmironpatriots.subsystems.chute.Chute;
import wmironpatriots.subsystems.chute.ChuteIOComp;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.elevator.ElevatorIOComp;
import wmironpatriots.subsystems.elevator.ElevatorIOSim;
import wmironpatriots.subsystems.tail.Tail;
import wmironpatriots.subsystems.tail.TailIOComp;
import wmironpatriots.subsystems.tail.TailIOSim;
import wmironpatriots.util.deviceUtil.OperatorController;

public class Robot extends TimedRobot implements Logged {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final CommandXboxController joystick;
  private final CommandXboxController operatorController;

  private final OperatorController operatorController2;

  private final Tail tail;
  private final Elevator elevator;
  private final Chute chute;
  private final ChoreoFactory factory;
  private final RobotVisualizer visualizer;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
          * 0.75; // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final AutoChooser chooser;

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
    joystick = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    operatorController2 = new OperatorController(3);

    Optional<SwerveDriveSimulation> swerveSim =
        Robot.isSimulation()
            ? Optional.of(
                new SwerveDriveSimulation(
                    SWERVE_SIM_CONFIG.get(), new Pose2d(3, 3, new Rotation2d())))
            : Optional.empty();

    if (Robot.isReal()) {
      tail = new TailIOComp();
      elevator = new ElevatorIOComp();
      chute = new ChuteIOComp();
    } else {
      tail = new TailIOSim();
      elevator = new ElevatorIOSim();
      chute = new Chute() {};
    }

    // Setup simulated arena if simulated
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim.orElse(null));
    }

    // * DEFAULT COMMANDS
    // Set up driver input streams
    configureBindings();

    // elevator.setDefaultCommand(
    //     Commands.sequence(
    //         elevator.runPoseZeroingCmmd().onlyIf(() -> !elevator.isZeroed()),
    //         elevator
    //             .setTargetPoseCmmd(3.4)
    //             .until(
    //                 () ->
    //                     elevator.inSetpointRange() || elevator.getSetpoint() >
    // elevator.getPose()),
    //         elevator.stopMotorInputCmmd()));

    // tail.setDefaultCommand(
    //     Commands.sequence(
    //         // tail.setRollerSpeedCmmd(0.0),
    //         tail.runPoseZeroingCmmd().onlyIf(() -> !tail.isZeroed()),
    //         tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
    //             .until(
    //                 () -> elevator.inSetpointRange() || elevator.getSetpoint() >
    // elevator.getPose())
    //             .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_ANGLE))));

    // .until(() -> Superstructure.isTailSafe(elevator, tail)),
    // tail.setTargetPoseCmmd(Tail.POSE_MIN_REVS).until(() -> tail.inSetpointRange()),
    // tail.stopMotorInputCmmd()));
    /*
    driveController
        .x()
        .whileTrue(tail.setRollerSpeedCmmd(1.4))
        .whileFalse(tail.setRollerSpeedCmmd(0));

    driveController
        .a()
        .whileTrue(tail.setRollerSpeedCmmd(-1.4))
        .whileFalse(tail.setRollerSpeedCmmd(0));

    driveController
        .y()
        .whileTrue(chute.runChuteSpeedCmmd(-1).alongWith(tail.setRollerSpeedCmmd(1.3)))
        .whileFalse(chute.runChuteSpeedCmmd(0).alongWith(tail.setRollerSpeedCmmd(0)));

    driveController.b().whileTrue(elevator.setTargetPoseCmmd(Elevator.POSE_L3));
    */
    // joystick.x().whileTrue(tail.setRollerSpeedCmmd(1)).onFalse(tail.setRollerSpeedCmmd(0.0));

    // * ELEVATOR LEVEL 1 COMMAND
    operatorController
        .a()
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_ANGLE)))
        .whileTrue(this.setElevatorToStowed().onlyIf(() -> tail.inSetpointRange()));

    // // * ELEVATOR LEVEL 2 COMMAND
    operatorController
        .x()
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_L2)))
        .whileTrue(
            elevator.setTargetPoseCmmd(Elevator.POSE_L2).onlyIf(() -> tail.inSetpointRange()));

    // // * ELEVATOR LEVEL 3 COMMAND
    operatorController
        .y()
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_L3)))
        .whileTrue(
            elevator.setTargetPoseCmmd(Elevator.POSE_L3).onlyIf(() -> tail.inSetpointRange()));

    // // * ELEVATOR LEVEL 4 COMMAND
    operatorController
        .b()
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                .andThen(tail.setTargetPoseCmmd(Tail.POSE_L4).until(joystick.leftBumper()).andThen(tail.setTargetPoseCmmd(Tail.POSE_IN_ANGLE))))
        .whileTrue(
            elevator.setTargetPoseCmmd(Elevator.POSE_L4).onlyIf(() -> tail.inSetpointRange()));

    // // * INTAKING CORAL COMMAND
    operatorController
        .leftBumper()
        .whileTrue(
            chute
                .runChuteSpeedCmmd(Chute.INTAKE_SPEED)
                .alongWith(tail.setRollerSpeedCmmd(Tail.INTAKING_SPEEDS)))
        .whileFalse(tail.setRollerSpeedCmmd(0))
        .whileFalse(chute.runChuteSpeedCmmd(0.0));

    // * AUTO CENTERING COMMAND

    operatorController
        .povRight()
        .onTrue(
            tail.setRollerSpeedCmmd(.5)
                .alongWith(chute.runChuteSpeedCmmd(-.1))
                .until(() -> tail.beamTripped = true)
                .andThen(tail.setRollerTimecmmd(.5, 1)).alongWith(chute.runChuteSpeedCmmd(0)));

    // // * OUTTAKING CORAL COMMAND
    operatorController
        .rightBumper()
        .whileTrue(
            chute
                .runChuteSpeedCmmd(Chute.OUTAKE_SPEED)
                .alongWith(tail.setRollerSpeedCmmd(Tail.OUTTAKING_SPEEDS)))
        .whileFalse(tail.setRollerSpeedCmmd(0))
        .whileFalse(chute.runChuteSpeedCmmd(0.0));

    // // * SCORING CORAL COMMAND
    joystick
        .rightBumper()
        .whileTrue(tail.setRollerSpeedCmmd(Tail.OUTPUTTING_SPEEDS))
        .onFalse(tail.setRollerSpeedCmmd(0.0));

    // * ALGAE DESCORING

    operatorController
        .povUp()
        .whileTrue(
            tail.setRollerSpeedCmmd(1)
                .alongWith(
                    tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                        .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                        .andThen(tail.setTargetPoseCmmd(Tail.POSE_ALGAE_HIGH))))
        .whileTrue(
            elevator
                .setTargetPoseCmmd(Elevator.POSE_ALGAE_HIGH)
                .onlyIf(() -> tail.inSetpointRange()))
        .whileFalse(tail.setRollerSpeedCmmd(0));

    operatorController
        .povDown()
        .whileTrue(
            tail.setRollerSpeedCmmd(1)
                .alongWith(
                    tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                        .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                        .andThen(tail.setTargetPoseCmmd(Tail.POSE_ALGAE_LOW))))
        .whileTrue(
            elevator
                .setTargetPoseCmmd(Elevator.POSE_ALGAE_LOW)
                .onlyIf(() -> tail.inSetpointRange()))
        .whileFalse(tail.setRollerSpeedCmmd(0));

    operatorController
        .povDown()
        .whileTrue(
            tail.setTargetPoseCmmd(Tail.POSE_ALGAE_LOW).alongWith(tail.setRollerSpeedCmmd(1)))
        .whileFalse(tail.setRollerSpeedCmmd(0));

    // * SUPERSTRUCTURE INIT
    Map<Requests, Trigger> triggerMap = new HashMap<Superstructure.Requests, Trigger>();

    // Create new superstructure visualizer
    visualizer = new RobotVisualizer(elevator, tail);

    // * AUTON INIT
    /*
      new Superstructure(
          elevator,
          tail,
          chute,
          triggerMap
          operatorController2.getBranchTarget(),
          operatorController2.getLevelTarget());
    */

    factory = new ChoreoFactory(drivetrain);
    chooser = factory.getChooser();
    SmartDashboard.putData("sdf", chooser);
    RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.povDown().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.povDown().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.povUp().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.povUp().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().simulationPeriodic();
    }

    Monologue.updateAll();
    visualizer.periodic();
  }

  @Override
  public void autonomousInit() {
    Command auton =
        factory
            .getMove(); // .withDeadline(Commands.waitUntil(() -> DriverStation.isTeleopEnabled()));
    // if (auton != null) {
    auton.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  private Command setElevatorToStowed() {
    return elevator
        .setTargetPoseCmmd(1.0)
        .until(() -> elevator.inSetpointRange() || elevator.getSetpoint() > elevator.getPose())
        .andThen(elevator.stopMotorInputCmmd());
  }
}
