// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

import static wmironpatriots.Constants.DT_TIME;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import wmironpatriots.subsystems.CommandSwerveDrivetrain;
import wmironpatriots.subsystems.elevator.Elevator;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.tail.Tail;

public class Autonomous {
  public static SwerveRequest.RobotCentric reqSpeed =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public static SendableChooser<Command> configureAutons(
      CommandSwerveDrivetrain drivetrain, Tail tail, Elevator elevator) {
    RobotConfig robotConfig =
        new RobotConfig(
            Swerve.MASS_KG,
            5.503,
            new ModuleConfig(
                Module.WHEEL_RADIUS_METERS,
                Swerve.MAX_LINEAR_ACCEL_MPS_SQRD,
                1.0,
                DCMotor.getKrakenX60(1),
                60.0,
                1),
            Swerve.MODULE_LOCS);

    AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::getSpeeds,
        (velocities, ff) ->
            drivetrain.applyRequest(
                () ->
                    new SwerveRequest.RobotCentric()
                        .withVelocityX(velocities.vxMetersPerSecond)
                        .withVelocityY(velocities.vyMetersPerSecond)
                        .withRotationalRate(velocities.omegaRadiansPerSecond)),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0), DT_TIME),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drivetrain);

    Command algaeHigh =
        Commands.parallel(
                tail.setTargetPoseCmmd(Tail.POSE_MOVE_ANGLE)
                    .repeatedly()
                    .until(() -> tail.inSetpointRange() && elevator.inSetpointRange())
                    .andThen(tail.setTargetPoseCmmd(Tail.POSE_ALGAE_HIGH)),
                elevator
                    .setTargetPoseCmmd(Elevator.POSE_ALGAE_HIGH)
                    .repeatedly()
                    .onlyIf(() -> tail.inSetpointRange()))
            .withDeadline(new WaitCommand(2));

    NamedCommands.registerCommand("removeHigh", new ScheduleCommand(algaeHigh));

    SendableChooser<Command> choser = AutoBuilder.buildAutoChooser();
    choser.addOption("play dead", Commands.none());
    return choser;
  }
}
