// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

import static wmironpatriots.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import wmironpatriots.Constants;
import wmironpatriots.subsystems.superstructure.Superstructure;
import wmironpatriots.subsystems.superstructure.Superstructure.ReefLevel;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.module.Module;

public class Autonomous {
  public static SendableChooser<Command> configureAutons(Swerve swerve, Superstructure soup) {
    RobotConfig robotConfig =
        new RobotConfig(
            MASS_KG,
            5.053,
            new ModuleConfig(
                Module.WHEEL_RADIUS_METERS,
                MAX_LINEAR_SPEED,
                1.0,
                DCMotor.getKrakenX60Foc(1),
                60.0,
                1),
            MODULE_LOCS);

    AutoBuilder.configure(
        swerve::getPose,
        swerve::resetOdo,
        swerve::getCurrentVelocities,
        swerve::runVelocities,
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0, 0), new PIDConstants(10.0, 0, 0), Constants.TICK_SPEED),
        robotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        swerve);

    NamedCommands.registerCommand("Intake", soup.intakeCoralCmmd());
    NamedCommands.registerCommand(
        "L4", soup.scoreCoralCmmd(ReefLevel.L4).withDeadline(soup.score().withTimeout(1)));

    SendableChooser<Command> choser = AutoBuilder.buildAutoChooser();
    choser.addOption("play dead", Commands.none());
    return choser;
  }
}
