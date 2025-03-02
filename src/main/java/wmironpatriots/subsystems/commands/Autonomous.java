// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.commands;

import static wmironpatriots.Constants.kTickSpeed;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.util.swerveUtil.SwerveConfig;

public class Autonomous {
  public static SendableChooser<Command> configureAutons(Swerve swerve) {
    SwerveConfig config = swerve.getConfig();
    wmironpatriots.util.swerveUtil.ModuleConfig genericModule = config.getModuleConfigs()[0];

    RobotConfig robotConfig = new RobotConfig(
        config.getRobotMassKg(),
        5.503, 
        new ModuleConfig(genericModule.wheelRadiusMeters, config.getMaxLinearAccelMetersPerSecSqrd(), 1.0, DCMotor.getKrakenX60(1), config.getDriveCurrentLimitAmps(), 1),
        config.getModuleLocs());
        
    AutoBuilder.configure(
        swerve::getPose,
        swerve::resetOdometry,
        swerve::getVelocitiesRobotRelative,
        (velocities, ff) -> swerve.runVelocities(velocities, false),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0, 0), 
            new PIDConstants(5.0, 0, 0), 
            kTickSpeed),
        robotConfig,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        swerve
    );

    SendableChooser<Command> choser = AutoBuilder.buildAutoChooser();
    choser.addOption("play dead", Commands.none());
    return choser;
  }
}
