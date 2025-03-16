// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

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
import wmironpatriots.Constants;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.module.Module;

public class Autonomous {
  public static SendableChooser<Command> configureAutons(Swerve swerve) {
    RobotConfig robotConfig =
        new RobotConfig(
            Swerve.MASS_KG,
            5.053,
            new ModuleConfig(
                Module.WHEEL_RADIUS_METERS,
                Swerve.MAX_LINEAR_SPEED,
                1.0,
                DCMotor.getKrakenX60Foc(1),
                60.0,
                1),
            Swerve.MODULE_LOCS);

    AutoBuilder.configure(
        swerve::getPose,
        swerve::resetOdo,
        swerve::getCurrentVelocities,
        swerve::runVelocities,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0), Constants.TICK_SPEED),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        swerve);

    SendableChooser<Command> choser = AutoBuilder.buildAutoChooser();
    choser.addOption("play dead", Commands.none());
    return choser;
  }
}
