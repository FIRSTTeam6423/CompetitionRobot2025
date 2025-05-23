// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands.factories;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lib.swerve.ChassisVelocity;
import wmironpatriots.subsystems.Swerve.Swerve;

public class SwerveCommandFactory {
  private final Swerve swerve;

  public SwerveCommandFactory(Swerve swerve) {
    this.swerve = swerve;
  }

  /**
   * Drives robot based on input streams
   *
   * @param xSpeedMagnitude Input stream representing desired x speed magnitude
   * @param ySpeedMagnitude Input stream representing desired y speed magnitude
   * @param angularRateMagnitude Input stream representing desiredd angular rate magnitude
   * @return {@link Command}
   */
  public Command teleopDrive(
      DoubleSupplier xSpeedMagnitude,
      DoubleSupplier ySpeedMagnitude,
      DoubleSupplier angularRateMagnitude) {
    return swerve.setChassisVelocity(
        ChassisVelocity.fromFieldRelativeSpeeds(
            xSpeedMagnitude.getAsDouble(),
            ySpeedMagnitude.getAsDouble(),
            angularRateMagnitude.getAsDouble(),
            swerve.getHeadingRotation2d().getRadians()));
  }
}
