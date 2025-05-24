// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands.factories;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import wmironpatriots.subsystems.Swerve.Swerve;
import wmironpatriots.subsystems.Swerve.SwerveConstants;

public class SwerveCommandFactory {
  private final Swerve swerve;

  public SwerveCommandFactory(Swerve swerve) {
    this.swerve = swerve;
  }

  public Command defaultCommand() {
    return Commands.runOnce(() -> swerve.stop(), swerve);
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
    return Commands.run(
        () ->
            swerve.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedMagnitude.getAsDouble()
                        * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond)
                        * -1,
                    ySpeedMagnitude.getAsDouble()
                        * SwerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
                    angularRateMagnitude.getAsDouble()
                        * SwerveConstants.MAX_ANGULAR_RATE.in(RadiansPerSecond)
                        * -1,
                    swerve.getHeadingRotation2d())),
        swerve);
  }
}
