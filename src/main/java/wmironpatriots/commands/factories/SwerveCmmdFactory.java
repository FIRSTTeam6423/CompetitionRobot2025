package wmironpatriots.commands.factories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.swerve.ChassisVelocity;
import wmironpatriots.subsystems.Swerve.Swerve;

public class SwerveCmmdFactory {
  private final Swerve swerve;

  public SwerveCmmdFactory(Swerve swerve) {
    this.swerve = swerve;
  }

  /**
   * Drives robot based on input streams
   * 
   * @param xSpeedMagnitude Input stream representing desired x speed magnitude
   * @param ySpeedMagnitude Input stream representing desired y speed magnitude
   * @param angularRateMagnitude Input stream representing desiredd angular rate magnitude
   * @return
   */
  public Command teleopDriveCmmd(DoubleSupplier xSpeedMagnitude, DoubleSupplier ySpeedMagnitude, DoubleSupplier angularRateMagnitude) {
    return Commands.run(() -> {
      swerve.setVelocitySetpoint(
        ChassisVelocity.fromFieldRelativeSpeeds(
          xSpeedMagnitude.getAsDouble(), 
          ySpeedMagnitude.getAsDouble(), 
          angularRateMagnitude.getAsDouble(), 
          swerve.getHeadingRotation2d().getRadians()));
    }, swerve);
  }
}
