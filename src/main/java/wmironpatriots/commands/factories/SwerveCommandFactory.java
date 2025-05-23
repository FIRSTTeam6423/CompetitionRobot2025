package wmironpatriots.commands.factories;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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
  public Command teleopDrive(DoubleSupplier xSpeedMagnitude, DoubleSupplier ySpeedMagnitude, DoubleSupplier angularRateMagnitude) {
    return swerve.setChassisVelocity(
      ChassisVelocity.fromFieldRelativeSpeeds(
        xSpeedMagnitude.getAsDouble(), 
        ySpeedMagnitude.getAsDouble(), 
        angularRateMagnitude.getAsDouble(), 
        swerve.getHeadingRotation2d().getRadians()));
  }
}
