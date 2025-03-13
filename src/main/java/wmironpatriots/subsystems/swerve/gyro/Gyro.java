package wmironpatriots.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Annotations.Log;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystemComponent;

public abstract class Gyro extends LoggedSubsystemComponent {
   @Log public double headingRevs; 

   public abstract Rotation2d getRotation2d();
}
