package wmironpatriots.subsystems.Swerve.gyro;

import edu.wpi.first.math.geometry.Rotation3d;

public class GyroHardwareNone implements GyroHardware {
  @Override
  public Rotation3d getRotation3d() {
    return Rotation3d.kZero;
  }
}
