package wmironpatriots.subsystems.Swerve.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import wmironpatriots.Constants.MATRIXID;

public class GyroHardwareComp implements GyroHardware {
  private final Pigeon2 pigeon;

  public GyroHardwareComp() {
    var pigeonId = MATRIXID.PIGEON;
    pigeon = new Pigeon2(pigeonId.getId(), pigeonId.getBusName());
  }

  @Override
  public Rotation3d getRotation3d() {
    return pigeon.getRotation3d();
  }
}
