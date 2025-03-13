package wmironpatriots.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import wmironpatriots.Constants.MATRIXID;

public class GyroIOComp extends Gyro {
    private final Pigeon2 pigeon;
    private final BaseStatusSignal yaw;

    public GyroIOComp() {
        pigeon = new Pigeon2(MATRIXID.PIGEON, MATRIXID.CANCHAN);
        yaw = pigeon.getYaw();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(yaw);
        headingRevs = yaw.getValueAsDouble();
    }

    @Override
    public Rotation2d getRotation2d() {
        return pigeon.getRotation2d();
    }
}
