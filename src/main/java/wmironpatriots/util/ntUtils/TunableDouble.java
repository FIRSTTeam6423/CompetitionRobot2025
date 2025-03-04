package wmironpatriots.util.ntUtils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableDouble implements DoubleSupplier {
    private final String key;
    private double initValue;

    public TunableDouble(String key, double initValue) {
        SmartDashboard.putNumber(key, initValue);
        
        this.key = key;
        this.initValue = initValue;
    }

    /** shorter version of {@link getAsDouble} */
    public double get() {
        return this.getAsDouble();
    }

    @Override
    public double getAsDouble() {
        return SmartDashboard.getNumber(key, initValue);
    }
}
