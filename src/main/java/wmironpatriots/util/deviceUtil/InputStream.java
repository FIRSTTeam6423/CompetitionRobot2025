package wmironpatriots.util.deviceUtil;

import java.util.function.DoubleSupplier;

/** Inspired by 1155's InputStream class
 * A functional interface for modifying double suppliers
 */
@FunctionalInterface
public interface InputStream extends DoubleSupplier {

    /** Create a new input stream from a previously existing stream */
    public static InputStream of(DoubleSupplier stream) {
        return stream::getAsDouble;
    }

    public static InputStream hypot(InputStream y, InputStream x) {
        return () -> Math.hypot(x.get(), y.get());
    }

    public static InputStream arcTan(InputStream y, InputStream x) {
        return () -> Math.atan2(y.get(), x.get());
    }

    public default double get() {
        return getAsDouble();
    }

    /** Deadbands stream's value within specified range around 0 and clamps value to specified distance from 0 */
    public default InputStream deadband(double deadband, double max) {
        double value = get();
        return () -> Math.abs(value) > deadband
            ? Math.max(-max, Math.min(value, max))
            : 0.0;
    }
}