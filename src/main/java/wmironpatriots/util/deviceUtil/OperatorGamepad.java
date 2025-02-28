package wmironpatriots.util.deviceUtil;

import java.util.function.Supplier;

import wmironpatriots.Constants.ReefTarget;

public class OperatorGamepad implements Supplier<ReefTarget> {
    
    public OperatorGamepad() {
    }
    
    @Override
    public ReefTarget get() {
        return ReefTarget.L1;
    }
}
