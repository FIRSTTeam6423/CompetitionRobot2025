package wmironpatriots.subsystems.chute;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Chute extends SubsystemBase {

    public Command runChuteVolts(double volts) {
        return this.run(() -> {});
    }
    
}
