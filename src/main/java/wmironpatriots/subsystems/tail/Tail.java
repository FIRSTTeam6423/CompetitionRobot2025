package wmironpatriots.subsystems.tail;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

public class Tail extends SubsystemBase {
    /** CONSTANTS */
    public static final double kReduction = 50/1;
    public static final double kMassKg = 0.0;

    /** LOGGED VALUES */
    @Log protected boolean pivotMotorOk = false;
    @Log protected boolean rollerMotorOk = false;

    @Log protected double pivotSetpointRads;
    @Log protected double pivotPoseRads;
    @Log protected double pivotVelRPM;
    @Log protected double pivotAppliedVolts;
    @Log protected double pivotSupplyCurrentAmps;
    @Log protected double pivotTorqueCurrentAmps;
}
