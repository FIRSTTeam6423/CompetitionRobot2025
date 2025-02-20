package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

public abstract class Tail extends SubsystemBase {
    /** CONSTANTS */
    public static final double REDUCTION = 5 * 5 * 0.5;
    public static final double MASS_KG = 0.0;

    public static final double MAX_POSE_RADS = 0.0;

    public static final double CURRENT_LIMIT = 0.0;

    /** LOGGED VALUES */
    @Log protected boolean pivotMotorOk = false;
    @Log protected boolean rollerMotorOk = false;

    @Log protected boolean beamUno = false;
    @Log protected boolean beamDos = false;

    @Log protected double pivotSetpointRads;
    @Log protected double pivotPoseRads;
    @Log protected double pivotVelRPM;
    @Log protected double pivotAppliedVolts;
    @Log protected double pivotSupplyCurrentAmps;
    @Log protected double pivotTorqueCurrentAmps;

    @Log protected double rollerVelRPM;
    @Log protected double rollerAppliedVolts;
    @Log protected double rollerSupplyCurrentAmps;
    @Log protected double rollerTorqueCurrentAmps;

    /** HARDWARE METHODS */
    public abstract void runPivotVolts(double volts);

    public abstract void runPivotSetpoint(double setpointRadians);

    public abstract void runRollerSpeed(double speed);

    public abstract void setEncoderPose(double poseMeters);

    public abstract void stopRollers();

    public abstract void pivotCoasting(boolean enabled);
}
