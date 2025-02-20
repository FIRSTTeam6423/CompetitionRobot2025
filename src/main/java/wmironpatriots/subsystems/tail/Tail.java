package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;

public abstract class Tail extends SubsystemBase {
    /** CONSTANTS */
    public static final double REDUCTION = 5 * 5 * 0.5;
    public static final double MASS_KG = 0.0;

    public static final double MAX_POSE_RADS = 0.0;

    public static final double CURRENT_LIMIT = 0.0;

    /** LOGGED VALUES */
    @Log protected boolean isZeroed = false;

    @Log protected boolean pivotMotorOk = false;
    @Log protected boolean rollerMotorOk = false;

    @Log protected boolean beamUnoTriggered = false;
    @Log protected boolean beamDosTriggered = false;

    @Log protected double pivotSetpointRads;
    @Log protected double pivotPoseRads;
    @Log protected double pivotVelRPM;
    @Log protected double pivotAppliedVolts;
    @Log protected double pivotSupplyCurrentAmps;

    @Log protected double rollerVelRPM;
    @Log protected double rollerAppliedVolts;
    @Log protected double rollerSupplyCurrentAmps;

    /** Runs target position in radians from current zeroed pose */
    public Command runTargetPoseCommand(double pose) {
        return this.run(() -> {
            pivotSetpointRads = pose;
            runPivotSetpoint(pose);
        });
    }

    /** Runs rollers at specific speed */
    public Command runRollersCommand(double speed) {
        return this.run(() -> {
            runRollerSpeed(speed);
        });
    }

    /** Zeroes tail pivot at current pose */
    public Command zeroPoseCommmand() {
        return this.run(() -> {
            isZeroed = true;
            resetEncoderPose();
        });
    }

    /** Runs pivot backwards until current spikes above threshold */
    public Command runPoseZeroingCommand() {
        return this.run(() -> runPivotVolts(0.5))
            .until(() -> pivotSupplyCurrentAmps > 20.0)
            .finallyDo(
                (interrupted) -> {
                    runPivotVolts(0.0);
                    resetEncoderPose();
                    isZeroed = true;
                }
            );
    }

    /** Checks if both tail beambreaks are triggered */
    public boolean hasCoral() {
        return beamUnoTriggered && beamDosTriggered;
    }

    /** HARDWARE METHODS */
    /** Run pivot voltage */
    protected abstract void runPivotVolts(double volts);

    /** Run pivot to specific setpoint in rads */
    protected abstract void runPivotSetpoint(double setpointRadians);

    /** Set roller speed */
    protected abstract void runRollerSpeed(double speed);

    /** Reset encoder to specific pose in rads */
    protected abstract void setEncoderPose(double poseMeters);

    /** Zeros pivot at current pose */
    private void resetEncoderPose() {
        setEncoderPose(0.0);
    }

    /** Stop pivot motor */
    protected abstract void stopPivot();

    /** Stop roller motor */
    protected abstract void stopRollers();

    /** Set tail to coast mode for easier movement */
    protected abstract void pivotCoasting(boolean enabled);
}
