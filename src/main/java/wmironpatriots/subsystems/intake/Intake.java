package wmironpatriots.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;

public abstract class Intake extends SubsystemBase implements Logged{
    /** INTAKE CONSTANTS */
    public static final double REDUCTION = 25;
    public static final double OFFSET_RADS = 0.0; // TODO MEASURE ABS OFFSET
    public static final double POSE_STOWED = 0.0;
    public static final double POSE_INTAKING = Math.PI * 2;

    /** LOGGED VALUES */
    @Log protected boolean pivotOk = false;
    @Log protected boolean rollerOk = false;

    @Log protected double pivotSetpointRads;
    @Log protected double pivotPoseABSRads;
    @Log protected double pivotPoseRads;
    @Log protected double pivotVelRadsPerSec;
    @Log protected double pivotAppliedVolts;
    @Log protected double pivotSupplyCurrent;
    @Log protected double pivotTorqueCurrent;
    @Log protected double pivotTempCelsius;
    
    /** VARIABLES */
    private final PositionTorqueCurrentFOC reqMotorPose =
        new PositionTorqueCurrentFOC(0.0);

    public Command setTargetPoseCommand(double pose) {
        return this.run(() -> {
            pivotSetpointRads = pose;
            runPivotControl(reqMotorPose.withPosition(pose));
        });
    }

    /** Enable coast mode to move the intake easier */
    public Command intakeCoasting(boolean enabled) {
        return this.runOnce(() -> enablePivotCoasting(enabled));
    }

    /** HARDWARE METHODS */
    /** Run pivot motor with control request */
    protected abstract void runPivotControl(ControlRequest request);

    /** Enable or disable motor coasting` */
    protected abstract void enablePivotCoasting(boolean booleanEnable);
}