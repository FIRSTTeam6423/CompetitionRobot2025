package wmironpatriots.subsystems.tail;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class TailIOComp extends Tail {
    private final SparkMax pivot, roller;
    private final SparkMaxConfig pivotConf, rollerConf;

    private final SparkClosedLoopController pivotFeedback;

    public TailIOComp() {
        pivot = new SparkMax(1, MotorType.kBrushless);
        roller = new SparkMax(2, MotorType.kBrushless);

        // Configure pivot
        pivotConf = new SparkMaxConfig();

        pivotConf
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) 0.0);
        
        pivotConf
            .softLimit
            .forwardSoftLimit(MAX_POSE_RADS)
            .reverseSoftLimit(0.0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        pivotConf
            .closedLoop
            .pid(0.0, 0.0, 0.0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        pivotConf
            .encoder
            .positionConversionFactor(2 * Math.PI * REDUCTION)
            .velocityConversionFactor(2 * Math.PI * REDUCTION)
            .uvwAverageDepth(16)
            .uvwMeasurementPeriod(32);

        pivot.configure(pivotConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Config roller
        rollerConf = new SparkMaxConfig();

        rollerConf
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) 0.0);
        
        roller.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure closed loop controller
        pivotFeedback = pivot.getClosedLoopController();
    }

    @Override
    public void runPivotVolts(double volts) {
        pivot.setVoltage(volts);
    }

    @Override
    public void runPivotSetpoint(double setpointRadians) {
        pivotFeedback.setReference(setpointRadians, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void runRollerSpeed(double speed) {
        roller.set(speed);
    }

    @Override
    public void setEncoderPose(double poseMeters) {
        pivot.getEncoder().setPosition(poseMeters);
    }

    @Override
    public void stopRollers() {
        roller.stopMotor();
    }

    @Override
    public void pivotCoasting(boolean enabled) {
        IdleMode idleMode = enabled ? IdleMode.kCoast : IdleMode.kBrake;
        pivotConf.idleMode(idleMode);
        pivot.configure(pivotConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
