package wmironpatriots.subsystems.swerve.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import wmironpatriots.utils.LoggedSubsystemComponent;

public abstract class Module extends LoggedSubsystemComponent {
    // * LOGGED VALUES
    @Log public double pivotPoseRevs;
    @Log public double pivotAppliedVolts;
    @Log public double pivotCurrentAmps;
    @Log public double pivotTorqueAmps;
    @Log public double drivePoseMeters;
    @Log public double driveVelMPS;
    @Log public double driveAppliedVolts;
    @Log public double driveCurrentAmps;
    @Log public double driveTorqueAmps;

    /**
     * Optimizes and runs desired swerve module state
     * 
     * @param setpoint desired state
     * @return optimized state
     */
    public SwerveModuleState runModuleSetpoint(SwerveModuleState setpoint) {
        return new SwerveModuleState();
    }

    /** Stop module */
    public void stopModule() {
        stopMotors();
    }

    /**
     * Get current module state
     * 
     * @return SwerveModuleState based on the current module state
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState();
    }

    /**
     * Get current module field pose
     * 
     * @return SwerveModulePosition based on current module position
     */
    public SwerveModulePosition getModulePose() {
        return new SwerveModulePosition();
    }

    // * HARDWARE METHODS
    protected abstract void setPivotVolts(double volts);

    protected abstract void setPivotPose(double poseRevs);

    protected abstract void setDriveVolts(double volts);

    protected abstract void setDriveVel(double velMPS);

    protected abstract void stopMotors();

    protected abstract void enableCoastMode(boolean enabled);
}