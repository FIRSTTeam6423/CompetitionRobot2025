package org.frc6423.frc2025.subsystems.swerve.module;

import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO m_IO;
    
    private final int m_index;
    private final ModuleConfig m_config;

    private final ModuleIOInputsAutoLogged m_inputs;

    private SimpleMotorFeedforward m_driveff;

    public Module(ModuleConfig config) {
        m_IO = new ModuleIOTalonFX(config);

        m_index = config.kIndex;
        this.m_config = config;

        m_inputs = new ModuleIOInputsAutoLogged();

        m_driveff = new SimpleMotorFeedforward(0.0, 0.0);
    }

    /** Update auto logged inputs */
    public void updateInputs() {
        m_IO.updateInputs(m_inputs);
    }

    /** Periodically ran logic */
    public void periodic() {
        Logger.processInputs("Swerve/Module" + m_index, m_inputs);
    }

    /** Run SwerveModuleState setpoint */
    public SwerveModuleState runSetpoint(SwerveModuleState setpointState) {
        setpointState.optimize(getPivotAngle());
        setpointState.speedMetersPerSecond *= setpointState.angle.minus(getPivotAngle()).getCos();

        double speedMPS = setpointState.speedMetersPerSecond;
        m_IO.setPivotAngle(setpointState.angle);
        m_IO.setDriveVelocity(speedMPS, m_driveff.calculate(speedMPS)); // !
        return setpointState;
    }

    /** Run SwerveModuleState setpoint with setpoint wheel torque (torque-based ff) */
    public SwerveModuleState runSetpoint(SwerveModuleState setpointState, double driveTorqueNm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runSetpoint'");
    }

    /** Runs SwerveModuleState setpoint but runs drive in open loop mode */
    public SwerveModuleState runSetpointOpenloop(SwerveModuleState setpointState, boolean FOCEnabled) {
        setpointState.optimize(getPivotAngle());
        setpointState.speedMetersPerSecond *= setpointState.angle.minus(getPivotAngle()).getCos();

        double speedMPS = setpointState.speedMetersPerSecond;
        m_IO.setPivotAngle(setpointState.angle);
        m_IO.setDriveVolts(speedMPS, FOCEnabled); // !
        return new SwerveModuleState();
    }

    /** Set pivot angle setpoint */
    public void setPivotAngle(Rotation2d desiredAngle) {
        m_IO.setPivotAngle(desiredAngle);
    }

    /** Set drive torque current setpoint */
    public void runDriveCurrent(double currentAmps) {
        m_IO.setDriveTorqueCurrent(currentAmps);
    }

    /** Stop all motor input */
    public void stop() {
        m_IO.stop();
    }
    
    /** Enable module coasting */
    public void enableCoast(boolean enabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableCoast'");
    }

    /** Get Module index */
    public int getModuleIndex() {
        return m_index;
    }

    /** returns current module angle */
    public Rotation2d getPivotAngle() {
        return m_inputs.pivotABSPose;
    }
}
