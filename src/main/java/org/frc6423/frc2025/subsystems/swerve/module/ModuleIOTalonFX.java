package org.frc6423.frc2025.subsystems.swerve.module;

import org.frc6423.frc2025.util.swerveUtil.ModuleConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOTalonFX implements ModuleIO {

    private final TalonFX m_pivotMotor, m_driveMotor;
    private final CANcoder m_pivotCANcoder;

    // Control Requests
    private final VoltageOut m_voltageOutRequest;
    private final PositionTorqueCurrentFOC m_positionTorqueRequest;
    private final VelocityTorqueCurrentFOC m_velocityTorqueRequest;

    // Signals
    private final BaseStatusSignal pivotABSPose, pivotPose, pivotVelRadsPerSec, pivotAppliedVolts, pivotSupplyCurrent, pivotTorqueCurrent;
    private final BaseStatusSignal drivePoseRads, driveVelRadsPerSec, driveAppliedVolts, driveSupplyCurrent, driveTorqueCurrent;

    public ModuleIOTalonFX(ModuleConfig config) {
        m_pivotMotor = new TalonFX(config.kPivotID);
        m_driveMotor = new TalonFX(config.kDriveID);
        
        m_pivotCANcoder = new CANcoder(config.kPivotABSID);

        m_voltageOutRequest = new VoltageOut(0.0).withEnableFOC(true);
        m_positionTorqueRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
        m_velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

        pivotABSPose = m_pivotCANcoder.getAbsolutePosition();
        pivotPose = m_pivotMotor.getPosition();
        pivotVelRadsPerSec = m_pivotMotor.getVelocity();
        pivotAppliedVolts = m_pivotMotor.getMotorVoltage();
        pivotSupplyCurrent = m_pivotMotor.getStatorCurrent();
        pivotTorqueCurrent = m_pivotMotor.getTorqueCurrent();

        drivePoseRads = m_driveMotor.getPosition();
        driveVelRadsPerSec = m_driveMotor.getVelocity();
        driveAppliedVolts = m_driveMotor.getMotorVoltage();
        driveSupplyCurrent = m_driveMotor.getStatorCurrent();
        driveTorqueCurrent = m_driveMotor.getTorqueCurrent();

        // ! Register to odo thread

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            pivotABSPose,
            pivotPose,
            pivotVelRadsPerSec,
            pivotAppliedVolts,
            pivotSupplyCurrent,
            pivotTorqueCurrent,
            drivePoseRads,
            driveVelRadsPerSec,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTorqueCurrent);
        
        m_pivotCANcoder.optimizeBusUtilization();
        m_pivotMotor.optimizeBusUtilization();
        m_driveMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            pivotABSPose,
            pivotPose,
            pivotVelRadsPerSec,
            pivotAppliedVolts,
            pivotSupplyCurrent,
            pivotTorqueCurrent,
            drivePoseRads,
            driveVelRadsPerSec,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTorqueCurrent);

        inputs.pivotEnabled = true;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public void setPivotVolts(double volts, boolean focEnabled) {
        m_pivotMotor.setControl(m_voltageOutRequest.withOutput(volts).withEnableFOC(focEnabled));
    }

    @Override
    public void setDriveVolts(double volts, boolean focEnabled) {
        m_driveMotor.setControl(m_voltageOutRequest.withOutput(volts).withEnableFOC(focEnabled));
    }

    @Override
    public void setPivotAngle(Rotation2d angle) {
        m_pivotMotor.setControl(m_positionTorqueRequest.withPosition(angle.getRotations()));
    }

    @Override
    public void setDriveVelocity(double velMetersPerSec, double ff) {
        m_driveMotor.setControl(
            m_velocityTorqueRequest
                .withVelocity(velMetersPerSec)
                .withFeedForward(ff)
        );
    }
    
}
