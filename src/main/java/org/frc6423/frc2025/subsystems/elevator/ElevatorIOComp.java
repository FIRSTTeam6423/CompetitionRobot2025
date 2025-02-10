package org.frc6423.frc2025.subsystems.elevator;

import static org.frc6423.frc2025.Constants.*;

import org.frc6423.frc2025.util.motorUtil.TalonFXUtil;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOComp implements ElevatorIO {
    private final TalonFX m_lMotor, m_rMotor;
    private final TalonFXConfiguration m_lMotorConf, m_rMotorConf;

    public ElevatorIOComp() {
        m_lMotor = new TalonFX(0, kCANivore);
        m_rMotor = new TalonFX(0, kCANivore);

        // register to global talonfx array
        TalonFXUtil.registerMotor(m_lMotor);
        TalonFXUtil.registerMotor(m_rMotor);

        m_lMotorConf = new TalonFXConfiguration();
        m_lMotorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_lMotorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_lMotorConf.Slot0.withKP(0.0).withKI(0.0).withKD(0.0); // PID config
        m_lMotorConf.Slot0.withKS(0.0).withKV(0.0).withKA(0.0); // feedforward config

        m_lMotorConf.CurrentLimits.StatorCurrentLimit = 80.0;
        m_lMotorConf.CurrentLimits.StatorCurrentLimitEnable = true;
        m_lMotorConf.CurrentLimits.SupplyCurrentLimit = 80.0;
        m_lMotorConf.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
        m_lMotorConf.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_lMotorConf.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        m_lMotorConf.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

        m_lMotorConf.MotionMagic.MotionMagicCruiseVelocity = 0.0; // ! TODO
        m_lMotorConf.MotionMagic.MotionMagicAcceleration = 0.0;
        m_lMotorConf.MotionMagic.MotionMagicJerk = 0.0;

        m_rMotorConf = new TalonFXConfiguration();
        m_rMotorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_rMotorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
    }

    @Override
    public void runLeftMotorVolts(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runLeftMotorVolts'");
    }

    @Override
    public void runRightMotorVolts(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runRightMotorVolts'");
    }

    @Override
    public void resetPose(double poseMeters) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }

    @Override
    public void setCoasting(boolean enabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCoasting'");
    }

    @Override
    public void runTargetPose(double poseMeters) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runTargetPose'");
    }

    @Override
    public void runTargetPose(double poseMeters, double ff) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runTargetPose'");
    }
}
