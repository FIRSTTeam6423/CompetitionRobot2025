package org.frc6423.frc2025.subsystems.wrist;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOComp implements WristIO {
    private final SparkMax m_pivotM, m_rollerM;
    private final SparkMaxConfig m_pivotC, m_rollerC;

    public WristIOComp() {
        m_pivotM = new SparkMax(0, MotorType.kBrushless);
        m_rollerM = new SparkMax(0, MotorType.kBrushless);

        m_pivotC = new SparkMaxConfig();
        m_pivotM.configure(m_pivotC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_rollerC = new SparkMaxConfig();
        m_rollerM.configure(m_rollerC,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void runVolts(double voltage) {
        m_pivotM.setVoltage(voltage);
    }

    @Override
    public void runRollerVolts(double voltage) {
        m_rollerM.setVoltage(voltage);
    }

    @Override
    public void runTargetPose(Rotation2d targetAngle) {
        m_pivotM.getClosedLoopController().setReference(targetAngle.getRotations(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void runTargetRollerVeloctiy(double radsPerSec) {
        m_rollerM.getClosedLoopController().setReference(radsPerSec, ControlType.kVelocity);
    }

    @Override
    public void resetPose(Rotation2d pose) {
    } // !

    @Override
    public void setCoasting(boolean enabled) {
    } // !
}
