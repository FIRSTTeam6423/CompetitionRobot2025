// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import static wmironpatriots.Constants.kCANbus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ElevatorIOComp extends Elevator {
  private final TalonFX m_parentM, m_childM;
  private final TalonFXConfiguration m_motorConf;

  private final BaseStatusSignal m_LPoseSig,
      m_LVelSig,
      m_LAppliedVolts,
      m_LSCurrentSig,
      m_LTCurrentSig,
      m_LTemp;
  private final BaseStatusSignal m_RPoseSig,
      m_RVelSig,
      m_RAppliedVolts,
      m_RSCurrentSig,
      m_RTCurrentSig,
      m_RTemp;

  public ElevatorIOComp() {
    m_parentM = new TalonFX(14, kCANbus);
    m_childM = new TalonFX(15, kCANbus); // ! ID

    // register to global talonfx array
    // Robot.talonHandler.registerTalon(m_parentM);
    // Robot.talonHandler.registerTalon(m_childM);

    m_motorConf = new TalonFXConfiguration();
    m_motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_motorConf.Slot0.withKP(0.0).withKI(0.0).withKD(0.0); // PID config
    m_motorConf.Slot0.withKS(0.0).withKV(0.0).withKA(0.0); // feedforward config

    m_motorConf.CurrentLimits.StatorCurrentLimit = 80.0;
    m_motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motorConf.CurrentLimits.SupplyCurrentLimit = 80.0;
    m_motorConf.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
    m_motorConf.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_motorConf.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    m_motorConf.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    m_motorConf.MotionMagic.MotionMagicCruiseVelocity = 0.0; // ! TODO
    m_motorConf.MotionMagic.MotionMagicAcceleration = 0.0;
    m_motorConf.MotionMagic.MotionMagicJerk = 0.0;

    // Conversion from rotations to meters
    // reduction * circumference
    // reduction is 1/5 and radius of spool radius is 0.878350 meters
    m_motorConf.Feedback.SensorToMechanismRatio = (1 / 5) * (2 * Math.PI * 0.878350);

    m_parentM.getConfigurator().apply(m_motorConf);
    m_childM.getConfigurator().apply(m_motorConf);

    m_childM.setControl(new Follower(m_parentM.getDeviceID(), true));
    m_childM.optimizeBusUtilization();
    m_parentM.optimizeBusUtilization();

    m_LPoseSig = m_parentM.getPosition();
    m_LVelSig = m_parentM.getVelocity();
    m_LAppliedVolts = m_parentM.getMotorVoltage();
    m_LSCurrentSig = m_parentM.getStatorCurrent();
    m_LTCurrentSig = m_parentM.getTorqueCurrent();
    m_LTemp = m_parentM.getDeviceTemp();

    m_RPoseSig = m_childM.getPosition();
    m_RVelSig = m_childM.getVelocity();
    m_RAppliedVolts = m_parentM.getMotorVoltage();
    m_RSCurrentSig = m_childM.getStatorCurrent();
    m_RTCurrentSig = m_childM.getTorqueCurrent();
    m_RTemp = m_childM.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        m_LPoseSig,
        m_LVelSig,
        m_LAppliedVolts,
        m_LSCurrentSig,
        m_LTCurrentSig,
        m_LTemp,
        m_RPoseSig,
        m_RVelSig,
        m_RAppliedVolts,
        m_RSCurrentSig,
        m_RTCurrentSig,
        m_RTemp);
  }

  @Override
  public void periodic() {
    LMotorOk =
        BaseStatusSignal.refreshAll(
                m_LPoseSig, m_LVelSig, m_RAppliedVolts, m_LSCurrentSig, m_LTCurrentSig, m_LTemp)
            .isOK();
    RMotorOk =
        BaseStatusSignal.refreshAll(
                m_RPoseSig, m_RVelSig, m_RAppliedVolts, m_RSCurrentSig, m_RTCurrentSig, m_RTemp)
            .isOK();

    LMotorPoseMeters = Units.rotationsToRadians(m_LPoseSig.getValueAsDouble());
    LMotorVelMPS = Units.rotationsPerMinuteToRadiansPerSecond(m_LVelSig.getValueAsDouble());
    LMotorAppliedVolts = m_LAppliedVolts.getValueAsDouble();
    LMotorSupplyCurrentAmps = m_LSCurrentSig.getValueAsDouble();
    LMotorTorqueCurrentAmps = m_LTCurrentSig.getValueAsDouble();
    LMotorTempCelsius = m_LTemp.getValueAsDouble();

    RMotorPoseMeters = Units.rotationsToRadians(m_RPoseSig.getValueAsDouble());
    RMotorVelMPS = Units.rotationsPerMinuteToRadiansPerSecond(m_RVelSig.getValueAsDouble());
    RMotorAppliedVolts = m_RAppliedVolts.getValueAsDouble();
    RMotorSupplyCurrentAmps = m_RSCurrentSig.getValueAsDouble();
    RMotorTorqueCurrentAmps = m_RTCurrentSig.getValueAsDouble();
    RMotorTempCelsius = m_RTemp.getValueAsDouble();

    poseMeters = (LMotorPoseMeters + RMotorPoseMeters) / 2;
    velMPS = (LMotorVelMPS + RMotorVelMPS) / 2;
  }

  @Override
  protected void runMotorControl(ControlRequest request) {
    m_parentM.setControl(request);
  }

  @Override
  protected void setEncoderPose(double poseMeters) {
    m_parentM.setPosition(0.0);
  }

  @Override
  protected void stopMotors() {
    m_parentM.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_parentM.setNeutralMode(idleMode);
  }
}
