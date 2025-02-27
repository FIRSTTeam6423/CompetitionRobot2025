// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import static wmironpatriots.Constants.CANIVORE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ElevatorIOComp extends Elevator {
  private final TalonFX parent, child;
  private final TalonFXConfiguration motorConf;

  private final PositionVoltage reqMotorPose =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final VoltageOut reqMotorVolt =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  private final BaseStatusSignal lPoseSig,
      parentVelSig,
      parentAppliedVoltsSig,
      parentSCurrentSig,
      parentTCurrentSig,
      parentTempSig;
  private final BaseStatusSignal rPoseSig,
      childVelSig,
      childAppliedVoltsSig,
      childSCurrentSig,
      childTCurrentSig,
      childTemp;

  public ElevatorIOComp() {
    super();
    parent = new TalonFX(14, CANIVORE);
    child = new TalonFX(15, CANIVORE); // ! ID

    // register to global talonfx array
    // Robot.talonHandler.registerTalon(m_parentM);
    // Robot.talonHandler.registerTalon(m_childM);

    motorConf = new TalonFXConfiguration();
    motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConf.Slot0.withKP(1).withKI(0.0).withKD(0); // PID config
    motorConf.Slot0.withKG(0.59).withKS(0.0).withKV(4.12).withKA(0.0); // feedforward config

    motorConf.CurrentLimits.StatorCurrentLimit = 80.0;
    motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConf.CurrentLimits.SupplyCurrentLimit = 80.0;
    motorConf.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
    motorConf.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConf.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    motorConf.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    motorConf.MotionMagic.MotionMagicCruiseVelocity = 0.03; // ! TODO
    motorConf.MotionMagic.MotionMagicAcceleration = 0.03;
    motorConf.MotionMagic.MotionMagicJerk = 0.0;

    parent.getConfigurator().apply(motorConf);
    child.getConfigurator().apply(motorConf);

    child.setControl(new Follower(parent.getDeviceID(), true));
    child.optimizeBusUtilization();
    parent.optimizeBusUtilization();

    lPoseSig = parent.getPosition();
    parentVelSig = parent.getVelocity();
    parentAppliedVoltsSig = parent.getMotorVoltage();
    parentSCurrentSig = parent.getStatorCurrent();
    parentTCurrentSig = parent.getTorqueCurrent();
    parentTempSig = parent.getDeviceTemp();

    rPoseSig = child.getPosition();
    childVelSig = child.getVelocity();
    childAppliedVoltsSig = parent.getMotorVoltage();
    childSCurrentSig = child.getStatorCurrent();
    childTCurrentSig = child.getTorqueCurrent();
    childTemp = child.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        lPoseSig,
        parentVelSig,
        parentAppliedVoltsSig,
        parentSCurrentSig,
        parentTCurrentSig,
        parentTempSig,
        rPoseSig,
        childVelSig,
        childAppliedVoltsSig,
        childSCurrentSig,
        childTCurrentSig,
        childTemp);
  }

  @Override
  public void periodic() {
    parentOk =
        BaseStatusSignal.refreshAll(
                lPoseSig,
                parentVelSig,
                childAppliedVoltsSig,
                parentSCurrentSig,
                parentTCurrentSig,
                parentTempSig)
            .isOK();
    childOk =
        BaseStatusSignal.refreshAll(
                rPoseSig,
                childVelSig,
                childAppliedVoltsSig,
                childSCurrentSig,
                childTCurrentSig,
                childTemp)
            .isOK();

    parentPoseRevs = lPoseSig.getValueAsDouble();
    parentVelRPM = Units.rotationsPerMinuteToRadiansPerSecond(parentVelSig.getValueAsDouble());
    parentAppliedVolts = parentAppliedVoltsSig.getValueAsDouble();
    parentSupplyCurrentAmps = parentSCurrentSig.getValueAsDouble();
    parentTorqueCurrentAmps = parentTCurrentSig.getValueAsDouble();
    parentTempCelsius = parentTempSig.getValueAsDouble();

    childPoseRevs = Units.rotationsToRadians(rPoseSig.getValueAsDouble());
    childVelRPM = Units.rotationsPerMinuteToRadiansPerSecond(childVelSig.getValueAsDouble());
    childAppliedVolts = childAppliedVoltsSig.getValueAsDouble();
    childSupplyCurrentAmps = childSCurrentSig.getValueAsDouble();
    childTorqueCurrentAmps = childTCurrentSig.getValueAsDouble();
    childTempCelsius = childTemp.getValueAsDouble();

    poseRevs = (parentPoseRevs + childPoseRevs) / 2;
    velRPM = (parentVelRPM + childVelRPM) / 2;
  }

  @Override
  protected void runMotorVolts(double volts) {
    parent.setControl(reqMotorVolt.withEnableFOC(true).withOutput(volts));
  }

  @Override
  protected void runMotorPose(double poseRevs) {
    parent.setControl(reqMotorPose.withEnableFOC(true).withPosition(poseRevs));
  }

  @Override
  protected void setEncoderPose(double pose) {
    parent.setPosition(pose);
  }

  @Override
  protected void stopMotors() {
    parent.stopMotor();
  }

  @Override
  protected void motorCoastingEnabled(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    parent.setNeutralMode(idleMode);
  }
}
