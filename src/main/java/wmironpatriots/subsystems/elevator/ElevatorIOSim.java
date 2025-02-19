// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import static wmironpatriots.Constants.kCANbus;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends Elevator {
  private final TalonFX m_parentM, m_childM;
  private final TalonFXConfiguration m_motorConf;

  private final ElevatorSim m_simulatedElevator;

  public ElevatorIOSim() {
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

    m_simulatedElevator =
        new ElevatorSim( // ! These are BS constants
            DCMotor.getKrakenX60Foc(2),
            Elevator.kReduction,
            Units.kilogramsToLbs(Elevator.kMassKg),
            Elevator.kSpoolRadiusMeters,
            0.0,
            Elevator.kRangeMeters,
            true,
            0.0);
  }

  @Override
  public void periodic() {
    super.periodic();

    TalonFXSimState parentSimState = new TalonFXSimState(m_parentM);
    m_simulatedElevator.setInput(parentSimState.getMotorVoltage());

    m_simulatedElevator.update(0.02);
    parentSimState.setRotorVelocity(
        (m_simulatedElevator.getVelocityMetersPerSecond())
            / m_motorConf
                .Feedback
                .SensorToMechanismRatio); // I have no clue if this is correct lmfao

    poseMeters = m_simulatedElevator.getPositionMeters();
    velMPS = m_simulatedElevator.getVelocityMetersPerSecond();
  }

  @Override
  protected void runMotorControl(ControlRequest request) {
    m_parentM.setControl(request);
  }

  @Override
  protected void setEncoderPose(double poseMeters) {
    // Elevator is always zeroed
  }

  @Override
  protected void stopMotors() {
    m_parentM.setVoltage(0.0);
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    // Bah, humbug
  }
}
