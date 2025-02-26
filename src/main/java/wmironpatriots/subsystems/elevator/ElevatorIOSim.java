// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.elevator;

import static wmironpatriots.Constants.CANIVORE;

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
  private final TalonFX parent, child;
  private final TalonFXConfiguration motorConf;

  private final ElevatorSim simulatedElevator;

  private final double conversion = (1 / 5) * (2 * Math.PI * 0.878350);

  public ElevatorIOSim() {
    super();
    parent = new TalonFX(14, CANIVORE);
    child = new TalonFX(15, CANIVORE); // ! ID

    // register to global talonfx array
    // Robot.talonHandler.registerTalon(m_parentM);
    // Robot.talonHandler.registerTalon(m_childM);

    motorConf = new TalonFXConfiguration();
    motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConf.Slot0.withKP(0.0).withKI(0.0).withKD(0.0); // PID config
    motorConf.Slot0.withKS(0.0).withKV(0.0).withKA(0.0); // feedforward config

    motorConf.CurrentLimits.StatorCurrentLimit = 80.0;
    motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConf.CurrentLimits.SupplyCurrentLimit = 80.0;
    motorConf.CurrentLimits.SupplyCurrentLowerLimit = -80.0;
    motorConf.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConf.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    motorConf.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    motorConf.MotionMagic.MotionMagicCruiseVelocity = 0.0; // ! TODO
    motorConf.MotionMagic.MotionMagicAcceleration = 0.0;
    motorConf.MotionMagic.MotionMagicJerk = 0.0;

    // Conversion from rotations to meters
    // reduction * circumference
    // reduction is 1/5 and radius of spool radius is 0.878350 meters
    motorConf.Feedback.SensorToMechanismRatio = (1 / 5) * (2 * Math.PI * 0.878350);

    parent.getConfigurator().apply(motorConf);
    child.getConfigurator().apply(motorConf);

    child.setControl(new Follower(parent.getDeviceID(), true));
    child.optimizeBusUtilization();
    parent.optimizeBusUtilization();

    simulatedElevator =
        new ElevatorSim( // ! These are BS constants
            DCMotor.getKrakenX60Foc(2),
            Elevator.REDUCTION,
            Units.kilogramsToLbs(Elevator.MASS_KG),
            Elevator.SPOOL_RADIUS_INCHES,
            0.0,
            Elevator.RANGE_ROTS * conversion,
            true,
            0.0);
  }

  @Override
  public void periodic() {
    super.periodic();

    TalonFXSimState parentSimState = new TalonFXSimState(parent);
    simulatedElevator.setInput(parentSimState.getMotorVoltage());

    simulatedElevator.update(0.02);
    parentSimState.setRotorVelocity(
        (simulatedElevator.getVelocityMetersPerSecond())
            / motorConf.Feedback.SensorToMechanismRatio); // I have no clue if this is correct lmfao

    poseRots = simulatedElevator.getPositionMeters();
    velRPM = simulatedElevator.getVelocityMetersPerSecond();
  }

  @Override
  protected void runMotorControl(ControlRequest request) {
    parent.setControl(request);
  }

  @Override
  protected void setEncoderPose(double poseMeters) {
    // Elevator is always zeroed
  }

  @Override
  protected void stopMotors() {
    parent.setVoltage(0.0);
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    // Bah, humbug
  }
}
