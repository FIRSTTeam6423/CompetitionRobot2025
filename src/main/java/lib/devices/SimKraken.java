package lib.devices;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.drivers.CanDeviceId;

// TODO Make this a motorIO interface
public class SimKraken {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSim;

  private final TalonFX talon;

  private final VoltageOut voltReq = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentReq = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC poseReq = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velReq = new VelocityTorqueCurrentFOC(0.0);

  private final double gearing;

  private Notifier simNotifier = null;
  private double lastUpdateTimestamp = 0.0;

  public SimKraken(CanDeviceId talonId, TalonFXConfiguration talonConfig, double JKgMetersSquared, double gearing) {
    talon = new TalonFX(talonId.getId(), talonId.getBusName());
    talon.getConfigurator().apply(talonConfig);

    // Configure sim state to follow the same orientation as talon config
    this.talon.getSimState().Orientation = 
      talonConfig.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
        ? ChassisReference.CounterClockwise_Positive
        : ChassisReference.Clockwise_Positive;

    motorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        motor, 
        JKgMetersSquared, 
        gearing), 
      motor, 
      0.0,
      0.0);

      this.gearing = gearing;

      // Run sim at a faster rate so PID gains behave better
      simNotifier = new Notifier(() -> {
        updateSimState();
      });
      simNotifier.startPeriodic(0.005);
  }

  public void setAppliedVolts(double volts, boolean focEnabled) {
    talon.setControl(voltReq.withEnableFOC(focEnabled).withOutput(volts));
  }

  public void setPose(double poseRevs) {
    talon.setControl(poseReq.withPosition(poseRevs));
  }

  public void setSpeed(double speedMps) {
    talon.setControl(velReq.withVelocity(speedMps));
  }

  public void stop() {
    talon.stopMotor();
  }



  private double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

  /** Updates simulated motor */
  private void updateSimState() {
    var simState = talon.getSimState();
    double simVolts = addFriction(simState.getMotorVoltage(), 0.25); 

    motorSim.setInput(simVolts);
    double timestamp = RobotController.getFPGATime();
    motorSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    simState.setRotorVelocity((motorSim.getAngularVelocityRPM() / 60.0) * gearing);
  }
}
