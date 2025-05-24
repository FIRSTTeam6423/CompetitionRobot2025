// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.module;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lib.utils.TalonFxUtil;
import wmironpatriots.subsystems.Swerve.SwerveConstants;
import wmironpatriots.subsystems.Swerve.SwerveConstants.ModuleConfig;

public class ModuleHardwareSim implements ModuleHardware {
  private final int index;

  private final DCMotor pivotModel = DCMotor.getKrakenX60Foc(1);
  private final DCMotor driveModel = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim pivotSim, driveSim;
  private final TalonFX drive;
  private final TalonFXConfiguration driveCfg;

  private final VoltageOut voltReq = new VoltageOut(0.0);
  private final PositionTorqueCurrentFOC poseReq = new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velReq = new VelocityTorqueCurrentFOC(0.0);

  private Notifier simNotifier = null;
  private double lastUpdateTimestamp = 0.0;
  private double pivotAppliedVolts = 0.0;

  private final PIDController pivotFeedback = new PIDController(100.0, 0.0, 0.0);

  public ModuleHardwareSim(ModuleConfig moduleConfig) {
    index = moduleConfig.index();

    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotModel, 0.004, 21.428571428571427),
            pivotModel,
            0.0,
            0.0);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveModel, 0.025, 6.122448979591837),
            driveModel,
            0.0,
            0.0);

    drive = new TalonFX(moduleConfig.driveId().getId(), moduleConfig.driveId().getBusName());

    // Drive Configs
    driveCfg = TalonFxUtil.getDefaultTalonFxCfg();

    driveCfg.MotorOutput.Inverted =
        moduleConfig.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveCfg.CurrentLimits.StatorCurrentLimit = 120.0;
    driveCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    driveCfg.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    driveCfg.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    driveCfg.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    driveCfg.ClosedLoopGeneral.ContinuousWrap = true;
    driveCfg.Feedback.SensorToMechanismRatio = 6.122448979591837; // TODO
    driveCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    driveCfg.Slot0.kP = 35.0;
    driveCfg.Slot0.kD = 0.0;
    driveCfg.Slot0.kA = 0.0;
    driveCfg.Slot0.kV = 0.0;
    driveCfg.Slot0.kS = 5.0;

    drive.getConfigurator().apply(driveCfg);

    drive.getSimState().Orientation =
        driveCfg.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
            ? ChassisReference.CounterClockwise_Positive
            : ChassisReference.Clockwise_Positive;

    // Run sim at a faster rate so PID gains behave better
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.startPeriodic(0.005);
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

  private void updateSimState() {
    var driveSimState = drive.getSimState();
    double driveSimVolts = addFriction(driveSimState.getMotorVoltage(), 0.25);

    driveSim.setInput(driveSimVolts);
    double timestamp = RobotController.getFPGATime();
    pivotSim.update(timestamp - lastUpdateTimestamp);
    driveSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    System.out.println(pivotSim.getAngularPositionRotations());
    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60.0) * driveCfg.Feedback.SensorToMechanismRatio);
  }

  @Override
  public LoggableState getLoggableState() {
    System.out.println("Hejwkhejfs");
    return new LoggableState(
        index,
        true,
        pivotSim.getAngularPositionRotations(),
        poseReq.Position,
        pivotAppliedVolts,
        pivotSim.getCurrentDrawAmps(),
        pivotSim.getTorqueNewtonMeters() / pivotModel.KtNMPerAmp,
        true,
        drive.getSimState().getMotorVoltage(),
        driveSim.getAngularVelocityRadPerSec() * SwerveConstants.WHEEL_RADIUS.in(Meters),
        velReq.Velocity,
        drive.getSimState().getMotorVoltage(),
        driveSim.getCurrentDrawAmps(),
        pivotSim.getTorqueNewtonMeters() / driveModel.KtNMPerAmp,
        true,
        pivotSim.getAngularPositionRad());
  }

  @Override
  public void setPivotAppliedVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotSim.setInputVoltage(12.0);
  }

  @Override
  public void setDriveAppliedVolts(double volts) {
    drive.setControl(voltReq.withOutput(volts));
  }

  @Override
  public void setPivotSetpointPose(double poseRevs) {
    setPivotAppliedVolts(pivotFeedback.calculate(pivotSim.getAngularPositionRotations(), poseRevs));
  }

  @Override
  public void setDriveSetpointSpeed(double speedMps) {
    drive.setControl(velReq.withVelocity(speedMps));
  }

  @Override
  public void stop() {
    pivotSim.setInputVoltage(0.0);
    drive.stopMotor();
  }

  @Override
  public void coastingEnabled(boolean enabled) {
    // Simulated motors can't coast lols
  }
}
