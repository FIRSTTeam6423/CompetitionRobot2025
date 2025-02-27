// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static wmironpatriots.Constants.CANIVORE;
import static wmironpatriots.Constants.TICKSPEED;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOSim extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;

  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final DCMotorSim pivotSim, driveSim;

  public ModuleIOSim(ModuleConfig config) {
    super(config);
    pivot = new TalonFX(config.kPivotID, CANIVORE);
    drive = new TalonFX(config.kDriveID, CANIVORE);
    cancoder = new CANcoder(config.kPivotABSID);

    // Robot.talonHandler.registerTalon(m_pivotM);
    // Robot.talonHandler.registerTalon(m_driveM);

    pivotConf = config.kPivotConfigTalonFX;
    driveConf = config.kDriveConfigTalonFX;
    cancoderConf = config.kCANcoderConfig;

    cancoderConf.MagnetSensor.MagnetOffset = config.kPivotOffset.getRotations();
    cancoderConf.MagnetSensor.SensorDirection =
        config.kPivotInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    pivotConf.Slot0.kP = 100.0;
    pivotConf.Slot0.kI = 0.0;
    pivotConf.Slot0.kD = 0.0;

    pivotConf.Slot0.kV = 0.0;
    pivotConf.Slot0.kA = 0.0;
    pivotConf.Slot0.kS = 0.0;

    pivotConf.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConf.Feedback.FeedbackRemoteSensorID = config.kPivotABSID;

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(config.kCANcoderConfig);

    DCMotor pivotDCMotor = DCMotor.getKrakenX60(1);
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotDCMotor, 1.0, config.kPivotReduction),
            pivotDCMotor);

    DCMotor driveDCMotor = DCMotor.getKrakenX60(1);
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveDCMotor, 1.0, config.kDriveReduction),
            driveDCMotor);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Update simulated hardware
    TalonFXSimState pivotSimState = new TalonFXSimState(pivot);
    TalonFXSimState driveSimState = new TalonFXSimState(drive);
    CANcoderSimState cancoderSimState = new CANcoderSimState(cancoder);
    cancoderSimState.setRawPosition(pivotABSPose.getRotations());

    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    driveSim.setInputVoltage(driveSimState.getMotorVoltage());

    pivotSim.update(TICKSPEED);
    driveSim.update(TICKSPEED);

    pivotSimState.setRotorVelocity(
        (pivotSim.getAngularVelocityRPM() / 60) * config.kPivotReduction);
    pivotSimState.setRawRotorPosition(
        (pivotSim.getAngularVelocityRPM() / 60) * config.kPivotReduction);
    driveSimState.setRotorVelocity(
        (driveSim.getAngularVelocityRPM() / 60) * config.kDriveReduction);
    driveSimState.setRawRotorPosition(
        (driveSim.getAngularVelocityRPM() / 60) * config.kDriveReduction);

    // Update logged values
    pivotOk = true;
    driveOk = true;

    pivotABSPose = Rotation2d.fromRadians(pivotSim.getAngularPositionRad());
    pivotPose = pivotABSPose;
    pivotVelRadsPerSec = pivotSim.getAngularVelocityRadPerSec();
    pivotAppliedVolts = pivotSim.getInputVoltage();
    pivotSupplyCurrent = pivotSim.getCurrentDrawAmps();
    pivotTorqueCurrent = pivotSim.getTorqueNewtonMeters();

    drivePoseRads = driveSim.getAngularPositionRad();
    driveVelRadsPerSec = driveSim.getAngularVelocityRadPerSec();
    driveAppliedVolts = driveSim.getInputVoltage();
    driveSupplyCurrent = driveSim.getCurrentDrawAmps();
    driveTorqueCurrent = driveSim.getTorqueNewtonMeters();
  }

  @Override
  protected void runPivotControl(ControlRequest request) {
    pivot.setControl(request);
  }

  @Override
  protected void runDriveControl(ControlRequest request) {
    drive.setControl(request);
  }

  @Override
  protected void stopMotors() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotConf.MotorOutput.NeutralMode = idleMode;
    driveConf.MotorOutput.NeutralMode = idleMode;

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
  }
}
