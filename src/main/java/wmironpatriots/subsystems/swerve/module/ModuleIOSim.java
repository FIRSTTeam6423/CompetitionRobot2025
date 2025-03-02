// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static wmironpatriots.Constants.CANIVORE;
import static wmironpatriots.Constants.kTickSpeed;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import wmironpatriots.util.swerveUtil.ModuleConfig;

public class ModuleIOSim extends Module {
  private final TalonFX drive;
  private final TalonFXConfiguration driveConf;
  private double pivotAppliedVolts;
  private final VoltageOut reqMotorVolts =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC reqMotorVel =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);

  private final PIDController pivotFeedback = new PIDController(100.0, 0.0, 0.0);

  private final DCMotorSim pivotSim, driveSim;

  public ModuleIOSim(ModuleConfig config) {
    super(config);

    drive = new TalonFX(config.driveID, CANIVORE);
    driveConf = config.driveConfig;
    drive.getConfigurator().apply(driveConf);

    DCMotor pivotDCMotor = DCMotor.getKrakenX60(1);
    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotDCMotor, 0.025, config.pivotReduction),
            pivotDCMotor,
            0,
            0);

    DCMotor driveDCMotor = DCMotor.getKrakenX60(1);
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveDCMotor, 0.004, config.pivotReduction),
            driveDCMotor,
            0,
            0);

    pivotFeedback.enableContinuousInput(0, 0.5);
  }

  @Override
  public void periodic() {
    super.periodic();
    // Update simulated hardware
    TalonFXSimState driveSimState = new TalonFXSimState(drive);
    driveSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    driveSim.setInputVoltage(driveSimState.getMotorVoltage());

    pivotSim.update(kTickSpeed);
    driveSim.update(kTickSpeed);

    driveSimState.setRotorVelocity((driveSim.getAngularVelocityRPM() / 60) * config.pivotReduction);

    // Update logged values
    pivotOk = true;
    driveOk = true;

    pivotABSPoseRads = pivotSim.getAngularPositionRad();
    pivotPoseRads = pivotSim.getAngularPositionRad();
    pivotVelRadsPerSec = pivotSim.getAngularVelocityRadPerSec();
    pivotAppliedVolts = pivotSim.getInputVoltage();
    pivotSupplyCurrent = pivotSim.getCurrentDrawAmps();
    pivotTorqueCurrent = pivotSim.getTorqueNewtonMeters();

    drivePoseMeters = driveSim.getAngularPositionRad() * config.wheelRadiusMeters;
    driveVelMPS = driveSim.getAngularVelocityRadPerSec();
    driveAppliedVolts = driveSim.getInputVoltage();
    driveSupplyCurrent = driveSim.getCurrentDrawAmps();
    driveTorqueCurrent = driveSim.getTorqueNewtonMeters();
  }

  @Override
  protected void runPivotPose(double poseRads) {
    runPivotVolts(
        pivotFeedback.calculate(pivotSim.getAngularPositionRotations(), poseRads / (Math.PI * 2)));
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  protected void runDriveVolts(double volts, boolean focEnabled) {
    drive.setControl(reqMotorVolts.withOutput(volts).withEnableFOC(true));
  }

  @Override
  protected void runDriveVel(double velMPS, double torqueff) {
    System.out.println(torqueff);
    drive.setControl(reqMotorVel.withVelocity(velMPS).withFeedForward(torqueff));
  }

  @Override
  protected void stopMotors() {
    drive.stopMotor();
    driveSim.setInputVoltage(0.0);
    pivotSim.setInputVoltage(0.0);
  }

  @Override
  protected void motorCoasting(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    driveConf.MotorOutput.NeutralMode = idleMode;

    drive.getConfigurator().apply(driveConf);
  }
}
