// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalOutput;

public class TailIOComp extends Tail {
  private final SparkMax pivot, roller;
  private final SparkMaxConfig pivotConf, rollerConf;
  private final DigitalOutput beamUno, beamDos;

  private final SparkClosedLoopController pivotFeedback;

  public TailIOComp() {
    pivot = new SparkMax(1, MotorType.kBrushless);
    roller = new SparkMax(2, MotorType.kBrushless);

    beamUno = new DigitalOutput(0);
    beamDos = new DigitalOutput(0); // TODO SET CHANNELS

    // Configure pivot
    pivotConf = new SparkMaxConfig();

    pivotConf.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    pivotConf
        .softLimit
        .forwardSoftLimit(POSE_MIN_REVS)
        .reverseSoftLimit(POSE_OUT_RADS)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    pivotConf
        .closedLoop
        .pid(PIVOT_P, PIVOT_I, PIVOT_D)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    pivotConf
        .encoder
        .uvwAverageDepth(16)
        .uvwMeasurementPeriod(32);

    pivot.configure(pivotConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Config roller
    rollerConf = new SparkMaxConfig();

    rollerConf.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

    roller.configure(rollerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure closed loop controller
    pivotFeedback = pivot.getClosedLoopController();
  }

  @Override
  public void periodic() {
    super.periodic();
    pivotMotorOk = pivot.hasStickyFault();
    rollerMotorOk = roller.hasStickyFault();

    beamITriggered = beamUno.get();
    beamIITriggered = beamDos.get();

    pivotPoseRads = pivot.getEncoder().getPosition();
    pivotVelRPM = pivot.getEncoder().getVelocity();
    pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    pivotSupplyCurrentAmps = pivot.getOutputCurrent();

    rollerVelRPM = roller.getEncoder().getVelocity();
    rollerAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    rollerSupplyCurrentAmps = roller.getOutputCurrent();
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivot.setVoltage(volts);
  }

  @Override
  protected void runPivotSetpoint(double setpointRadians) {
    pivotFeedback.setReference(setpointRadians, ControlType.kMAXMotionPositionControl);
  }

  @Override
  protected void runRollerSpeed(double speed) {
    roller.set(speed);
  }

  @Override
  protected void setEncoderPose(double poseRads) {
    pivot.getEncoder().setPosition(poseRads);
  }

  @Override
  protected void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  protected void stopRollers() {
    roller.stopMotor();
  }

  @Override
  protected void pivotCoastingEnabled(boolean enabled) {
    IdleMode idleMode = enabled ? IdleMode.kCoast : IdleMode.kBrake;
    pivotConf.idleMode(idleMode);
    pivot.configure(pivotConf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}