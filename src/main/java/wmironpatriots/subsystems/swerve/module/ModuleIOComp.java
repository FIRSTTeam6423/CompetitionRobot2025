// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.subsystems.swerve.Swerve;
import wmironpatriots.subsystems.swerve.TalonOdoThread;

public class ModuleIOComp extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;
  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final VoltageOut reqVolts;
  private final PositionVoltage reqPose;
  private final VelocityTorqueCurrentFOC reqVel;

  private final BaseStatusSignal pivotPose, pivotVolts, pivotCurrent;
  private final BaseStatusSignal drivePose, driveVel, driveVolts, driveCurrent, driveTorque;
  private final Queue<Double> pivotPoseQueue, drivePoseQueue;

  public ModuleIOComp(ModuleConfig config) {
    pivot = new TalonFX(config.pivotID(), MATRIXID.CANCHAN);
    drive = new TalonFX(config.driveID(), MATRIXID.CANCHAN);
    cancoder = new CANcoder(config.cancoderID(), MATRIXID.CANCHAN);

    pivotConf = getPivotConf(config.cancoderID(), config.pivotInverted());
    driveConf = getDriveConf();
    cancoderConf = getCancoderConf(config.cancoderOffsetRevs());

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(cancoderConf);

    reqVolts = new VoltageOut(0.0).withEnableFOC(true);
    reqPose = new PositionVoltage(0.0).withEnableFOC(true);
    reqVel = new VelocityTorqueCurrentFOC(0.0);

    pivotPose = pivot.getPosition();
    pivotVolts = pivot.getMotorVoltage();
    pivotCurrent = pivot.getStatorCurrent();
    drivePose = drive.getPosition();
    driveVel = drive.getVelocity();
    driveVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveTorque = drive.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, pivotVolts, pivotCurrent, driveVel, driveVolts, driveCurrent, driveTorque);
    BaseStatusSignal.setUpdateFrequencyForAll(Swerve.ODO_FREQ, pivotPose, drivePose);

    pivotPoseQueue = TalonOdoThread.getInstance().registerSignal(pivot, pivotPose);
    drivePoseQueue = TalonOdoThread.getInstance().registerSignal(drive, drivePose);

    pivot.optimizeBusUtilization(0, 0.1);
    drive.optimizeBusUtilization(0, 0.1);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        pivotPose,
        pivotVolts,
        pivotCurrent,
        drivePose,
        driveVel,
        driveVolts,
        driveCurrent,
        driveTorque);
    pivotPoseRevs = pivotPose.getValueAsDouble();
    pivotAppliedVolts = pivotVolts.getValueAsDouble();
    pivotCurrentAmps = pivotCurrent.getValueAsDouble();

    drivePoseMeters = (drivePose.getValueAsDouble() * 2 * Math.PI * WHEEL_RADIUS_METERS) / 60;
    driveVelMPS = (driveVel.getValueAsDouble() * 2 * Math.PI * WHEEL_RADIUS_METERS) / 60;
    driveAppliedVolts = driveVolts.getValueAsDouble();
    driveCurrentAmps = driveCurrent.getValueAsDouble();
    driveTorqueAmps = driveTorque.getValueAsDouble();

    odoPivotPoseRevsQueue = 
      pivotPoseQueue.stream()
        .mapToDouble(sigValue -> sigValue)
        .toArray();
    odoDrivePoseMetersQueue =
      drivePoseQueue.stream()
        .mapToDouble(sigValue -> (sigValue * 2 * Math.PI * WHEEL_RADIUS_METERS)/60)
        .toArray();
    pivotPoseQueue.clear();
    drivePoseQueue.clear();
  }

  @Override
  protected void setPivotVolts(double volts) {
    pivot.setControl(reqVolts.withOutput(volts));
  }

  @Override
  protected void setPivotPose(double poseRevs) {
    pivot.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  protected void setDriveVolts(double volts, boolean focEnabled) {
    drive.setControl(reqVolts.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  protected void setDriveVel(double velMPS, double accelMPSSqrd) {
    if (velMPS == 0 && accelMPSSqrd == 0 && Math.abs(velMPS) > 0.1) setDriveVolts(0.0, false);
    else
      drive.setControl(
          reqVel
              .withVelocity((velMPS * 60) / (2 * Math.PI * WHEEL_RADIUS_METERS))
              .withAcceleration(0.0));
  }

  @Override
  protected void stopMotors() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  protected void enableCoastMode(boolean enabled) {
    var idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotConf.MotorOutput.NeutralMode = idleMode;
    driveConf.MotorOutput.NeutralMode = idleMode;
  }
}
