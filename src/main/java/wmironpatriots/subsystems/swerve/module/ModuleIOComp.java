// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static wmironpatriots.subsystems.swerve.SwerveConstants.ODO_FREQ;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.subsystems.swerve.SwerveConstants;
import wmironpatriots.subsystems.swerve.SwerveConstants.ModuleConfig;

public class ModuleIOComp extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;
  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final TorqueCurrentFOC reqTorque;
  private final PositionTorqueCurrentFOC reqPose;
  private final VelocityTorqueCurrentFOC reqVel;

  private final SimpleMotorFeedforward feedforward;

  private final BaseStatusSignal pivotPoseSig, cancoderPoseSig, pivotVoltsSig, pivotCurrentSig;
  private final BaseStatusSignal drivePoseSig, driveVelSig, driveVoltsSig, driveCurrentSig, driveTorqueSig;

  public ModuleIOComp(ModuleConfig config) {
    super(config.index());
    pivot = new TalonFX(config.pivotID(), MATRIXID.CANCHAN);
    drive = new TalonFX(config.driveID(), MATRIXID.CANCHAN);
    cancoder = new CANcoder(config.cancoderID(), MATRIXID.CANCHAN);

    pivotConf = SwerveConstants.getPivotConf(config.cancoderID(), config.pivotInverted());
    driveConf = SwerveConstants.getDriveConf(config.driveInverted());
    cancoderConf =
        SwerveConstants.getCancoderConf(config.cancoderOffsetRevs(), config.encoderInverted());

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(cancoderConf);

    reqTorque = new TorqueCurrentFOC(0.0);
    reqPose = new PositionTorqueCurrentFOC(0.0);
    reqVel = new VelocityTorqueCurrentFOC(0.0);

    pivotPoseSig = pivot.getPosition();
    cancoderPoseSig = cancoder.getPosition();
    pivotVoltsSig = pivot.getMotorVoltage();
    pivotCurrentSig = pivot.getStatorCurrent();
    drivePoseSig = drive.getPosition();
    driveVelSig = drive.getVelocity();
    driveVoltsSig = drive.getMotorVoltage();
    driveCurrentSig = drive.getStatorCurrent();
    driveTorqueSig = drive.getTorqueCurrent();

    feedforward = new SimpleMotorFeedforward(5.0, 0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, pivotVoltsSig, pivotCurrentSig, driveVelSig, driveVoltsSig, driveCurrentSig, driveTorqueSig);
    BaseStatusSignal.setUpdateFrequencyForAll(ODO_FREQ, pivotPoseSig, drivePoseSig);

    pivot.optimizeBusUtilization(0, 0.1);
    drive.optimizeBusUtilization(0, 0.1);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        pivotPoseSig,
        pivotVoltsSig,
        pivotCurrentSig,
        drivePoseSig,
        driveVelSig,
        driveVoltsSig,
        driveCurrentSig,
        driveTorqueSig);

    inputs.data =
        new ModuleData(
            pivotPoseSig.getValueAsDouble(),
            cancoderPoseSig.getValueAsDouble(),
            pivotVoltsSig.getValueAsDouble(),
            pivotCurrentSig.getValueAsDouble(),
            drivePoseSig.getValueAsDouble(),
            driveVelSig.getValueAsDouble(),
            driveVoltsSig.getValueAsDouble(),
            driveCurrentSig.getValueAsDouble(),
            driveTorqueSig.getValueAsDouble());
  }

  @Override
  protected void setPivotCurrent(double amps) {
    pivot.setControl(reqTorque.withOutput(amps));
  }

  @Override
  protected void setDriveCurrent(double amps, boolean focEnabled) {
    drive.setControl(reqTorque.withOutput(amps));
  }

  @Override
  protected void setPivotPose(double poseRevs) {
    pivot.setControl(reqPose.withPosition(poseRevs));
  }

  @Override
  protected void setDriveVel(double velMPS) {
    drive.setControl(reqVel.withVelocity(velMPS).withFeedForward(feedforward.calculate(velMPS)));
  }

  @Override
  public void stopMotors() {
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
