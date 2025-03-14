// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import wmironpatriots.Constants.MATRIXID;

public class ModuleIOComp extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;
  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final VoltageOut reqVolts;
  private final PositionVoltage reqPose;
  private final VelocityTorqueCurrentFOC reqVel;

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
  protected void setDriveVolts(double volts) {
    drive.setControl(reqVolts.withOutput(volts));
  }

  @Override
  protected void setDriveVel(double velMPS, double accelMPSSqrd) {
    if (velMPS == 0 && accelMPSSqrd == 0 && Math.abs(velMPS) > 0.1) setDriveVolts(0.0);
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
