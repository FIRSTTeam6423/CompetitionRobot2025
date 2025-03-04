// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Radian;
import static wmironpatriots.Constants.CANIVORE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import wmironpatriots.util.deviceUtil.MaplePhoenixUtil;

public class ModuleIOSim extends Module {
  private final TalonFX pivot, drive;
  private final TalonFXConfiguration pivotConf, driveConf;

  private final CANcoder cancoder;
  private final CANcoderConfiguration cancoderConf;

  private final VoltageOut reqPivotVolts, reqDriveVolts;
  private final VelocityTorqueCurrentFOC reqDriveVel;
  private final MotionMagicVoltage reqPivotFeedback;
  private final TorqueCurrentFOC reqDriveCurrent;

  private final BaseStatusSignal cancoderPose,
      pivotPose,
      pivotVel,
      pivotVolts,
      pivotSCurrent,
      pivotTCurrent;
  private final BaseStatusSignal drivePose, driveVel, driveVolts, driveSCurrent, driveTCurrent;

  private final SwerveModuleSimulation simulation;

  public ModuleIOSim(ModuleConfig config, SwerveModuleSimulation simulation) {
    super(config);

    pivot = new TalonFX(config.pivotID(), CANIVORE);
    drive = new TalonFX(config.driveID(), CANIVORE);
    cancoder = new CANcoder(config.cancoderID(), CANIVORE);

    pivotConf = getPivotConfig();
    driveConf = getDriveConfig();
    cancoderConf = getCANcoderConfig();

    pivotConf.MotorOutput.Inverted =
        config.pivotInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    pivotConf.Feedback.FeedbackRotorOffset = config.cancoderOffsetRads() / (2 * Math.PI);

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(cancoderConf);

    reqPivotVolts = new VoltageOut(0.0).withEnableFOC(true);
    reqDriveVolts = new VoltageOut(0.0).withEnableFOC(true);
    reqDriveVel = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    reqPivotFeedback = new MotionMagicVoltage(0.0).withEnableFOC(true);
    reqDriveCurrent = new TorqueCurrentFOC(0.0);

    cancoderPose = cancoder.getAbsolutePosition();
    pivotPose = pivot.getPosition();
    pivotVel = pivot.getVelocity();
    pivotVolts = pivot.getMotorVoltage();
    pivotSCurrent = pivot.getSupplyCurrent();
    pivotTCurrent = pivot.getTorqueCurrent();

    drivePose = drive.getPosition();
    driveVel = drive.getVelocity();
    driveVolts = drive.getMotorVoltage();
    driveSCurrent = drive.getSupplyCurrent();
    driveTCurrent = drive.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        cancoderPose,
        pivotPose,
        pivotVel,
        pivotVolts,
        pivotSCurrent,
        pivotTCurrent,
        drivePose,
        driveVel,
        driveVolts,
        driveSCurrent,
        driveTCurrent);

    pivot.optimizeBusUtilization();
    drive.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    simulation.useSteerMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            pivot, true, cancoder, false, Angle.ofBaseUnits(0.0, Radian)));
    simulation.useDriveMotorController(new MaplePhoenixUtil.TalonFXMotorControllerSim(drive, true));

    this.simulation = simulation;
  }

  @Override
  public void periodic() {
    super.periodic();

    BaseStatusSignal.refreshAll(
        cancoderPose,
        pivotPose,
        pivotVel,
        pivotVolts,
        pivotSCurrent,
        pivotTCurrent,
        drivePose,
        driveVel,
        driveVolts,
        driveSCurrent,
        driveTCurrent);

    pivotOk = true;
    driveOk = true; // TODO add a debouncer

    double rotsToRads = (2 * Math.PI);

    pivotABSPoseRads = cancoderPose.getValueAsDouble() * rotsToRads;
    pivotPoseRads = pivotPose.getValueAsDouble() * rotsToRads;
    pivotVelRadsPerSec = pivotVel.getValueAsDouble() * rotsToRads;
    pivotAppliedVolts = pivotVolts.getValueAsDouble();
    pivotSupplyCurrent = pivotSCurrent.getValueAsDouble();
    pivotTorqueCurrent = pivotTCurrent.getValueAsDouble();

    drivePoseMeters = drivePose.getValueAsDouble() * rotsToRads * WHEEL_RADIUS_METERS;
    driveVelMPS = driveVel.getValueAsDouble() * rotsToRads * WHEEL_RADIUS_METERS;
    driveAppliedVolts = driveVolts.getValueAsDouble();
    driveSupplyCurrent = driveSCurrent.getValueAsDouble();
    driveTorqueCurrent = driveTCurrent.getValueAsDouble();
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivot.setControl(reqPivotVolts.withOutput(volts));
  }

  @Override
  protected void runPivotPose(double poseRads) {
    pivot.setControl(reqPivotFeedback.withPosition(poseRads / (2 * Math.PI)));
  }

  @Override
  protected void runDriveVolts(double volts, boolean focEnabled) {
    drive.setControl(reqDriveVolts.withOutput(volts).withEnableFOC(focEnabled));
  }

  @Override
  protected void runDriveVel(double velMPS, double ff) {
    System.out.println(velMPS);
    drive.setControl(
        reqDriveVel
            .withVelocity(velMPS / (2 * Math.PI * WHEEL_RADIUS_METERS))
            .withFeedForward(ff)); // TODO yo fix feedforward
  }

  @Override
  protected void stopMotors() {
    pivot.stopMotor();
    drive.stopMotor();
  }

  @Override
  protected void motorCoastingEnabled(boolean enabled) {
    NeutralModeValue idleMode = enabled ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    pivotConf.MotorOutput.NeutralMode = idleMode;
    driveConf.MotorOutput.NeutralMode = idleMode;
  }
}
