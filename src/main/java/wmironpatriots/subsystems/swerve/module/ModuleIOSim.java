// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Rotation;
import static wmironpatriots.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import java.util.Queue;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.subsystems.swerve.TalonOdoThread;
import wmironpatriots.utils.simUtils.MaplePhoenixUtil;

public class ModuleIOSim extends Module {
  private final TalonFX pivot, drive;
  private final CANcoder cancoder;
  private final TalonFXConfiguration pivotConf, driveConf;
  private final CANcoderConfiguration cancoderConf;

  private final TorqueCurrentFOC reqTorque;
  private final PositionTorqueCurrentFOC reqPose;
  private final VelocityTorqueCurrentFOC reqVel;

  private final SimpleMotorFeedforward feedforward;

  private final BaseStatusSignal pivotPose, pivotVolts, pivotCurrent;
  private final BaseStatusSignal drivePose, driveVel, driveVolts, driveCurrent, driveTorque;
  private final Queue<Double> pivotPoseQueue, drivePoseQueue;

  private final SwerveModuleSimulation simulation;

  public ModuleIOSim(ModuleConfig config, SwerveModuleSimulation simulation) {
    pivot = new TalonFX(config.pivotID(), MATRIXID.CANCHAN);
    drive = new TalonFX(config.driveID(), MATRIXID.CANCHAN);
    cancoder = new CANcoder(config.cancoderID(), MATRIXID.CANCHAN);

    pivotConf = getPivotConf(config.cancoderID(), config.pivotInverted());
    driveConf = getDriveConf(config.driveInverted());
    cancoderConf = getCancoderConf(config.cancoderOffsetRevs(), config.encoderInverted());

    pivot.getConfigurator().apply(pivotConf);
    drive.getConfigurator().apply(driveConf);
    cancoder.getConfigurator().apply(cancoderConf);

    reqTorque = new TorqueCurrentFOC(0.0);
    reqPose = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    reqVel = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

    pivotPose = pivot.getPosition();
    pivotVolts = pivot.getMotorVoltage();
    pivotCurrent = pivot.getStatorCurrent();
    drivePose = drive.getPosition();
    driveVel = drive.getVelocity();
    driveVolts = drive.getMotorVoltage();
    driveCurrent = drive.getStatorCurrent();
    driveTorque = drive.getTorqueCurrent();

    feedforward = new SimpleMotorFeedforward(5.0, 0.134);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, pivotVolts, pivotCurrent, driveVel, driveVolts, driveCurrent, driveTorque);
    BaseStatusSignal.setUpdateFrequencyForAll(ODO_FREQ, pivotPose, drivePose);

    pivotPoseQueue = TalonOdoThread.getInstance().registerSignal(pivot, pivotPose);
    drivePoseQueue = TalonOdoThread.getInstance().registerSignal(drive, drivePose);

    pivot.optimizeBusUtilization(0, 0.1);
    drive.optimizeBusUtilization(0, 0.1);

    this.simulation = simulation;
    simulation.useSteerMotorController(
        new MaplePhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            pivot,
            config.pivotInverted(),
            cancoder,
            false,
            Angle.ofBaseUnits(config.cancoderOffsetRevs(), Rotation)));
    simulation.useDriveMotorController(new MaplePhoenixUtil.TalonFXMotorControllerSim(drive, true));
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

    drivePoseMeters = drivePose.getValueAsDouble();
    driveVelMPS = driveVel.getValueAsDouble();
    driveAppliedVolts = driveVolts.getValueAsDouble();
    driveCurrentAmps = driveCurrent.getValueAsDouble();
    driveTorqueAmps = driveTorque.getValueAsDouble();

    odoPivotPoseRevsQueue = pivotPoseQueue.stream().mapToDouble(sigValue -> sigValue).toArray();
    odoDrivePoseMetersQueue = drivePoseQueue.stream().mapToDouble(sigValue -> sigValue).toArray();
    pivotPoseQueue.clear();
    drivePoseQueue.clear();
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
