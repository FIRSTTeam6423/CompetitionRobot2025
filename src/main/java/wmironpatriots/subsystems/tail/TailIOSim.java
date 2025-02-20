// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Random;

public class TailIOSim extends Tail {
  private final SingleJointedArmSim tailSim;
  private double pivotInputVoltage;
  private final PIDController pivotFeedback;

  private final double initPoseRads;

  public TailIOSim() {
    initPoseRads = new Random().nextDouble(POSE_MAX_RADS); // generates random starting pose
    tailSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            REDUCTION,
            3.0,
            Units.inchesToMeters(LENGTH_INCHES),
            0.0,
            POSE_MAX_RADS,
            true,
            initPoseRads);

    pivotFeedback = new PIDController(1, 0, 0);
  }

  @Override
  public void periodic() {
    pivotMotorOk = true;
    rollerMotorOk = true;

    beamUnoTriggered = false;
    beamDosTriggered = false;

    pivotPoseRads = tailSim.getAngleRads();
    pivotVelRPM = Units.radiansPerSecondToRotationsPerMinute(tailSim.getAngleRads());
    pivotAppliedVolts = pivotInputVoltage;
    pivotSupplyCurrentAmps = tailSim.getCurrentDrawAmps();
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12, 12);
    tailSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  protected void runPivotSetpoint(double setpointRadians) {
    pivotSetpointRads = setpointRadians;
    runPivotVolts(pivotFeedback.calculate(pivotPoseRads, setpointRadians));
  }

  @Override
  protected void runRollerSpeed(double speed) {} // No rollers

  @Override
  protected void setEncoderPose(double poseMeters) {}

  @Override
  protected void stopPivot() {
    runPivotVolts(0);
  }

  @Override
  protected void stopRollers() {} // no rollers

  @Override
  protected void pivotCoasting(boolean enabled) {} // Sim motors can't coast
}
