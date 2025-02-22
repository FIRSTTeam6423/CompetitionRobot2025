// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TailIOSim extends Tail {
  private final SingleJointedArmSim tailSim;
  private double pivotInputVoltage;
  private final ProfiledPIDController pivotFeedback;
  private final ArmFeedforward pivotFeedforward;

  public TailIOSim() {
    super();
    // pivotPoseRads = new Random().nextDouble(POSE_MAX_RADS); // generates random starting pose
    tailSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            REDUCTION,
            1.0,
            Units.inchesToMeters(LENGTH_INCHES),
            POSE_OUT_RADS,
            POSE_OUT_RADS,
            true,
            POSE_OUT_RADS);

    pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    pivotFeedback =
        new ProfiledPIDController(100, 0, 6.0, new TrapezoidProfile.Constraints(10, 10));
  }

  @Override
  public void periodic() {
    tailSim.update(0.02);

    pivotMotorOk = true;
    rollerMotorOk = true;

    beamUnoTriggered = true;
    beamDosTriggered = true;

    pivotPoseRads = tailSim.getAngleRads();
    pivotVelRPM = Units.radiansPerSecondToRotationsPerMinute(tailSim.getAngleRads());
    pivotAppliedVolts = pivotInputVoltage;
    pivotSupplyCurrentAmps = tailSim.getCurrentDrawAmps();

    mechBase.setAngle(pivotPoseRads * (180 / Math.PI));
  }

  @Override
  protected void runPivotVolts(double volts) {
    pivotAppliedVolts = volts;
    tailSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  protected void runPivotSetpoint(double setpointRadians) {
    runPivotVolts(
        pivotFeedback.calculate(pivotPoseRads, setpointRadians)
            + pivotFeedforward.calculate(
                pivotFeedback.getSetpoint().position, pivotFeedback.getSetpoint().velocity));
    System.out.println(pivotFeedback.getSetpoint().velocity);
  }

  @Override
  protected void runRollerSpeed(double speed) {} // No rollers

  @Override
  protected void setEncoderPose(double poseRads) {}

  @Override
  protected void stopPivot() {
    runPivotVolts(0);
  }

  @Override
  protected void stopRollers() {} // no rollers

  @Override
  protected void pivotCoasting(boolean enabled) {} // Sim motors can't coast
}
