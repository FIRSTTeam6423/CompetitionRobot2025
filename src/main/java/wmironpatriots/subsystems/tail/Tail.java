// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.tail;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import wmironpatriots.Robot;

public abstract class Tail extends SubsystemBase {
  /** CONSTANTS */
  public static final double REDUCTION = 50;

  public static final double MASS_KG = 0.0; // TODO CALCULATE VALUE IN CAD
  public static final double LENGTH_INCHES = 10.0; // TODO CALCULATE VALUE IN CAD
  public static final double JKG_METERS_SQRD = 3.0; // TODO CLACULATE VALUE IN CAD

  public static final double POSE_IN_RADS = Math.PI / 2;
  public static final double POSE_OUT_RADS = Math.PI / 6;
  public static final double POSE_LNONFOUR_RADS = POSE_IN_RADS;
  public static final double POSE_L4_RADS = POSE_OUT_RADS;

  // Roller speeds
  public static final double INTAKING_SPEEDS = 5;
  public static final double OUTTAKING_SPEEDS = 5;

  public static final double CURRENT_LIMIT = 40.0;

  /** LOGGED VALUES */
  @Log protected boolean isZeroed = false;

  @Log protected boolean pivotMotorOk = false;
  @Log protected boolean rollerMotorOk = false;

  @Log protected boolean beamUnoTriggered = false;
  @Log protected boolean beamDosTriggered = false;

  @Log protected double pivotSetpointRads;
  @Log protected double pivotPoseRads;
  @Log protected double pivotVelRPM;
  @Log protected double pivotAppliedVolts;
  @Log protected double pivotSupplyCurrentAmps;

  @Log protected double rollerVelRPM;
  @Log protected double rollerAppliedVolts;
  @Log protected double rollerSupplyCurrentAmps;

  /** Tail widget */
  protected final Mechanism2d mechCanvas = new Mechanism2d(24, 24);

  protected final MechanismRoot2d mechAnchor = mechCanvas.getRoot("Tail", 0, 5);
  protected final MechanismLigament2d mechBase =
      mechAnchor.append(new MechanismLigament2d("Arm", LENGTH_INCHES, 0));
  protected final MechanismLigament2d mechFrontSide =
      mechBase.append(new MechanismLigament2d("FrontSide", LENGTH_INCHES / 4.5, -90));
  protected final MechanismLigament2d mechBackSide =
      mechFrontSide.append(
          new MechanismLigament2d(
              "bah",
              Math.hypot(mechBase.getLength(), mechFrontSide.getLength()),
              180
                  + (Math.atan(mechBase.getLength() / mechFrontSide.getLength())
                      * (180)
                      / Math.PI)));

  public Tail() {
    SmartDashboard.putData("Tail/Mech2d", this.mechCanvas);
  }

  /** Runs target position in radians from current zeroed pose */
  public Command setTargetPoseCommand(double pose) {
    return this.run(
        () -> {
          pivotSetpointRads = pose;
          runPivotSetpoint(pose);
        });
  }

  /** Runs rollers at specific speed */
  public Command runRollersCommand(double speed) {
    return this.run(
        () -> {
          runRollerSpeed(speed);
        });
  }

  /** Zeroes tail pivot at current pose */
  public Command zeroPoseCommmand() {
    return this.run(
        () -> {
          isZeroed = true;
          setEncoderPose(POSE_OUT_RADS);
        });
  }

  /** Runs pivot backwards until current spikes above threshold */
  public Command runPoseZeroingCommand() {
    return this.run(() -> runPivotVolts(0.5))
        .until(() -> pivotSupplyCurrentAmps > 20.0)
        .finallyDo(
            (interrupted) -> {
              runPivotVolts(0.0);
              setEncoderPose(POSE_OUT_RADS);
              isZeroed = true;
            });
  }

  /** Returns pivot pose in radians */
  public double getPose() {
    return pivotPoseRads;
  }

  /** Checks if pivot pose is +- PI/8 rads from specified pose */
  public boolean inRange(double pose) {
    return Math.abs(pose - pivotPoseRads) < Math.PI / 8;
  }

  /** Checks if both tail beambreaks are triggered */
  public boolean hasCoral(boolean simReturn) {
    return (Robot.isReal()) ? beamUnoTriggered && beamDosTriggered : simReturn;
  }

  /** HARDWARE METHODS */
  /** Run pivot voltage */
  protected abstract void runPivotVolts(double volts);

  /** Run pivot to specific setpoint in rads */
  protected abstract void runPivotSetpoint(double setpointRadians);

  /** Set roller speed */
  protected abstract void runRollerSpeed(double speed);

  /** Reset encoder to specific pose in rads */
  protected abstract void setEncoderPose(double poseRads);

  /** Zeros pivot at current pose */
  private void resetEncoderPose() {
    setEncoderPose(0.0);
  }

  /** Stop pivot motor */
  protected abstract void stopPivot();

  /** Stop roller motor */
  protected abstract void stopRollers();

  /** Set tail to coast mode for easier movement */
  protected abstract void pivotCoasting(boolean enabled);
}
