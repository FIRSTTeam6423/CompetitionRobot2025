// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package org.frc6423.frc2025.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public class ArmIOInputs {
    public boolean pivotMotorEnabled = false;
    public boolean rollerMotorEnabled = false;

    public Rotation2d pivotPose =
        new Rotation2d(); /* Position is measured from STOWED pose (0 rads) to max out */
    public double pivotVelRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrent = 0.0;

    public double rollerVelRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrent = 0.0;
  }

  /** Run pivot motor voltage */
  public void runVolts(double voltage);

  /** Run drive motor voltage */
  public void runRollerVolts(double voltage);

  /** Run arm to specific angle */
  public void runTargetPose(Rotation2d targetAngle);

  /** Set target roller velocity in rads per second */
  public void runTargetRollerVeloctiy(double radsPerSec);
}
