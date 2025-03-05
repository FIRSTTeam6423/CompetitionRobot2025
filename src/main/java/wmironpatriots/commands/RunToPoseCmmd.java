// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import wmironpatriots.subsystems.swerve.Swerve;

public class RunToPoseCmmd extends Command {
  private final Swerve swerve;
  private final Supplier<Pose2d> desiredPose, currentPose;

  public RunToPoseCmmd(Swerve swerve, Supplier<Pose2d> desiredPose, Supplier<Pose2d> currentPose) {
    this.swerve = swerve;
    this.desiredPose = desiredPose;
    this.currentPose = currentPose;
  }

  @Override
  public void initialize() {
    ChassisSpeeds currentFieldRelativeVelocties = swerve.getFieldRelativeVelocities();
  }
}
