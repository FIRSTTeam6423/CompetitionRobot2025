// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import wmironpatriots.subsystems.swerve.Swerve;

/** WIP drive to pose command */
public class DriveToPose extends Command {
  private final Swerve swerve;

  private final Supplier<Pose2d> poseSupplier;

  /**
   * Constructs a new {@link DriveToPose} command
   *
   * @param poseSupplier desired position supplier
   */
  public DriveToPose(Supplier<Pose2d> poseSupplier) {
    swerve = Swerve.getInstance();
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  public boolean atPose() {
    return false; // ! PLACEHOLDER
  }
}
