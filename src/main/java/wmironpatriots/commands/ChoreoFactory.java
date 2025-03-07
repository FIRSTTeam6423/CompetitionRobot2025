// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import wmironpatriots.subsystems.CommandSwerveDrivetrain;

public class ChoreoFactory {
  private final CommandSwerveDrivetrain drivetrain;

  private final AutoFactory factory;
  private final SwerveRequest.RobotCentric controller;
  private final PIDController linearFeedback = new PIDController(5.0, 0.0, 0.0);
  private final PIDController angularFeedback = new PIDController(5.0, 0.0, 0.0);
  private final PIDController headingFeedback = new PIDController(5.0, 0.0, 0.0);

  public ChoreoFactory(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    controller =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    factory =
        new AutoFactory(
            drivetrain::getPose, drivetrain::resetPose, this::followTraj, true, drivetrain);

    SmartDashboard.putData("LinearPID", linearFeedback);
    SmartDashboard.putData("AngularPID", angularFeedback);
    SmartDashboard.putData("HeadingPID", headingFeedback);
  }

  public Command followTraj(SwerveSample sample) {
    return Commands.run(
        () -> {
          Pose2d pose = drivetrain.getPose();
          drivetrain.applyRequest(
              () ->
                  controller
                      .withVelocityX(sample.vx + linearFeedback.calculate(pose.getX(), sample.x))
                      .withVelocityY(sample.vy + linearFeedback.calculate(pose.getY(), sample.y))
                      .withRotationalRate(
                          sample.omega
                              + headingFeedback.calculate(
                                  pose.getRotation().getRadians(), sample.heading)));
        },
        drivetrain);
  }

  public AutoFactory getFactor() {
    return factory;
  }

  public Command getMove() {
    return Commands.sequence(factory.resetOdometry("move"), factory.trajectoryCmd("move"));
  }

  public AutoChooser getChooser() {
    AutoChooser chooser = new AutoChooser();
    chooser.addCmd("move", this::getMove);
    return chooser;
  }
}
