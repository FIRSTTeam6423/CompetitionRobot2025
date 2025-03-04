package wmironpatriots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
