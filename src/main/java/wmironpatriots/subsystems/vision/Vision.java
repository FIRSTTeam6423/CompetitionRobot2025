// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystem;

public abstract class Vision implements LoggedSubsystem {
  // * CONSTANTS
  public static record CameraConfig(
      String camID, double width, double height, double camFOV, Transform3d robotToCam) {}

  public static record PoseEstimate(EstimatedRobotPose pose, Matrix<N3, N1> stdevs) {}

  public CameraConfig[] CAM_CONFIGS =
      new CameraConfig[] {
        new CameraConfig(
            "camchan",
            0,
            0,
            0,
            new Transform3d(
                new Translation3d(Inches.of(-11.74), Inches.of(-9.85), Inches.of(11.62)),
                // new Rotation3d(0.0, 0.506145275, -1.977632338725)))
                new Rotation3d(0.0, 0.5061, 2.7348)))
      };

  public AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double[] TAG_WEIGHTS = {
    0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1, 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1
  };

  public abstract PoseEstimate[] getEstimatedPoses();
}
