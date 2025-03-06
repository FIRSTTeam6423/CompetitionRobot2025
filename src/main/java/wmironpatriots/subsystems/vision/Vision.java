// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.EstimatedRobotPose;
import wmironpatriots.util.mechanismUtil.LoggedSubsystem;

public class Vision implements LoggedSubsystem {
  /** CONSTANTS */
  public static final CameraConfig[] CAM_CONFS =
      new CameraConfig[] {
        new CameraConfig("reef", new Transform3d(), 1280, 720, 55),
        new CameraConfig("source", new Transform3d(), 1280, 720, 55)
      };

  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 0.3;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double[] TAG_WEIGHTS = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  public static record CameraConfig(
      String name, Transform3d loc, int height, int width, double fovDegrees) {}

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  public static class Field {
    public static final Distance LENGTH = Centimeters.of(1755);
    public static final Distance WIDTH = Centimeters.of(805);

    /** Returns whether the provided position is within the boundaries of the field. */
    public static boolean inField(Pose3d pose) {
      return (pose.getX() > 0
          && pose.getX() < Field.LENGTH.in(Meters)
          && pose.getY() > 0
          && pose.getY() < Field.WIDTH.in(Meters));
    }
  }

  /** LOGGED VALUES */
  public static boolean[] cameraConnected;
}
