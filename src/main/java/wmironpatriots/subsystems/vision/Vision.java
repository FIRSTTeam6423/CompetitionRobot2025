// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystem;

public abstract class Vision implements LoggedSubsystem {
  // * CONSTANTS
  public static record CameraConfig(
      String camID, double width, double height, double camFOV, Transform3d robotToCam) {}
  public static record PoseEstimate(Pose3d pose, Matrix<N3, N1> stdevs) {}
  
  public CameraConfig[] CAM_CONFIGS = new CameraConfig[] {};

  public AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final double[] TAG_WEIGHTS = {
    0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1, 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1
  };

  public abstract PoseEstimate[] getEstimatedPoses();
}
