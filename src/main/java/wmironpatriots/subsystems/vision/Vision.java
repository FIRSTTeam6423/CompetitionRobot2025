// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import wmironpatriots.util.mechanismUtil.IronSubsystem;

public class Vision implements IronSubsystem {
  /** CONSTANTS */
  public static final CameraConfig[] CAM_CONFS =
      new CameraConfig[] {new CameraConfig("reef", new Transform3d()), new CameraConfig("source", new Transform3d())};
  public static final AprilTagFieldLayout LAYOUT =
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static record CameraConfig(String name, Transform3d loc) {}
}
