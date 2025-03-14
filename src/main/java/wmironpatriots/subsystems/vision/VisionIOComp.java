// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOComp extends Vision {
  private final PhotonCamera[] cams;
  private final PhotonPoseEstimator[] estimators;

  public VisionIOComp() {
    cams = new PhotonCamera[CAM_CONFIGS.length];
    estimators = new PhotonPoseEstimator[CAM_CONFIGS.length];

    for (int i = 0; i < CAM_CONFIGS.length; i++) {
      CameraConfig conf = CAM_CONFIGS[i];
      cams[i] = new PhotonCamera(conf.camID());
      estimators[i] = new PhotonPoseEstimator(
        LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        conf.robotToCam());
    }
  }

  @Override
  public PoseEstimate[] getEstimatedPoses() {
    ArrayList<PoseEstimate> estimates = new ArrayList<>();
    for (int i = 0; i < cams.length; i++) {  
      // Gets all new cam results since last call
      var estimator = estimators[i];
      var results = cams[i].getAllUnreadResults();

      // Go through each update
      Optional<EstimatedRobotPose> estimate = Optional.empty();
      for (var result : results) {
        estimate = estimator.update(result);
        estimate.ifPresent(e -> estimates.add(new PoseEstimate(e, getStdevs(e.estimatedPose.toPose2d(), result))));
      }
    }

    return estimates.toArray(PoseEstimate[]::new);
  }

  public Matrix<N3, N1> getStdevs(Pose2d estimated, PhotonPipelineResult result) {
    var estStdDevs = SINGLE_TAG_STD_DEVS;
    var targets = result.getTargets();
    int numTags = 0;
    double avgDist = 0;
    double avgWeight = 0;
    for (var tgt : targets) {
      var tagPose = LAYOUT.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimated.getTranslation());
      avgWeight += TAG_WEIGHTS[tgt.getFiducialId() - 1];
    }
    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;
    avgWeight /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = MULTIPLE_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    estStdDevs = estStdDevs.times(avgWeight);

    return estStdDevs;
  }
}
