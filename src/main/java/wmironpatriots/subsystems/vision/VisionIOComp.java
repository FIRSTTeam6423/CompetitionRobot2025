// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import wmironpatriots.Robot;
import wmironpatriots.subsystems.vision.Vision.PoseEstimate;

public class VisionIOComp extends Vision {
  private final PhotonCamera[] cameras;
  private final PhotonCameraSim[] simulatedCams;

  private final PhotonPoseEstimator[] estimators;
  private final PhotonPipelineResult[] prevResults;

  private VisionSystemSim visionSim;

  public VisionIOComp(CameraConfig[] configs) {
    cameras = new PhotonCamera[configs.length];
    simulatedCams = new PhotonCameraSim[configs.length];
    estimators = new PhotonPoseEstimator[configs.length];
    prevResults = new PhotonPipelineResult[configs.length];

    for (int i = 0; i < configs.length; i++) {
      cameras[i] = new PhotonCamera(configs[i].name());
      simulatedCams[i] = new PhotonCameraSim(cameras[i]);

      estimators[i] =
          new PhotonPoseEstimator(
              LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, configs[i].loc());

      prevResults[i] = new PhotonPipelineResult();
    }

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("v1Ultrakill");
      visionSim.addAprilTags(LAYOUT);

      for (int i = 0; i < configs.length; i++) {
        CameraConfig config = configs[i];
        var prop = new SimCameraProperties();
        prop.setCalibration(
            config.width(), config.height(), Rotation2d.fromDegrees(config.fovDegrees()));
        prop.setCalibError(0.15, 0.05);
        prop.setFPS(45);
        prop.setAvgLatencyMs(12);
        prop.setLatencyStdDevMs(3.5);

        PhotonCameraSim camSim = new PhotonCameraSim(cameras[i], prop);
        camSim.setMaxSightRange(5);
        camSim.enableRawStream(true);
        camSim.enableProcessedStream(true);
        camSim.enableDrawWireframe(true);

        visionSim.addCamera(camSim, configs[i].loc());
        simulatedCams[i] = camSim;
      }
    }
  }

  public PoseEstimate[] estimatedGlobalPoses() {
    List<PoseEstimate> estimates = new ArrayList<>();
    for (int i = 0; i < estimators.length; i++) {
      var unreadChanges = cameras[i].getAllUnreadResults();
      Optional<EstimatedRobotPose> estimate = Optional.empty();

      int unreadLength = unreadChanges.size();

      // feeds latest result for visualization; multiple different pos breaks getSeenTags()
      prevResults[i] = unreadLength == 0 ? prevResults[i] : unreadChanges.get(unreadLength - 1);

      for (int j = 0; j < unreadLength; j++) {
        var change = unreadChanges.get(j);
        estimate = estimators[i].update(change);
        // ("estimates present " + i, estimate.isPresent());
        estimate
            .filter(
                f ->
                    Field.inField(f.estimatedPose)
                        && Math.abs(f.estimatedPose.getZ()) < MAX_HEIGHT
                        && Math.abs(f.estimatedPose.getRotation().getX()) < MAX_ANGLE
                        && Math.abs(f.estimatedPose.getRotation().getY()) < MAX_ANGLE)
            .ifPresent(
                e ->
                    estimates.add(
                        new PoseEstimate(
                            e, estimationStdDevs(e.estimatedPose.toPose2d(), change))));
      }
    }
    return estimates.toArray(PoseEstimate[]::new);
  }

  public Pose3d[] getSeenTags() {
    return Arrays.stream(prevResults)
        .flatMap(c -> c.targets.stream())
        .map(PhotonTrackedTarget::getFiducialId)
        .map(LAYOUT::getTagPose)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  public Matrix<N3, N1> estimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult) {
    var estStdDevs = SINGLE_TAG_STD_DEVS;
    var targets = pipelineResult.getTargets();
    int numTags = 0;
    double avgDist = 0;
    double avgWeight = 0;
    for (var tgt : targets) {
      var tagPose = LAYOUT.getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
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

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }
}
