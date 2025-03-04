// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

import wmironpatriots.Robot;

public class VisionIOComp extends Vision {
    private final PhotonCamera[] cameras;
    private final PhotonCameraSim[] simulatedCams;
    
    private final PhotonPoseEstimator[] estimators;
    private final PhotonPipelineResult[] prevResults;

    public VisionIOComp(CameraConfig[] configs) {
        cameras = new PhotonCamera[configs.length];
        simulatedCams = new PhotonCameraSim[configs.length];
        estimators = new PhotonPoseEstimator[configs.length];
        prevResults = new PhotonPipelineResult[configs.length];

        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new PhotonCamera(configs[i].name());
            simulatedCams[i] = new PhotonCameraSim(cameras[i]);

            estimators[i] = new PhotonPoseEstimator(
                LAYOUT, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                configs[i].loc());
            
            prevResults[i] = new PhotonPipelineResult();
        } 
    }
}
