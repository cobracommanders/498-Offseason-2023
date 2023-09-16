package org.team498.C2023.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

// Credit to team 8177 for inspiring this class
public class VisionIOSingleCamera implements VisionIO {
    private final PhotonCamera photonCamera; 
         
    public VisionIOSingleCamera(String name) {
        photonCamera = new PhotonCamera(name);
    }
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if(inputs.enabled){
            inputs.connected = photonCamera.isConnected();
            PhotonPipelineResult latestResult = photonCamera.getLatestResult();
            inputs.targetData = latestResult.populatePacket(new Packet(latestResult.getPacketSize())).getData();
            inputs.targetTimestamp = photonCamera.getLatestResult().getTimestampSeconds();

            var matrixData = photonCamera.getCameraMatrix();
            inputs.cameraMatrixData = matrixData.isPresent() ? matrixData.get().getData() : new double[] {};

            var distanceCoefficient = photonCamera.getDistCoeffs();
            inputs.distanceCoefficientData = distanceCoefficient.isPresent() ? distanceCoefficient.get().getData() : new double[] {};
        }
    }

}