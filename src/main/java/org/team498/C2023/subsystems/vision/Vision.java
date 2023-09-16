/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.team498.C2023.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.team498.C2023.Constants;
import org.team498.C2023.FieldPositions;
import org.team498.C2023.Constants.Mode;
import org.team498.lib.photonvision.PhotonPoseEstimator;
import org.team498.lib.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team498.lib.util.PoseUtil;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

// Credit to team 8177 for inspiring this class
public class Vision extends SubsystemBase implements VisionIO {
    private final VisionIO rightCamera;
    private final VisionIO leftCamera;
    private VisionIOInputs rightInputs;
    private VisionIOInputs leftInputs;
    private PhotonPoseEstimator rightPoseEstimator;
    private PhotonPoseEstimator leftPoseEstimator;

    private final Transform3d rightCameraPose = new Transform3d(
            new Translation3d(Units.inchesToMeters(2.5),
                    -Units.inchesToMeters(5.6875),
                    Units.inchesToMeters(22)),
            new Rotation3d());

            private final Transform3d leftCameraPose = new Transform3d(
            new Translation3d(Units.inchesToMeters(2.5),
                    Units.inchesToMeters(5.6875),
                    Units.inchesToMeters(22)),
            new Rotation3d());

    private Vision() {
        PhotonCamera.setVersionCheckEnabled(false);
        rightCamera = new VisionIOSingleCamera("Arducam_OV9281_USB_Camera");
        leftCamera = new VisionIOSingleCamera("Arducam_OV9281_USB_Camera_2");
        rightInputs = new VisionIOInputs();
        leftInputs = new VisionIOInputs();

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            rightPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP,
                    rightCameraPose);
            rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            leftPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP,
                    leftCameraPose);
            leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            rightPoseEstimator = null;
        }
    }

    /**
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create the estimate
     */
    public Optional<EstimatedRobotPose> rightEstimatedPose() {
        if (rightPoseEstimator == null || !rightInputs.enabled) {
            return Optional.empty();
        }

        // photonPoseEstimator.setReferencePose(Drivetrain.getInstance().getPose());
        PhotonPipelineResult result = rightPipelineResult();

        var estimatedPosition = rightPoseEstimator.update(
                result,
                rightInputs.cameraMatrixData,
                rightInputs.distanceCoefficientData);

        if (estimatedPosition.isPresent()) {
            List<PhotonTrackedTarget> targets = estimatedPosition.get().targetsUsed;

            if (targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.05) {
                return Optional.empty();
            }

           
        }
        return estimatedPosition;
    }

    public Optional<EstimatedRobotPose> leftEstimatedPose() {
        if (leftPoseEstimator == null || !leftInputs.enabled) {
            return Optional.empty();
        }

        // photonPoseEstimator.setReferencePose(Drivetrain.getInstance().getPose());
        PhotonPipelineResult result = leftPipelineResult();

        var estimatedPosition = leftPoseEstimator.update(
                result,
                leftInputs.cameraMatrixData,
                leftInputs.distanceCoefficientData);

        if (estimatedPosition.isPresent()) {
            List<PhotonTrackedTarget> targets = estimatedPosition.get().targetsUsed;

            if (targets.size() == 1 && targets.get(0).getPoseAmbiguity() > 0.05) {
                return Optional.empty();
            }

           
        }
        return estimatedPosition;
    }

    // x, y
    private double[][] targetCornerToDoubleArray(List<TargetCorner> corners) {
        var output = new double[2][corners.size()];
        for (int i = 0; i < corners.size(); i++) {
            TargetCorner corner = corners.get(i);
            output[0][i] = corner.x;
            output[1][i] = corner.y;
        }
        return output;
    }

    public PhotonPipelineResult rightPipelineResult() {
        PhotonPipelineResult result = new PhotonPipelineResult();
        if (rightInputs.targetData.length != 0) {
            result.createFromPacket(new Packet(rightInputs.targetData));
            result.setTimestampSeconds(rightInputs.targetTimestamp);
        }
        return result;
        
    }

    public PhotonPipelineResult leftPipelineResult() {
        PhotonPipelineResult result = new PhotonPipelineResult();
        if (leftInputs.targetData.length != 0) {
            result.createFromPacket(new Packet(leftInputs.targetData));
            result.setTimestampSeconds(leftInputs.targetTimestamp);
        }
        return result;
    }

    @Override
    public void periodic() {
        rightCamera.updateInputs(rightInputs);
        leftCamera.updateInputs(leftInputs);
    }
    public void setEnabled(boolean right, boolean left) {
        rightInputs.enabled = right;
        leftInputs.enabled = left;
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(){
        Optional<EstimatedRobotPose> lPose2d = leftEstimatedPose();
        Optional<EstimatedRobotPose> rPose2d = rightEstimatedPose();
        if (lPose2d.isEmpty() && rPose2d.isEmpty()) {
            return Optional.empty();
        } else if (lPose2d.isEmpty()) {
            return rPose2d;
        } else if (rPose2d.isEmpty()) {
            return lPose2d;
        } else{
            Transform2d left = new Transform2d(new Pose2d(), toPose2d(lPose2d.get()));
            Transform2d right = new Transform2d(new Pose2d(), toPose2d(rPose2d.get()));
            Transform2d avg = right.plus(left).div(2);
            List<PhotonTrackedTarget> targets = List.copyOf(lPose2d.get().targetsUsed);
            targets.addAll(rPose2d.get().targetsUsed);
            return Optional.of(new EstimatedRobotPose(new Pose3d(PoseUtil.toPose2d(avg)), lPose2d.get().timestampSeconds, targets));
        }
    }
    private Pose2d toPose2d(EstimatedRobotPose pose) {
        Pose3d initial = pose.estimatedPose;
        return PoseUtil.toPose2d(initial);
    }

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}