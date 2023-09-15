package org.team498.C2023.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

// Credit to team 8177 for inspiring this class
public interface VisionIO {
    class VisionIOInputs {
        public boolean connected = false;
        public byte[] targetData = {};
        public double targetTimestamp = 0.0;
        public double[] cameraMatrixData = {};
        public double[] distanceCoefficientData = {};
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
