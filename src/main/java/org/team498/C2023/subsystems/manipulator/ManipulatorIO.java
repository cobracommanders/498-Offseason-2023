package org.team498.C2023.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
    @AutoLog
    class ManipulatorIOInputs {
        public double motorCurrentAmps = 0.0;
        public double motorTemp = 0.0;
        public double velocityRotationsPerSecond = 0.0;
    }

    default void updateInputs(ManipulatorIOInputs inputs) {}

    default void setSpeed(double speed) {}
}
