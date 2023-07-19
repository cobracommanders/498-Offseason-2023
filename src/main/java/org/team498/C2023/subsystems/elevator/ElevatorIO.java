package org.team498.C2023.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double targetPositionMeters = 0.0;

        public double rawAbsoluteEncoder = 0.0;

        public double frontAppliedVolts = 0.0;
        public double frontCurrentAmps = 0.0;
        public double frontTemp = 0.0;
        public double frontRawEncoder = 0.0;

        public double backAppliedVolts = 0.0;
        public double backCurrentAmps = 0.0;
        public double backTemp = 0.0;
        public double backRawEncoder = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setPosition(double position) {}
    default void setManual(double speed) {}
    default void updateInitialPosition(boolean inAutoPose) {}
    default void setBrakeMode(boolean enable) {}
}
