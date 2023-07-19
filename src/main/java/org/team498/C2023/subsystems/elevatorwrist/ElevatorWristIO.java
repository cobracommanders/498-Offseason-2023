package org.team498.C2023.subsystems.elevatorwrist;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorWristIO {
    @AutoLog
    class ElevatorWristIOInputs {
        public double angle = 0.0;
        public double targetAngle = 0.0;

        public double rawAbsoluteEncoder = 0.0;

        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorTemp = 0.0;
    }

    default void updateInputs(ElevatorWristIOInputs inputs) {}

    default void setPosition(double position) {}
    default void setManual(double speed) {}
    default void setBrakeMode(boolean enable) {}
}
