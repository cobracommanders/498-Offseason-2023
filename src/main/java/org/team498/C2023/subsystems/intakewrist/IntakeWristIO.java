package org.team498.C2023.subsystems.intakewrist;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO {
    @AutoLog
    class IntakeWristIOInputs {
        public double angle = 0.0;
        public double targetAngle = 0.0;

        public double rawAbsoluteEncoder = 0.0;

        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTemp = 0.0;

        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTemp = 0.0;
    }

    default void updateInputs(IntakeWristIOInputs inputs) {}

    default void setPosition(double position) {}
    default void setBrakeMode(boolean enable) {}
}
