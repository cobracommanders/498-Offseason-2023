package org.team498.C2023.subsystems.intakerollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    @AutoLog
    class IntakeRollerIOInputs {
        public double topCurrentAmps = 0.0;
        public double topTemp = 0.0;

        public double bottomCurrentAmps = 0.0;
        public double bottomTemp = 0.0;

        public double thirdCurrentAmps = 0.0;
        public double thirdTemp = 0.0;

        public double velocity = 0.0;
    }

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    default void setSpeed(double bottom, double top, double third) {}
}
