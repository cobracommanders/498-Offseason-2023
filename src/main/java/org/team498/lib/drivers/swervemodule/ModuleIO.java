package org.team498.lib.drivers.swervemodule;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double positionMeters = 0.0;
        public double speedMetersPerSecond = 0.0;
        public double angle = 0.0;

        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTemp = 0.0;

        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;
        public double steerTemp = 0.0;
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setState(SwerveModuleState state) {}
    default void updateIntegratedEncoder() {}
    default void setBrakeMode(boolean enable) {}  
    default String getName() {return "";}
}
