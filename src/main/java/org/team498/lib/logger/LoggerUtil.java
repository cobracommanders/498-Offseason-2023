package org.team498.lib.logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class LoggerUtil {

    public static void recordOutput(String key, SwerveModuleState... value) {
        double[] data = new double[value.length * 2];
        for (int i = 0; i < value.length; i++) {
          data[i * 2] = value[i].angle.getDegrees();
          data[i * 2 + 1] = value[i].speedMetersPerSecond;
        }
        Logger.getInstance().recordOutput(key, data);
    }
}
