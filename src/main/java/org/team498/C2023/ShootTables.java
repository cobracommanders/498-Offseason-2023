package org.team498.C2023;

import org.team498.lib.util.LinearInterpolator;

public class ShootTables {
    public static class midCube {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                {1, 0.5},
                {1.8, 1}
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                {1, -0.04},
                {1.8, 0.0625},
                });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                {1, 0.5},
                {1.8, 0.5},
                });
    }

    public static class topCube {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                {1.43, 0.5},
                {2, 1}
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                {1.43, -0.02},
                {2, 0.1}
        });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                {1.43, 0.885},
                {2, 0.885}
        });
    }

    public static class midCone {
        public static final LinearInterpolator shooterRPM = new LinearInterpolator(new double[][] {
                {1.5, 0.5},
                {1.6, 0.6},
                {1.7, 0.7},
                {1.8, 0.8},
                {1.9, 0.9}
        });
        public static final LinearInterpolator wristAngle = new LinearInterpolator(new double[][] {
                {1.5, -0.1},
                {1.6, -0.05},
                {1.7, 0},
                {1.8, 0.05},
                {1.9, 0.1}
        });
        public static final LinearInterpolator elevatorHeight = new LinearInterpolator(new double[][] {
                {1.5, 0.5},
                {1.6, 0.6},
                {1.7, 0.7},
                {1.8, 0.8},
                {1.9, 0.875}
        });
    }
}
