package org.team498.C2023;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public enum Mode {
        REAL, SIM, REPLAY
    }

    public static Mode mode = Mode.REAL;

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_ID = 0;
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    public static final class DrivetrainConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 20; //TODO: test for best traction/speed

        public static final double SWERVE_MODULE_DISTANCE_FROM_CENTER = 10.75;

        public static final double MK4I_DRIVE_REDUCTION_L3 = 6.12;
        public static final double MK4I_STEER_REDUCTION_L3 = 21.428571428571428571428571428571; // 150 / 7

        public static final double DRIVE_WHEEL_DIAMETER = 4;
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double FL_MODULE_OFFSET = 315.172266;
        public static final double FR_MODULE_OFFSET = 170.854688;
        public static final double BL_MODULE_OFFSET = 31.811719;
        public static final double BR_MODULE_OFFSET = 26.980078;

        public static final double ROBOT_WIDTH = 26.5 + 6; // Robot width with bumpers, in inches

        public static final class AngleConstants {
            public static final double P = 5;
            public static final double I = 0;
            public static final double D = 0;
            public static final double EPSILON = Constants.mode == Mode.SIM ? 5.0 : 1.0;


            // Constraints for the profiled angle controller
            public static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 720;
            public static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_SPEED_DEGREES_PER_SECOND, 2);

            public static final TrapezoidProfile.Constraints CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND_SQUARED
            );

        }

        public static final class PoseConstants {
            public static final double P = -2.5;
            public static final double I = 0;
            public static final double D = 0;

            public static final double EPSILON = 0.05;
        }
    }

}
