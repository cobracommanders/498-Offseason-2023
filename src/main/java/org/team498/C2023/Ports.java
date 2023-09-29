package org.team498.C2023;

public final class Ports {

    public static class Drivetrain {
        public static final int FL_DRIVE = 1;
        public static final int FR_DRIVE = 5;
        public static final int BL_DRIVE = 3;
        public static final int BR_DRIVE = 7;

        public static final int FL_STEER = 2;
        public static final int FR_STEER = 6;
        public static final int BL_STEER = 4;
        public static final int BR_STEER = 8;

        public static final int FL_CANCODER = 11;
        public static final int FR_CANCODER = 13;
        public static final int BL_CANCODER = 12;
        public static final int BR_CANCODER = 14;

        public static final int GYRO = 15;
    }

    public static class IntakeRollers {
        public static final int TOP_ROLLER = 20;
        public static final int BOTTOM_ROLLER = 23;
        public static final int THIRD_ROLLER = 24;
    }

    public static class IntakeWrist {
        public static final int L_WRIST = 21;
        public static final int R_WRIST = 22;
        public static final int ENCODER_PORT = 1; // DIO
    }

    public static class Elevator {
        public static final int F_ELEVATOR_ID = 30;
        public static final int B_ELEVATOR_ID = 31;

        public static final int ENCODER_PORT = 4; // DIO
    }

    public static class Manipulator {
        public static final int L_ROLLERS = 40;
        public static final int R_ROLLERS = 41; 
    }

    public static class ElevatorWrist {
        public static final int WRIST = 50;
        public static final int ENCODER_PORT = 0; // DIO
    }

    public static class Accessories {
        public static final int BLINKIN = 7; // PWM
        public static final int SETUP_SWITCH = 8; //DIO
        public static final String DriveBus = "bus"; //Canivore's name
        public static final String RioBus =  "rio"; //Name of the native bus on the roborio
    }
}
