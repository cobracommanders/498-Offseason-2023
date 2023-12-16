package org.team498.C2023;

public final class Ports {

    public static class DrivetrainPorts {
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

    public static class Accessories {
        public static final int BLINKIN = 9; // PWM
        public static final int SETUP_SWITCH = 8; //DIO
        public static final String DriveBus = "bus"; //Canivore's name
        public static final String RioBus =  "rio"; //Name of the native bus on the roborio
    }
}

