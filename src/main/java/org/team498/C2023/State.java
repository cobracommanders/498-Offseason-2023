package org.team498.C2023;

public enum State {
    SPIT_CUBE(Elevator.SPIT, ElevatorWrist.SPIT, IntakeWrist.SPIT, IntakeRollers.SPIT, Manipulator.SPIT),
    SLOW_SPIT(Elevator.SPIT, ElevatorWrist.SPIT, IntakeWrist.SPIT, IntakeRollers.SLOW_SPIT, Manipulator.SPIT),

    TOP_CONE(Elevator.TOP_CONE, ElevatorWrist.TOP_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.TOP_CONE),
    TOP_CUBE(Elevator.TOP_CUBE, ElevatorWrist.TOP_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.TOP_CUBE),

    MID_CONE(Elevator.MID_CONE, ElevatorWrist.MID_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.MID_CONE),
    MID_CUBE(Elevator.MID_CUBE, ElevatorWrist.MID_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.MID_CUBE),

    LOW_CONE(Elevator.LOW_CONE, ElevatorWrist.LOW_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.LOW_CONE),

    IDLE_CONE(Elevator.IDLE, ElevatorWrist.IDLE_CONE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.IDLE),
    IDLE_CUBE(Elevator.IDLE, ElevatorWrist.IDLE_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.IDLE),

    OUTTAKE(Elevator.OUTTAKE, ElevatorWrist.OUTTAKE, IntakeWrist.OUTTAKE, IntakeRollers.OUTTAKE, Manipulator.SPIT),

    INTAKE(Elevator.INTAKE, ElevatorWrist.INTAKE, IntakeWrist.INTAKE, IntakeRollers.INTAKE, Manipulator.INTAKE_CUBE),

    UNSTICK_CUBE(Elevator.UNSTICK_CUBE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CUBE, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE),

    CONEARISER_CUBE(Elevator.CONEARISER_CUBE, ElevatorWrist.CONEARISER_CUBE, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE),

    DOUBLE_SS(Elevator.DOUBLE_SS, ElevatorWrist.DOUBLE_SS, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.INTAKE_CONE),
    SINGLE_SS(Elevator.SINGLE_SS, ElevatorWrist.SINGLE_SS, IntakeWrist.IDLE_OUT, IntakeRollers.IDLE, Manipulator.INTAKE_CUBE),

    SHOOT_DRIVE_CUBE_MID(Elevator.SHOOT_DRIVE_CUBE_MID, ElevatorWrist.SHOOT_DRIVE_CUBE_MID, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.SHOOT_DRIVE_CUBE_MID),
    SHOOT_DRIVE_CUBE_TOP(Elevator.SHOOT_DRIVE_CUBE_TOP, ElevatorWrist.SHOOT_DRIVE_CUBE_TOP, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.SHOOT_DRIVE_CUBE_TOP),
    SHOOT_DRIVE_CONE_MID(Elevator.SHOOT_DRIVE_CONE_MID, ElevatorWrist.SHOOT_DRIVE_CONE_MID, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.SHOOT_DRIVE_CONE_MID),

    TRAVEL_EMPTY(Elevator.IDLE, ElevatorWrist.TRAVEL, /*IntakeWrist.IDLE_IN,*/ IntakeWrist.TEMPORARY_IDLE, IntakeRollers.IDLE, Manipulator.IDLE),
    // TRAVEL_CONE(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CONE, IntakeRollers.PUSH, Manipulator.IDLE),
    TRAVEL_CONE(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.TRAVEL_CONE, IntakeRollers.PUSH, Manipulator.IDLE),
    TRAVEL_CUBE(Elevator.IDLE, ElevatorWrist.TRAVEL, IntakeWrist.TEMPORARY_IDLE, IntakeRollers.PUSH, Manipulator.TRAVEL_CUBE),

    AUTO_SHOT(Elevator.AUTO_SHOT, ElevatorWrist.AUTO_SHOT, IntakeWrist.IDLE_IN, IntakeRollers.IDLE, Manipulator.AUTO_SHOT),
    ;

    public final Elevator elevator;
    public final ElevatorWrist elevatorWrist;
    public final IntakeWrist intakeWrist;
    public final IntakeRollers intakeRollers;
    public final Manipulator manipulator;

    State(Elevator elevator, ElevatorWrist elevatorWrist, IntakeWrist intakeWrist, IntakeRollers intakeRollers, Manipulator manipulator) {
        this.elevator = elevator;
        this.elevatorWrist = elevatorWrist;
        this.intakeWrist = intakeWrist;
        this.intakeRollers = intakeRollers;
        this.manipulator = manipulator;
    }

    public enum Elevator {
        CONEARISER_CONE(0),
        CONEARISER_CUBE(0),
        SINGLE_SS(0.16745),
        DOUBLE_SS(0.55),

        SPIT(0),

        MID_CONE(0.6),//.45
        MID_CUBE(0.4),

        LOW_CONE(0.3),

        TOP_CONE(0.58),
        TOP_CUBE( 0.7),

        AUTO_SHOT(0),

        INTAKE(0.05),
        OUTTAKE(0),
        // INTAKE(0),

        IDLE(0),

        UNSTICK_CUBE(0.4),

        SHOOT_DRIVE_CUBE_MID(0),
        SHOOT_DRIVE_CUBE_TOP(0),
        SHOOT_DRIVE_CONE_MID(0);

        public final double setpoint;

        Elevator(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ElevatorWrist {
        SINGLE_SS(0.06),
        DOUBLE_SS(0),

        SPIT(-0.025),

        LOW_CONE(0),

        MID_CONE(.06),
        // MID_CONE(0.1),
        MID_CUBE(-0.04),

        // TOP_CONE(0.25),
        TOP_CONE(.1),
        TOP_CUBE(0),

        TRAVEL(0.041067),

        INTAKE(-0.06),
        // INTAKE(-0.03),
        OUTTAKE(-0.06),

        AUTO_SHOT(0.02),

        IDLE_CUBE(0),
        IDLE_CONE(-0.06),

        CONEARISER_CUBE(-0.065),
        CONEARISER_CONE(0),

        SHOOT_DRIVE_CUBE_MID(0),
        SHOOT_DRIVE_CUBE_TOP(0),
        SHOOT_DRIVE_CONE_MID(0);

        public final double setpoint;

        ElevatorWrist(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum IntakeRollers {
        INTAKE(1, -1, 0.5),
        SPIT(1, 1, 0),
        SLOW_SPIT(0.75, 0.75, 0),
        IDLE(0, 0, 0),
        OUTTAKE(-0.4, 0.4, -0.5),
        PUSH(-0.2, 0, 0);

        public final double bottomRollerSpeed;
        public final double topRollerSpeed;
        public final double thirdRollerSpeed;

        IntakeRollers(double bottomRollerSpeed, double topRollerSpeed, double thirdRollerSpeed) {
            this.bottomRollerSpeed = bottomRollerSpeed;
            this.topRollerSpeed = topRollerSpeed;
            this.thirdRollerSpeed = thirdRollerSpeed;
        }
    }

    public enum IntakeWrist {
        INTAKE(0),
        SPIT(0),
        IDLE_OUT(0),
        TRAVEL_CUBE(0),
        TEMPORARY_IDLE(0),
        TRAVEL_CONE(0),
        IDLE_IN(-0.3),
        OUTTAKE(0.);

        public final double position;

        IntakeWrist(double position) {
            this.position = position;
        }
    }

    public enum Manipulator {
        INTAKE_CONE(-.75),
        INTAKE_CUBE(-.75),

        SPIT(0.9),
        MID_CONE(.1),
        MID_CUBE(0.4),
        TOP_CONE(.9),
        TOP_CUBE(0.4),
        LOW_CONE(0.75),

        AUTO_SHOT(0.5),
        SHOOT_DRIVE_CUBE_MID(0),
        SHOOT_DRIVE_CUBE_TOP(0),
        SHOOT_DRIVE_CONE_MID(0),

        TRAVEL_CUBE(0), //probably needs to be a low value
        IDLE(0);

        public final double setpoint;

        Manipulator(double setpoint) {
            this.setpoint = setpoint;
        }
    }
}