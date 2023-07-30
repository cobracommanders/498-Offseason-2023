package org.team498.C2023.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team498.C2023.*;
import org.team498.C2023.subsystems.intakewrist.IntakeWrist;

public class Elevator extends SubsystemBase {
    private final ElevatorIO IO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private State.Elevator state;

    // Offsets all the elevator setpoints during a match to adjust scoring if necessary
    private double onTheFlyOffset = 0.0;

    private Elevator() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY -> new ElevatorIOFalcon500();
            // case REAL, REPLAY -> new ElevatorIO() {};
            case SIM -> new ElevatorIOSim();
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        //Logger.getInstance().processInputs("Elevator", inputs);
        IO.setBrakeMode(RobotState.isEnabled());

        Robot.elevatorMechanism.setLength(inputs.positionMeters);
        //Logger.getInstance().recordOutput("Elevator/Stage 1 Pose", getStageOnePose());
        //Logger.getInstance().recordOutput("Elevator/Stage 2 Pose", getStageTwoPose());
        //SmartDashboard.putNumber("elevatorposition", inputs.positionMeters);

        if (state == State.Elevator.SHOOT_DRIVE_CUBE_MID || state == State.Elevator.SHOOT_DRIVE_CUBE_TOP || state == State.Elevator.SHOOT_DRIVE_CONE_MID) {
            IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        }
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetPositionMeters - inputs.positionMeters) < 0.05;
    }

    public void setManual(double speed) {
        IO.setManual(speed);
    }

    public void setState(State.Elevator state) {
        //if (IntakeWrist.getInstance().isElevatorSafe()) {
            IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
            Logger.getInstance().recordOutput("Elevator/State", state.name());
            this.state = state;
        //}
    }

    public Pose3d getStageOnePose() {
        double x1 = 0;
        double z1 = inputs.positionMeters * 0.5;
        double theta = 60;

        double L = Math.sqrt(Math.pow(x1, 2) + Math.pow(z1, 2));

        double x = x1 + L * Math.sin(Math.toRadians(theta));
        double y = z1 + L * Math.cos(Math.toRadians(theta));

        return new Pose3d(
                new Translation3d(x - 0.3303863, 0, y + 0.186215),
                new Rotation3d(Math.toRadians(90), Math.toRadians(0), Math.toRadians(90))
        );
    }

    public Pose3d getStageTwoPose() {
        double x1 = 0;  // initial x-coordinate of top of pole
        double z1 = (inputs.positionMeters * 0.5) + (inputs.positionMeters * 0.45);
        double theta = 60;

        double L = Math.sqrt(Math.pow(x1, 2) + Math.pow(z1, 2));

        double x = x1 + L * Math.sin(Math.toRadians(theta));
        double y = z1 + L * Math.cos(Math.toRadians(theta));

        return new Pose3d(
                new Translation3d(x - 0.2724059, 0, y + 0.286386),
                new Rotation3d(Math.toRadians(120), Math.toRadians(0), Math.toRadians(90))
        );
    }

    public boolean aboveIntakeHeight() {
        return inputs.positionMeters > 0.25; //TODO find out the real number here
    }

    public void incrementOffset(double increment) {
        onTheFlyOffset += increment;
        Logger.getInstance().recordOutput("Elevator/OnTheFlyOffset", onTheFlyOffset);
        IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
    }

    private double getSetpoint(State.Elevator state, double interpolatedValue) {
        SmartDashboard.putNumber("Interpolation Value", interpolatedValue);
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.elevatorHeight.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        }
                + onTheFlyOffset;
    }

    public void setBrakeMode(boolean enable) {
        IO.setBrakeMode(enable);
    }

    public void updateInitialPosition(boolean inAutoPose) {
        IO.updateInitialPosition(inAutoPose);
    }

    public boolean checkEncoderConnection() {
        return inputs.rawAbsoluteEncoder != 0.0;
    }


    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}
