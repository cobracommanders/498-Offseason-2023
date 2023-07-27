package org.team498.C2023.subsystems.elevatorwrist;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team498.C2023.*;
import org.team498.C2023.Constants.Mode;
import org.team498.C2023.subsystems.elevator.Elevator;

public class ElevatorWrist extends SubsystemBase {
    private final ElevatorWristIO IO;
    private final ElevatorWristIOInputsAutoLogged inputs = new ElevatorWristIOInputsAutoLogged();
    private State.ElevatorWrist state;

    // Offsets all the wrist setpoints during a match to adjust scoring if necessary
    private double onTheFlyOffset = 0.0;

    private ElevatorWrist() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY -> new ElevatorWristIONEO();
            case SIM -> new ElevatorWristIOSim();
        };
        IO.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("ElevatorWrist", inputs);

        Robot.elevatorWristMechanism.setAngle((inputs.angle - 0.353510) * 360); // - whaatever on 360 based on angle
        SmartDashboard.putNumber("elevator wrist angle", inputs.angle);
        Logger.getInstance().recordOutput("ElevatorWrist/Pose", getPose());

        // IO.setBrakeMode(RobotState.isEnabled());

        if (state == State.ElevatorWrist.SHOOT_DRIVE_CUBE_MID || state == State.ElevatorWrist.SHOOT_DRIVE_CUBE_TOP
                || state == State.ElevatorWrist.SHOOT_DRIVE_CONE_MID) {
            IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        }
    }

    public Pose3d getPose() {
        if (Constants.mode == Mode.SIM) {
            return Elevator.getInstance().getStageTwoPose().transformBy(new Transform3d(
                    new Translation3d(0.0254, 0.130073, 0),
                    new Rotation3d(Math.toRadians(inputs.angle * 360 - 30), Math.toRadians(0), Math.toRadians(0))));
        } else {
            return Elevator.getInstance().getStageTwoPose().transformBy(new Transform3d(
                    new Translation3d(0.0254, 0.130073, 0),
                    new Rotation3d(Math.toRadians(inputs.angle * -360 - 30), Math.toRadians(0), Math.toRadians(0))));
        }
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetAngle - inputs.angle) < 0.02;
    }

    public void setManual(double speed) {
        IO.setManual(speed);
    }

    public void setState(State.ElevatorWrist state) {
        IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        Logger.getInstance().recordOutput("ElevatorWrist/State", state.name());
        this.state = state;
    }

    public void incrementOffset(double increment) {
        onTheFlyOffset += increment;
        Logger.getInstance().recordOutput("ElevatorWrist/OnTheFlyOffset", onTheFlyOffset);
        IO.setPosition(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
    }

    private double getSetpoint(State.ElevatorWrist state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.wristAngle.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        }
                + onTheFlyOffset;
    }

    public void setBrakeMode(boolean enable) {
        IO.setBrakeMode(enable);
    }

    public boolean checkEncoderConnection() {
        return inputs.rawAbsoluteEncoder != 0.0;
    }

    private static ElevatorWrist instance;

    public static ElevatorWrist getInstance() {
        if (instance == null) {
            instance = new ElevatorWrist();
        }
        return instance;
    }
}
