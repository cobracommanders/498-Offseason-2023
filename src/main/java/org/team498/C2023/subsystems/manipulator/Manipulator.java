package org.team498.C2023.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.ShootTables;
import org.team498.C2023.State;

public class Manipulator extends SubsystemBase {
    private final ManipulatorIO IO;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
    private State.Manipulator state;

    private Manipulator() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY -> new ManipulatorIOFALCON();
            case SIM -> new ManipulatorIO() {};
        };
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("Manipulator", inputs);

        if (state == State.Manipulator.SHOOT_DRIVE_CUBE_MID || state == State.Manipulator.SHOOT_DRIVE_CUBE_TOP || state == State.Manipulator.SHOOT_DRIVE_CONE_MID) {
            IO.setSpeed(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        }
    }

    public void setState(State.Manipulator state) {
        IO.setSpeed(getSetpoint(state, RobotPosition.getFutureScoringNodeDistance()));
        Logger.getInstance().recordOutput("Manipulator/State", state.name());
        this.state = state;
    }

    private double getSetpoint(State.Manipulator state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.shooterRPM.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.shooterRPM.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.shooterRPM.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public boolean isStalling() {
        return inputs.motorCurrentAmps > 20;
    }

    public double getCurrentDraw() {
        return inputs.motorCurrentAmps;
    }

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }
}
