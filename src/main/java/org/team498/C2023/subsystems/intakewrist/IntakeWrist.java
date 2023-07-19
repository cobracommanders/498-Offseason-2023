package org.team498.C2023.subsystems.intakewrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.Robot;
import org.team498.C2023.State;

public class IntakeWrist extends SubsystemBase {
    private final IntakeWristIO IO;
    private final IntakeWristIOInputsAutoLogged inputs = new IntakeWristIOInputsAutoLogged();

    private IntakeWrist() {
        IO = switch (Constants.mode) {
            case REAL, REPLAY -> new IntakeWristIONEO();
            case SIM -> new IntakeWristIO() {};
        };

        IO.setPosition(State.IntakeWrist.IDLE_IN.position);
        IO.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.getInstance().processInputs("IntakeWrist", inputs);

        Robot.intakeWristMechanism.setAngle(inputs.angle * 360 - 20);
        // IO.setBrakeMode(RobotState.isEnabled());

        // SmartDashboard.putData(this);
        // SmartDashboard.putBoolean("IntakeWrist at Setpoint", atSetpoint());
        // SmartDashboard.putNumber("IntakeWrist Error", inputs.targetAngle - inputs.angle);
        // SmartDashboard.putNumber("IntakeWrist Angle", inputs.angle);
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.targetAngle - inputs.angle) < 0.05;
        // return true;
    }

    public void setState(State.IntakeWrist state) {
        IO.setPosition(state.position);
        Logger.getInstance().recordOutput("IntakeWrist/State", state.name());
    }

    public boolean checkEncoderConnection() {
        return inputs.rawAbsoluteEncoder != 0.0;
    }

    private static IntakeWrist instance;

    public static IntakeWrist getInstance() {
        if (instance == null) {
            instance = new IntakeWrist();
        }
        return instance;
    }
}
