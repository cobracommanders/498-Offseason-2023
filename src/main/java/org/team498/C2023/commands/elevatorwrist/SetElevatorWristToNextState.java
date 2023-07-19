package org.team498.C2023.commands.elevatorwrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.elevatorwrist.ElevatorWrist;

public class SetElevatorWristToNextState extends CommandBase {
    private final ElevatorWrist wrist = ElevatorWrist.getInstance();

    public SetElevatorWristToNextState() {
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setState(RobotState.getInstance().getState().elevatorWrist);
    }

    @Override
    public boolean isFinished() {
        return wrist.atSetpoint();
    }
}

