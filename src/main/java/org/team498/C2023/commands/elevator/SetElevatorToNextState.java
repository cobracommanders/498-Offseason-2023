package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.elevator.Elevator;

public class SetElevatorToNextState extends Command {
    private final Elevator elevator = Elevator.getInstance();

    public SetElevatorToNextState() {
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setState(RobotState.getInstance().getState().elevator);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
