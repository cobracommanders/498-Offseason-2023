package org.team498.C2023.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;

public class SetRobotState extends InstantCommand {
    private final State state;
    public SetRobotState(State state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        RobotState.getInstance().setState(state);
    }
}
