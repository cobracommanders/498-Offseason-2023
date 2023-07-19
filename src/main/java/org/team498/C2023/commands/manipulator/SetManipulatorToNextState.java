package org.team498.C2023.commands.manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.manipulator.Manipulator;

public class SetManipulatorToNextState extends InstantCommand {
    private final Manipulator manipulator = Manipulator.getInstance();
    public SetManipulatorToNextState() {
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        manipulator.setState(RobotState.getInstance().getState().manipulator);
    }
}
