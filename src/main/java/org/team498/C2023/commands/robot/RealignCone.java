package org.team498.C2023.commands.robot;

import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RealignCone extends SequentialCommandGroup {
    public RealignCone() {
        super(
            new SetRobotState(State.MID_CONE),
            new SetManipulatorToNextState(),
            new WaitCommand(0.5),
            new SetRobotState(State.DOUBLE_SS),
            new SetManipulatorToNextState(),
            new SetRobotState(State.IDLE_CONE),
            new SetManipulatorToNextState()
        );
    }
}
