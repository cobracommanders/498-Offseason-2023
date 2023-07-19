package org.team498.C2023.commands.robot;

import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Spit extends SequentialCommandGroup {
    public Spit() {
        super(
                new SetRobotState(State.SPIT_CUBE),
                new SetManipulatorToNextState(),
                new WaitCommand(0.75),
                new ReturnToIdle());
    }
}
