package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.*;

public class Score extends SequentialCommandGroup {
    public Score() {
        super(
                new SetManipulatorToNextState(),
                new ConditionalCommand(new WaitCommand(0.3), new WaitCommand(0), () -> RobotState.getInstance().inConeMode()),
                new ReturnToIdle());
    }
}
