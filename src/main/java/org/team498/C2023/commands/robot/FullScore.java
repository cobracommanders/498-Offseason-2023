package org.team498.C2023.commands.robot;

import org.team498.C2023.RobotState;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FullScore extends SequentialCommandGroup {
    public FullScore() {
        super(
            new PrepareToScore(),
            new ConditionalCommand(new WaitCommand(0.5), new WaitCommand(.5), () -> RobotState.getInstance().inConeMode()),
            new Score());
    }
}
