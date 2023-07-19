package org.team498.C2023.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team498.C2023.RobotState;

public class SetRobotToNextScoringState extends InstantCommand {
    @Override
    public void initialize() {
        RobotState.getInstance().setState(RobotState.getInstance().getNextScoringState());
    }
}
