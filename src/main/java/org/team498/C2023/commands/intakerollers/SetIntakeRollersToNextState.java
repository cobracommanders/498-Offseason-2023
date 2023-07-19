package org.team498.C2023.commands.intakerollers;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.intakerollers.IntakeRoller;

public class SetIntakeRollersToNextState extends InstantCommand {
    private final IntakeRoller intake = IntakeRoller.getInstance();

    public SetIntakeRollersToNextState() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(RobotState.getInstance().getState().intakeRollers);

    }
}
