package org.team498.C2023.commands.intakewrist;

import edu.wpi.first.wpilibj2.command.Command;

import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.intakewrist.IntakeWrist;

public class SetIntakeWristToNextState extends Command {
    private final IntakeWrist intake = IntakeWrist.getInstance();

    public SetIntakeWristToNextState() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(RobotState.getInstance().getState().intakeWrist);
    }

    @Override
    public boolean isFinished() {
        return intake.atSetpoint();
    }
}
