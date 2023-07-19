package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;


public class GroundIntake extends ParallelCommandGroup {
    public GroundIntake() {
        super(
                new SetElevatorToNextState(),
                new SetElevatorWristToNextState(),
                new SetManipulatorToNextState(),
                new SetIntakeWristToNextState(),
                new SetIntakeRollersToNextState()
        );
    }
}