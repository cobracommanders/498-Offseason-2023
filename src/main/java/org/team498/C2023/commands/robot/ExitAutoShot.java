package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

public class ExitAutoShot extends SequentialCommandGroup {
    public ExitAutoShot() {
        super(
                new SetRobotState(State.TRAVEL_EMPTY),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState()
                ),
                new SetRobotState(State.IDLE_CUBE),
                new SetElevatorToNextState(),
                new ParallelCommandGroup(
                        new SetElevatorWristToNextState(),
                        new SetIntakeWristToNextState()
                ),
                new SetIntakeRollersToNextState(),
                new SetManipulatorToNextState()
             );
    }
}
