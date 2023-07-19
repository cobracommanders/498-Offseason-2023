package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.*;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.subsystems.elevator.Elevator;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;

public class CollectFromSS extends SequentialCommandGroup {
    public CollectFromSS() {
        super(
                new SetRobotState(State.TRAVEL_EMPTY),
                new SetElevatorWristToNextState(),
                new SetIntakeWristToNextState(),
                new ConditionalCommand(new SetRobotState(State.DOUBLE_SS), new SetRobotState(State.SINGLE_SS), () -> RobotState.getInstance().inConeMode()),
                new ParallelCommandGroup(
                        new SetElevatorToNextState(),
                        new SetManipulatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight() || Elevator.getInstance().atSetpoint()),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()))));
    }
}
