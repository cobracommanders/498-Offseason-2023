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
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;

public class CollectFromSS extends SequentialCommandGroup {
    public CollectFromSS() {
        super(
                // new SetRobotState(State.TRAVEL_CONE),
                // new SetElevatorWristToNextState(),
                // new SetIntakeWristToNextState(),
                // new SetRobotState(State.DOUBLE_SS),//new ConditionalCommand(new SetRobotState(State.DOUBLE_SS), new SetRobotState(State.SINGLE_SS), () -> RobotState.getInstance().inConeMode()),
                // new ParallelCommandGroup(
                //         new SetElevatorToNextState(),
                //         new SetManipulatorToNextState(),
                //         new SequentialCommandGroup(
                //                 new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight() || Elevator.getInstance().atSetpoint()),
                //                 new ParallelCommandGroup(
                //                         new SetElevatorWristToNextState(),
                //                         new SetIntakeWristToNextState()))));
                new SetRobotState(State.TRAVEL_CONE),
                new ParallelCommandGroup(
                        new SetIntakeWristToNextState(),
                        new SequentialCommandGroup(
                                new WaitCommand(0.08),
                                new SetRobotState(State.DOUBLE_SS),
                                new ParallelCommandGroup(
                                        new SetElevatorToNextState(),
                                        new SetManipulatorToNextState(),
                                        new SetElevatorWristToNextState()),
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight()
                                        || Elevator.getInstance().atSetpoint()))),
                new SetRobotState(State.IDLE_CONE),
                new SetIntakeWristToNextState(),
                new SetIntakeRollersToNextState());
    }
}
