package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.subsystems.elevator.Elevator;

public class ReturnToIdle extends ConditionalCommand {
    public ReturnToIdle() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SetRobotState(State.TRAVEL_CONE),
                                        new SetRobotState(State.TRAVEL_CUBE),
                                        () -> RobotState.getInstance().inConeMode()),
                                new SetIntakeRollersToNextState(),
                                new SetManipulatorToNextState(),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()),
                                new ConditionalCommand(
                                        new SetRobotState(State.IDLE_CONE),
                                        new SetRobotState(State.IDLE_CUBE),
                                        () -> RobotState.getInstance().inConeMode()),
                                new SetElevatorToNextState(),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()),
                                new SetIntakeRollersToNextState(),
                                new SetManipulatorToNextState()),
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SetRobotState(State.TRAVEL_CONE),
                                        new SetRobotState(State.TRAVEL_CUBE),
                                        () -> RobotState.getInstance().inConeMode()),
                                new SetIntakeRollersToNextState(),
                                new SetManipulatorToNextState(),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState(),
                                        new SetElevatorToNextState()),
                                new ConditionalCommand(
                                        new SetRobotState(State.IDLE_CONE),
                                        new SetRobotState(State.IDLE_CUBE),
                                        () -> RobotState.getInstance().inConeMode()),
                                new SetElevatorToNextState(),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()),
                                new SetIntakeRollersToNextState(),
                                new SetManipulatorToNextState()),
                        () -> false),//RobotState.getInstance().getNextScoringOption() == ScoringOption.TOP),

                new SequentialCommandGroup(
                        new ConditionalCommand(new SetRobotState(State.IDLE_CONE),
                                new SetRobotState(State.IDLE_CUBE),
                                () -> RobotState.getInstance().inConeMode()),
                        new ParallelCommandGroup(
                                new SetElevatorToNextState(),
                                new SetElevatorWristToNextState(),
                                new SetIntakeWristToNextState()),
                        new SetIntakeRollersToNextState(),
                        new SetManipulatorToNextState()),
                () -> Elevator.getInstance().aboveIntakeHeight()); // TODO add command for travelling
                                                                   // down while empty
    }
}
