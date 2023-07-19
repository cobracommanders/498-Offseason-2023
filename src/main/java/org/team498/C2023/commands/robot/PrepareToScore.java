package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.*;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.SetRobotToNextScoringState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;
import org.team498.C2023.subsystems.elevator.Elevator;

public class PrepareToScore extends SequentialCommandGroup {
    public PrepareToScore() {
        super(
                new ConditionalCommand(new SetRobotState(State.TRAVEL_CONE), new SetRobotState(State.TRAVEL_CUBE),
                        () -> RobotState.getInstance().inConeMode()),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetIntakeWristToNextState(),
                        new SetIntakeRollersToNextState(),
                        new SequentialCommandGroup(
                                new WaitCommand(0.15),
                                new SetRobotToNextScoringState(),
                                new ParallelCommandGroup(
                                        new SetElevatorToNextState(),
                                        new SetElevatorWristToNextState()),
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight()
                                        || Elevator.getInstance().atSetpoint()))),
                new SetIntakeWristToNextState(),
                new SetIntakeRollersToNextState()
        );
    }
}
