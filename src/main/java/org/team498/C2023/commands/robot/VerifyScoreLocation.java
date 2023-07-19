package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.*;

import org.team498.C2023.commands.SetRobotToNextScoringState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.subsystems.elevator.Elevator;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakerollers.SetIntakeRollersToNextState;

public class VerifyScoreLocation extends SequentialCommandGroup {
    public VerifyScoreLocation() {
        super(
                //new ConditionalCommand(new SetRobotState(State.TRAVEL_CONE), new SetRobotState(State.TRAVEL_CUBE), () -> RobotState.getInstance().inConeMode()),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetIntakeWristToNextState(),
                        new SetIntakeRollersToNextState(),
                        new SetElevatorWristToNextState()
                ),
                new SetRobotToNextScoringState(),
                new ParallelCommandGroup(
                        new SetElevatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight() || Elevator.getInstance().atSetpoint()),
                                new ParallelCommandGroup(
                                        new SetElevatorWristToNextState(),
                                        new SetIntakeWristToNextState()))));
    }
}
