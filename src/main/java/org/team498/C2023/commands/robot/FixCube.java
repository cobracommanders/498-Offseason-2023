package org.team498.C2023.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team498.C2023.State;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.elevatorwrist.SetElevatorWristToNextState;
import org.team498.C2023.commands.intakewrist.SetIntakeWristToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FixCube extends RepeatCommand {
    public FixCube() {
        super(
                new SequentialCommandGroup(
                        new SetRobotState(State.UNSTICK_CUBE),
                        new SetManipulatorToNextState(),
                        new ParallelCommandGroup(
                                new SetIntakeWristToNextState(),
                                new SetElevatorWristToNextState()
                        ),
                        new SetElevatorToNextState(),
                        new SetRobotState(State.CONEARISER_CUBE),
                        new SetManipulatorToNextState(),
                        new SetElevatorWristToNextState(),
                        new SetElevatorToNextState(),
                        new SetIntakeWristToNextState(),
                        new WaitCommand(0.5)));
    }
}
