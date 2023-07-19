package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.drivetrain.LockWheels;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.robot.GroundIntake;
import org.team498.C2023.commands.robot.PrepareToScore;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.commands.robot.Score;
import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HighHighCubeBump implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                new PrepareToScore(),
                new WaitCommand(0.1),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new PathPlannerFollower(PathLib.eighthNodeToFourthCube),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ReturnToIdle(),
                                        new WaitCommand(2)),
                                new SetRobotState(State.INTAKE),
                                new GroundIntake())),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new PathPlannerFollower(PathLib.fourthCubeToFithNode),
                            new LockWheels()
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(.75),
                                new ReturnToIdle(),
                                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                                new WaitCommand(2.5),
                                new PrepareToScore())),
                new WaitCommand(0.2),
                new Score());
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.eighthNodeToFourthCube.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }
}
