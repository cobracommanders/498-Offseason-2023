package org.team498.C2023.commands.auto;

import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.RobotState.GameMode;
import org.team498.C2023.RobotState.ScoringOption;
import org.team498.C2023.commands.SetRobotState;
import org.team498.C2023.commands.drivetrain.LockWheels;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.drivetrain.chargestation.AutoEngageBangBang;
import org.team498.C2023.commands.robot.FullScore;
import org.team498.C2023.commands.robot.GroundIntake;
import org.team498.C2023.commands.robot.PrepareToScore;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.commands.robot.Score;
import org.team498.lib.auto.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HighHighCone implements Auto {
    @Override
    public Command getCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CONE)),
                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                new FullScore(),
                new SetRobotState(State.INTAKE),
                new ParallelCommandGroup(
                        new PathPlannerFollower(PathLib.firstNodeToTopCube),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new GroundIntake())),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new PathPlannerFollower(PathLib.topCubeToSecondNode),
                            new LockWheels()
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new ReturnToIdle(),
                                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GameMode.CUBE)),
                                new InstantCommand(() -> RobotState.getInstance().setNextScoringOption(ScoringOption.TOP)),
                                new WaitCommand(1),
                                new PrepareToScore())),
                new ParallelCommandGroup(
                        new Score(),
                        new SequentialCommandGroup(
                                new ConditionalCommand(new WaitCommand(0.3), new WaitCommand(0), () -> RobotState.getInstance().inConeMode())))
                                //new PathPlannerFollower(PathLib.secondNodeToChargeStation)))
                //new AutoEngageBangBang()
                );
    }

    @Override
    public Pose2d getInitialPose() {
        return PathLib.firstNodeToTopCube.getInitialHolonomicPose();
    }

    @Override
    public State getInitialState() {
        return State.IDLE_CUBE;
    }
}
